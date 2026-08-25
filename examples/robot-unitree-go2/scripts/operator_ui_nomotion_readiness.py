#!/usr/bin/env python3
"""Passive loopback readiness probes for the no-motion operator UI profile."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import http.client
import json
from pathlib import Path
import socket
import subprocess
import sys
from typing import Any, Sequence


LOOPBACK = "127.0.0.1"


class ProbeError(RuntimeError):
    """A surface is not ready or does not match the no-motion contract."""


@dataclass(frozen=True)
class OperatorEndpoints:
    atlas_port: int = 50051
    scene_port: int = 50107
    mapping_port: int = 8091
    dashboard_port: int = 8092
    client_port: int = 7860


def _validate_port(port: int) -> int:
    if not 1 <= port <= 65535:
        raise ValueError(f"invalid TCP port: {port}")
    return port


def port_has_listener(port: int, *, timeout: float = 0.2) -> bool:
    """Return whether an IPv4 loopback TCP listener accepts connections."""

    _validate_port(port)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        return connection.connect_ex((LOOPBACK, port)) == 0


def _http_get(
    port: int,
    path: str,
    *,
    timeout: float,
) -> tuple[int, str, bytes]:
    _validate_port(port)
    if not path.startswith("/"):
        raise ValueError("HTTP path must be absolute")
    connection = http.client.HTTPConnection(LOOPBACK, port, timeout=timeout)
    try:
        connection.request(
            "GET",
            path,
            headers={"Accept": "application/json, text/html;q=0.9"},
        )
        response = connection.getresponse()
        body = response.read(2 * 1024 * 1024)
        content_type = response.getheader("Content-Type", "")
        return response.status, content_type, body
    except (OSError, http.client.HTTPException) as exc:
        raise ProbeError(f"http://{LOOPBACK}:{port}{path}: {exc}") from exc
    finally:
        connection.close()


def _json_object(port: int, path: str, *, timeout: float) -> dict[str, Any]:
    status, _, body = _http_get(port, path, timeout=timeout)
    if status != 200:
        raise ProbeError(
            f"http://{LOOPBACK}:{port}{path}: unexpected HTTP {status}"
        )
    try:
        payload = json.loads(body)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProbeError(
            f"http://{LOOPBACK}:{port}{path}: response is not JSON"
        ) from exc
    if not isinstance(payload, dict):
        raise ProbeError(f"http://{LOOPBACK}:{port}{path}: expected JSON object")
    return payload


def probe_atlas(rbnx: Path, port: int, *, timeout: float) -> None:
    """Verify both the Atlas listener and its rbnx introspection protocol."""

    if not rbnx.is_file() or not (rbnx.stat().st_mode & 0o111):
        raise ProbeError(f"workspace rbnx is not executable: {rbnx}")
    if not port_has_listener(port, timeout=timeout):
        raise ProbeError(f"Atlas is not listening on {LOOPBACK}:{port}")
    try:
        result = subprocess.run(
            [
                str(rbnx),
                "caps",
                "--json",
                "--server",
                f"{LOOPBACK}:{port}",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=max(1.0, timeout * 4.0),
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ProbeError(f"Atlas introspection failed: {type(exc).__name__}") from exc
    if result.returncode != 0:
        raise ProbeError(f"Atlas introspection returned {result.returncode}")
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise ProbeError("Atlas introspection did not return JSON") from exc
    if not isinstance(payload, list):
        raise ProbeError("Atlas introspection response is not a capability list")


def probe_scene(port: int, *, timeout: float) -> None:
    status, content_type, body = _http_get(port, "/user", timeout=timeout)
    if status != 200:
        raise ProbeError(
            f"http://{LOOPBACK}:{port}/user: unexpected HTTP {status}"
        )
    if "text/html" not in content_type.lower() or not body.strip():
        raise ProbeError(f"http://{LOOPBACK}:{port}/user: expected HTML page")


def probe_mapping(port: int, *, timeout: float) -> None:
    state = _json_object(port, "/api/state", timeout=timeout)
    if "mode" not in state or "has_map" not in state:
        raise ProbeError("Mapping /api/state omitted mode or has_map")


def probe_dashboard(port: int, *, timeout: float) -> None:
    health = _json_object(port, "/healthz", timeout=timeout)
    if health.get("ok") is not True:
        raise ProbeError("Dashboard /healthz did not report ok=true")
    if health.get("telemetry_read_only") is not True:
        raise ProbeError("Dashboard did not assert telemetry_read_only=true")


def probe_client(port: int, atlas_port: int, *, timeout: float) -> None:
    defaults = _json_object(port, "/api/defaults", timeout=timeout)
    if defaults.get("robotHost") != LOOPBACK:
        raise ProbeError("Client defaults do not target loopback robot host")
    if defaults.get("atlasPort") != atlas_port:
        raise ProbeError("Client defaults do not target the audited Atlas port")


def probe_stack(
    endpoints: OperatorEndpoints,
    rbnx: Path,
    *,
    timeout: float,
) -> None:
    probe_atlas(rbnx, endpoints.atlas_port, timeout=timeout)
    probe_scene(endpoints.scene_port, timeout=timeout)
    probe_mapping(endpoints.mapping_port, timeout=timeout)
    probe_dashboard(endpoints.dashboard_port, timeout=timeout)


def probe_full(
    endpoints: OperatorEndpoints,
    rbnx: Path,
    *,
    timeout: float,
) -> None:
    probe_stack(endpoints, rbnx, timeout=timeout)
    probe_client(endpoints.client_port, endpoints.atlas_port, timeout=timeout)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--phase", choices=("stack", "full"))
    action.add_argument("--assert-port-free", type=int)
    parser.add_argument("--rbnx", type=Path)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not 0.05 <= args.timeout <= 5.0:
        print("timeout must be between 0.05 and 5 seconds", file=sys.stderr)
        return 2
    if args.assert_port_free is not None:
        try:
            occupied = port_has_listener(args.assert_port_free, timeout=args.timeout)
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 2
        if occupied:
            print(
                f"loopback port {args.assert_port_free} is already occupied; "
                "no process was stopped",
                file=sys.stderr,
            )
            return 1
        return 0
    if args.rbnx is None:
        print("--rbnx is required for readiness probes", file=sys.stderr)
        return 2
    endpoints = OperatorEndpoints()
    try:
        if args.phase == "stack":
            probe_stack(endpoints, args.rbnx, timeout=args.timeout)
        else:
            probe_full(endpoints, args.rbnx, timeout=args.timeout)
    except (ProbeError, ValueError) as exc:
        if args.verbose:
            print(f"not ready: {exc}", file=sys.stderr)
        return 1
    if args.verbose:
        print(f"operator UI {args.phase} readiness passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
