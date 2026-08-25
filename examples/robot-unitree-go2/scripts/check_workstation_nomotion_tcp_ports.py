#!/usr/bin/env python3
"""Fail closed when a workstation no-motion TCP endpoint is already in use.

This helper is intentionally passive: it reads the audited manifest and
``/proc/net/tcp{,6}``, but it never opens a socket and never signals a process.
The proc-net root is injectable so the complete decision can be tested without
examining the live host.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress
from pathlib import Path
import socket
import sys
from typing import Any, Iterable

import yaml


LISTEN_STATE = "0A"
DASHBOARD_PORT_TOKEN = "${GO2_DASHBOARD_PORT}"


class PreflightError(ValueError):
    """The manifest or requested port set is not safe to inspect."""


@dataclass(frozen=True, order=True)
class PortClaim:
    port: int
    owner: str


@dataclass(frozen=True, order=True)
class Listener:
    port: int
    address: str
    family: str
    inode: str


def _port(value: Any, label: str, *, allow_zero: bool = False) -> int:
    if isinstance(value, bool):
        raise PreflightError(f"{label} must be an integer TCP port")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as error:
        raise PreflightError(f"{label} must be an integer TCP port") from error
    minimum = 0 if allow_zero else 1
    if parsed < minimum or parsed > 65535:
        suffix = "0..65535" if allow_zero else "1..65535"
        raise PreflightError(f"{label} must be in {suffix}")
    return parsed


def _endpoint_port(value: Any, label: str) -> int:
    if not isinstance(value, str) or ":" not in value:
        raise PreflightError(f"{label} must be a host:port endpoint")
    host, separator, value_port = value.rpartition(":")
    if not separator or not host:
        raise PreflightError(f"{label} must be a host:port endpoint")
    return _port(value_port, label)


def _named(entries: Any, name: str, section: str) -> dict[str, Any]:
    if not isinstance(entries, list):
        raise PreflightError(f"manifest {section} must be a list")
    matches = [
        entry
        for entry in entries
        if isinstance(entry, dict) and entry.get("name") == name
    ]
    if len(matches) != 1:
        raise PreflightError(
            f"manifest must contain exactly one {section}.{name}"
        )
    return matches[0]


def discover_port_claims(
    manifest: dict[str, Any], dashboard_port: int, semantic_port: int
) -> list[PortClaim]:
    """Return every fixed TCP listener owned by this workstation profile."""

    if not isinstance(manifest, dict) or manifest.get("manifestVersion") != 1:
        raise PreflightError("manifest is not manifestVersion 1")

    claims: list[PortClaim] = []
    system = manifest.get("system")
    if not isinstance(system, dict):
        raise PreflightError("manifest system must be a mapping")
    for name, config in system.items():
        if not isinstance(config, dict):
            raise PreflightError(f"manifest system.{name} must be a mapping")
        if "listen" in config:
            claims.append(
                PortClaim(
                    _endpoint_port(config["listen"], f"system.{name}.listen"),
                    f"system.{name}",
                )
            )
        if "web_port" in config:
            web_port = _port(
                config["web_port"], f"system.{name}.web_port", allow_zero=True
            )
            # Zero asks the provider for an ephemeral port and therefore has
            # no deterministic endpoint that can be preflighted.
            if web_port:
                if name == "scene" and config.get("web_host") != "127.0.0.1":
                    raise PreflightError(
                        "system.scene.web_host must be the literal loopback "
                        "address 127.0.0.1 when its web UI is enabled"
                    )
                claims.append(PortClaim(web_port, f"system.{name}.web"))

    audio = _named(manifest.get("primitive"), "audio_client_bridge", "primitive")
    audio_config = audio.get("config")
    if not isinstance(audio_config, dict):
        raise PreflightError("primitive.audio_client_bridge config is missing")
    claims.append(
        PortClaim(
            _port(
                audio_config.get("listen_port"),
                "primitive.audio_client_bridge.listen_port",
            ),
            "primitive.audio_client_bridge",
        )
    )

    mapping = _named(manifest.get("service"), "mapping", "service")
    mapping_config = mapping.get("config")
    if not isinstance(mapping_config, dict):
        raise PreflightError("service.mapping config is missing")
    claims.append(
        PortClaim(
            _port(mapping_config.get("webui_port"), "service.mapping.webui_port"),
            "service.mapping.webui",
        )
    )

    dashboard = _named(manifest.get("service"), "go2_dashboard", "service")
    dashboard_config = dashboard.get("config")
    if not isinstance(dashboard_config, dict):
        raise PreflightError("service.go2_dashboard config is missing")
    if dashboard_config.get("port") != DASHBOARD_PORT_TOKEN:
        raise PreflightError(
            "service.go2_dashboard.port must remain owned by GO2_DASHBOARD_PORT"
        )
    claims.append(
        PortClaim(
            _port(dashboard_port, "GO2_DASHBOARD_PORT"),
            "service.go2_dashboard",
        )
    )
    claims.append(
        PortClaim(
            _port(semantic_port, "SEMANTIC_INTENT_PORT"),
            "semantic_intent_router",
        )
    )

    by_port: dict[int, list[str]] = {}
    for claim in claims:
        by_port.setdefault(claim.port, []).append(claim.owner)
    collisions = {
        port: sorted(owners) for port, owners in by_port.items() if len(owners) > 1
    }
    if collisions:
        detail = "; ".join(
            f"{port} ({', '.join(owners)})"
            for port, owners in sorted(collisions.items())
        )
        raise PreflightError(f"profile assigns one TCP port to multiple owners: {detail}")
    return sorted(claims)


def _decode_address(encoded: str, family: str) -> str:
    raw = bytes.fromhex(encoded)
    if family == "tcp":
        if len(raw) != 4:
            raise ValueError("invalid IPv4 address width")
        return socket.inet_ntop(socket.AF_INET, raw[::-1])
    if len(raw) != 16:
        raise ValueError("invalid IPv6 address width")
    # Linux renders each 32-bit word in host byte order in /proc/net/tcp6.
    network_order = b"".join(raw[index : index + 4][::-1] for index in range(0, 16, 4))
    return socket.inet_ntop(socket.AF_INET6, network_order)


def parse_proc_net(path: Path, family: str) -> list[Listener]:
    listeners: list[Listener] = []
    try:
        lines = path.read_text(encoding="ascii").splitlines()
    except FileNotFoundError:
        if family == "tcp6":
            return []
        raise PreflightError(f"required proc-net table is missing: {path}")
    except OSError as error:
        raise PreflightError(f"cannot read proc-net table {path}: {error}") from error

    for line_number, line in enumerate(lines[1:], start=2):
        fields = line.split()
        if not fields:
            continue
        if len(fields) < 10:
            raise PreflightError(f"malformed {path}:{line_number}")
        if fields[3].upper() != LISTEN_STATE:
            continue
        encoded_address, separator, encoded_port = fields[1].partition(":")
        if not separator:
            raise PreflightError(f"malformed local endpoint in {path}:{line_number}")
        try:
            port = int(encoded_port, 16)
            address = _decode_address(encoded_address, family)
            ipaddress.ip_address(address)
        except (ValueError, OSError) as error:
            raise PreflightError(
                f"malformed local endpoint in {path}:{line_number}"
            ) from error
        listeners.append(
            Listener(port=port, address=address, family=family, inode=fields[9])
        )
    return listeners


def occupied_claims(
    claims: Iterable[PortClaim], proc_net_root: Path
) -> list[tuple[PortClaim, Listener]]:
    listeners = parse_proc_net(proc_net_root / "tcp", "tcp")
    listeners.extend(parse_proc_net(proc_net_root / "tcp6", "tcp6"))
    by_port: dict[int, list[Listener]] = {}
    for listener in listeners:
        by_port.setdefault(listener.port, []).append(listener)
    return [
        (claim, listener)
        for claim in claims
        for listener in sorted(by_port.get(claim.port, []))
    ]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--dashboard-port", required=True)
    parser.add_argument("--semantic-port", required=True)
    parser.add_argument("--proc-net-root", type=Path, default=Path("/proc/net"))
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        manifest = yaml.safe_load(args.manifest.read_text(encoding="utf-8"))
        claims = discover_port_claims(
            manifest,
            _port(args.dashboard_port, "GO2_DASHBOARD_PORT"),
            _port(args.semantic_port, "SEMANTIC_INTENT_PORT"),
        )
        occupied = occupied_claims(claims, args.proc_net_root)
    except (OSError, yaml.YAMLError, PreflightError) as error:
        print(f"workstation no-motion TCP preflight could not prove safety: {error}", file=sys.stderr)
        return 2

    if occupied:
        print(
            "refusing workstation no-motion startup: required TCP port(s) already listen",
            file=sys.stderr,
        )
        for claim, listener in occupied:
            print(
                f"  {claim.port} ({claim.owner}): "
                f"{listener.family} {listener.address}, inode {listener.inode}",
                file=sys.stderr,
            )
        print("no process was stopped; shut down the owning launch cleanly before retrying", file=sys.stderr)
        return 1

    print(
        "Workstation no-motion TCP port preflight passed: "
        + ", ".join(f"{claim.owner}={claim.port}" for claim in claims)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
