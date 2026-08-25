#!/usr/bin/env python3
"""Fail-closed, read-only readiness gate for the real Robonix Go2 stack.

The gate only performs filesystem/process inspection, loopback HTTP/TCP
probes, Atlas introspection, and bounded ROS graph/subscription reads.  It
does not call a Robonix task capability, a ROS action, or any robot command.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass, field
import datetime as dt
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import socket
import struct
import subprocess
import sys
import time
from typing import Any, Callable, Iterable, Mapping, Sequence
import urllib.error
import urllib.parse
import urllib.request

import yaml


PASS = "PASS"
FAIL = "FAIL"
UNKNOWN = "UNKNOWN"

DIRECT_BOOT_COMPONENTS = frozenset(
    {
        "atlas",
        "executor",
        "pilot",
        "liaison",
        "soma",
        "scene",
        "mapping",
        "speech",
        "nav2",
        "go2_dashboard",
    }
)

LOOPBACK_PORTS = {
    "atlas": 50051,
    "executor": 50061,
    "pilot": 50071,
    "liaison": 50081,
    "soma": 50091,
}

REQUIRED_CAPABILITIES: dict[str, frozenset[str]] = {
    "executor": frozenset({"robonix/system/executor/execute"}),
    "pilot": frozenset({"robonix/system/pilot"}),
    "liaison": frozenset(
        {
            "robonix/system/liaison/submit",
            "robonix/system/liaison/voice",
        }
    ),
    "soma": frozenset(
        {
            "robonix/system/soma/get_yaml",
            "robonix/system/soma/get_urdf",
        }
    ),
    "scene": frozenset({"robonix/system/scene/goal_near"}),
    "go2_chassis": frozenset(
        {
            "robonix/primitive/chassis/driver",
            "robonix/primitive/chassis/twist_in",
            "robonix/primitive/chassis/odom",
        }
    ),
    "go2_sensors": frozenset(
        {
            "robonix/primitive/lidar/driver",
            "robonix/primitive/lidar/lidar3d",
            "robonix/primitive/imu/imu",
            "robonix/primitive/camera/rgb",
        }
    ),
    "robot_description": frozenset(
        {"robonix/primitive/robot_description/driver"}
    ),
    "audio_client_bridge": frozenset(
        {
            "robonix/primitive/audio/driver",
            "robonix/primitive/audio/mic",
        }
    ),
    "mapping": frozenset(
        {
            "robonix/service/map/driver",
            "robonix/service/map/occupancy_grid",
            "robonix/service/map/lifecycle",
        }
    ),
    "speech": frozenset(
        {
            "robonix/service/speech/driver",
            "robonix/service/speech/asr",
            "robonix/service/speech/asr_stream",
        }
    ),
    "nav2": frozenset(
        {
            "robonix/service/navigation/driver",
            "robonix/service/navigation/navigate",
            "robonix/service/navigation/navigate/status",
            "robonix/service/navigation/navigate/cancel",
        }
    ),
    "go2_dashboard": frozenset(
        {
            "robonix/service/telemetry/dashboard/driver",
            "robonix/service/telemetry/dashboard/status",
        }
    ),
    "semantic_navigation": frozenset(
        {
            "robonix/skill/semantic_navigation/driver",
            "robonix/skill/semantic_navigation/navigate_landmark",
            "robonix/skill/semantic_navigation/navigate_landmark/status",
            "robonix/skill/semantic_navigation/navigate_landmark/cancel",
        }
    ),
}

EXPECTED_PROVIDER_NAMESPACES = {
    "go2_sensors": "robonix/primitive/lidar",
}

# go2_sensors deliberately owns one atomic lifecycle for the lidar relay, IMU
# relay, and camera bridge.  Atlas therefore reports the two sibling-domain
# ROS data contracts below as advisory namespace diagnostics.  Keep this
# deployment exception exact: changing the provider, runtime namespace,
# contract, or transport must fail readiness.
ALLOWED_NAMESPACE_DIAGNOSTICS: frozenset[tuple[str, str, str, str]] = frozenset(
    {
        (
            "go2_sensors",
            "robonix/primitive/lidar",
            "robonix/primitive/imu/imu",
            "ros2",
        ),
        (
            "go2_sensors",
            "robonix/primitive/lidar",
            "robonix/primitive/camera/rgb",
            "ros2",
        ),
    }
)


@dataclass(frozen=True)
class Check:
    check_id: str
    status: str
    detail: str
    evidence: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CommandResult:
    argv: tuple[str, ...]
    returncode: int
    stdout: str
    stderr: str
    timed_out: bool = False


@dataclass(frozen=True)
class TopicRequirement:
    label: str
    topic: str
    message_type: str
    max_age_s: float | None
    expected_frame: str
    transient_local: bool = False


TOPICS = (
    TopicRequirement(
        "camera",
        "/camera/color/image_raw",
        "sensor_msgs/msg/Image",
        2.0,
        "front_camera",
    ),
    TopicRequirement(
        "cloud",
        "/scanner/cloud",
        "sensor_msgs/msg/PointCloud2",
        2.0,
        "utlidar_lidar",
    ),
    TopicRequirement(
        "scan",
        "/scanner/scan",
        "sensor_msgs/msg/LaserScan",
        1.0,
        "base_link",
    ),
    TopicRequirement(
        "imu", "/scanner/imu", "sensor_msgs/msg/Imu", 1.5, "utlidar_imu"
    ),
    TopicRequirement(
        "odom", "/odom", "nav_msgs/msg/Odometry", 1.5, "odom"
    ),
    TopicRequirement(
        "map", "/map", "nav_msgs/msg/OccupancyGrid", None, "map", True
    ),
)

NAV_LIFECYCLE_NODES = (
    "controller_server",
    "planner_server",
    "smoother_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
)

NAV_ACTIONS = {
    "/navigate_to_pose": "nav2_msgs/action/NavigateToPose",
    "/compute_path_to_pose": "nav2_msgs/action/ComputePathToPose",
    "/follow_path": "nav2_msgs/action/FollowPath",
}

CHASSIS_DIAGNOSTICS_TOPIC = "/diagnostics"
CHASSIS_DIAGNOSTIC_NAME = "go2_chassis_adapter"
CHASSIS_MAX_STATE_AGE_S = 0.20
CHASSIS_MAX_SOURCE_STAMP_AGE_S = 0.20
CHASSIS_MAX_SOURCE_FUTURE_SKEW_S = 0.05


class CommandRunner:
    """Run bounded read commands without a shell."""

    def __init__(self, timeout_s: float = 6.0) -> None:
        self.timeout_s = float(timeout_s)

    def run(self, argv: Sequence[str], *, accept_timeout: bool = False) -> CommandResult:
        command = tuple(str(value) for value in argv)
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=self.timeout_s + 2.0,
                env=os.environ.copy(),
            )
            timed_out = completed.returncode in {124, 137}
            if timed_out and not accept_timeout:
                return CommandResult(
                    command,
                    completed.returncode,
                    completed.stdout,
                    completed.stderr,
                    True,
                )
            return CommandResult(
                command,
                completed.returncode,
                completed.stdout,
                completed.stderr,
                timed_out,
            )
        except subprocess.TimeoutExpired as exc:
            return CommandResult(
                command,
                124,
                _decoded(exc.stdout),
                _decoded(exc.stderr),
                True,
            )

    def ros(self, argv: Sequence[str], *, accept_timeout: bool = False) -> CommandResult:
        seconds = max(1, int(math.ceil(self.timeout_s)))
        return self.run(
            ("timeout", "--signal=TERM", f"{seconds}s", *argv),
            accept_timeout=accept_timeout,
        )


def _decoded(value: str | bytes | None) -> str:
    if value is None:
        return ""
    return value.decode("utf-8", "replace") if isinstance(value, bytes) else value


def _check(check_id: str, ok: bool, detail: str, **evidence: Any) -> Check:
    return Check(check_id, PASS if ok else FAIL, detail, evidence)


def _unknown(check_id: str, detail: str, **evidence: Any) -> Check:
    return Check(check_id, UNKNOWN, detail, evidence)


def _load_json_file(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _process_identity(pid: int, expected_pgid: int | None = None) -> tuple[bool, str]:
    if isinstance(pid, bool) or pid <= 1:
        return False, "invalid pid"
    stat_path = Path("/proc") / str(pid) / "stat"
    cmdline_path = Path("/proc") / str(pid) / "cmdline"
    try:
        raw = stat_path.read_text(encoding="utf-8")
        suffix = raw.rsplit(") ", 1)[1].split()
        state = suffix[0]
        process_group = int(suffix[2])
        cmdline = cmdline_path.read_bytes().replace(b"\0", b" ").strip()
    except (OSError, IndexError, ValueError) as exc:
        return False, f"process identity unavailable: {type(exc).__name__}"
    if state == "Z":
        return False, "process is a zombie"
    if not cmdline:
        return False, "process command line is empty"
    if expected_pgid is not None and process_group != expected_pgid:
        return False, f"process group {process_group} != persisted {expected_pgid}"
    return True, "live"


def validate_boot_state(path: Path, deploy_dir: Path) -> Check:
    check_id = "rbnx_state_complete"
    if not path.is_file():
        return _unknown(check_id, f"missing Robonix boot state: {path}")
    try:
        state = _load_json_file(path)
    except (OSError, json.JSONDecodeError) as exc:
        return _unknown(check_id, f"cannot parse boot state: {type(exc).__name__}")
    if not isinstance(state, dict):
        return _unknown(check_id, "boot state is not a JSON object")
    if state.get("failures"):
        return _check(check_id, False, "boot state records failures")
    manifest = Path(str(state.get("manifest_path", "")))
    expected_manifest = (deploy_dir / "robonix_manifest.yaml").resolve()
    try:
        actual_manifest = manifest.resolve(strict=True)
    except OSError:
        return _check(check_id, False, "persisted manifest path does not exist")
    if actual_manifest != expected_manifest:
        return _check(
            check_id,
            False,
            "boot state belongs to another manifest",
            manifest=str(actual_manifest),
        )
    if state.get("atlas_endpoint") != "127.0.0.1:50051":
        return _check(check_id, False, "boot state Atlas endpoint is not exact loopback")
    boot_pid = state.get("boot_pid")
    if not isinstance(boot_pid, int) or isinstance(boot_pid, bool):
        return _unknown(check_id, "boot_pid is absent or invalid")
    alive, reason = _process_identity(boot_pid)
    if not alive:
        return _check(check_id, False, f"rbnx boot process is not live: {reason}")
    components = state.get("components")
    if not isinstance(components, list):
        return _unknown(check_id, "components is not a JSON array")
    names: list[str] = []
    for component in components:
        if not isinstance(component, dict):
            return _unknown(check_id, "component record is not a JSON object")
        name = component.get("name")
        pid = component.get("pid")
        pgid = component.get("pgid")
        if not isinstance(name, str) or not name:
            return _unknown(check_id, "component name is absent")
        if not isinstance(pid, int) or isinstance(pid, bool):
            return _unknown(check_id, f"component {name} has invalid pid")
        if not isinstance(pgid, int) or isinstance(pgid, bool) or pgid != pid:
            return _check(check_id, False, f"component {name} has invalid process group")
        alive, reason = _process_identity(pid, pgid)
        if not alive:
            return _check(check_id, False, f"component {name} is not live: {reason}")
        names.append(name)
    if len(names) != len(set(names)):
        return _check(check_id, False, "boot state has duplicate component names")
    missing = sorted(DIRECT_BOOT_COMPONENTS - set(names))
    if missing:
        return _check(
            check_id,
            False,
            "best-effort boot did not persist every required component",
            missing=missing,
        )
    return _check(
        check_id,
        True,
        "state exists; every direct component is persisted and live",
        component_count=len(names),
        boot_pid=boot_pid,
    )


def _decode_proc_ipv4(hex_address: str) -> ipaddress.IPv4Address:
    packed = struct.pack("<I", int(hex_address, 16))
    return ipaddress.IPv4Address(packed)


def _decode_proc_ipv6(hex_address: str) -> ipaddress.IPv6Address:
    if len(hex_address) != 32:
        raise ValueError("invalid proc IPv6 address")
    packed = b"".join(
        bytes.fromhex(hex_address[index : index + 8])[::-1]
        for index in range(0, 32, 8)
    )
    return ipaddress.IPv6Address(packed)


def listeners_for_port(port: int, proc_net: Path = Path("/proc/net")) -> list[str]:
    listeners: list[str] = []
    for filename, decoder in (("tcp", _decode_proc_ipv4), ("tcp6", _decode_proc_ipv6)):
        path = proc_net / filename
        try:
            lines = path.read_text(encoding="ascii").splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            if len(fields) < 4 or fields[3] != "0A":
                continue
            address_hex, port_hex = fields[1].split(":", 1)
            if int(port_hex, 16) != port:
                continue
            listeners.append(str(decoder(address_hex)))
    return listeners


def validate_loopback_listener(name: str, port: int) -> Check:
    check_id = f"loopback_{name}"
    listeners = listeners_for_port(port)
    if not listeners:
        return _unknown(check_id, f"no TCP listener discovered on port {port}")
    try:
        addresses = [ipaddress.ip_address(value) for value in listeners]
    except ValueError:
        return _unknown(check_id, "listener address could not be decoded")
    non_loopback = [str(value) for value in addresses if not value.is_loopback]
    if non_loopback:
        return _check(
            check_id,
            False,
            f"{name} port is exposed beyond loopback",
            addresses=listeners,
        )
    if ipaddress.ip_address("127.0.0.1") not in addresses:
        return _check(
            check_id,
            False,
            f"{name} has no IPv4 loopback listener required by this deployment",
            addresses=listeners,
        )
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1.0):
            pass
    except OSError as exc:
        return _check(check_id, False, f"loopback TCP connect failed: {type(exc).__name__}")
    return _check(
        check_id,
        True,
        f"{name} listens only on loopback",
        port=port,
        addresses=listeners,
    )


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):  # noqa: ANN001
        return None


def get_json_loopback(url: str, timeout_s: float = 2.0) -> tuple[Any, str]:
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme != "http" or parsed.hostname != "127.0.0.1":
        raise ValueError("HTTP probe URL must use literal 127.0.0.1")
    if parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError("HTTP probe URL contains unsupported authority/query data")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
    request = urllib.request.Request(
        url,
        method="GET",
        headers={"Accept": "application/json", "User-Agent": "go2-readiness/1"},
    )
    with opener.open(request, timeout=timeout_s) as response:
        if response.status != 200:
            raise ValueError(f"HTTP status {response.status}")
        content_type = response.headers.get_content_type()
        if content_type != "application/json":
            raise ValueError(f"unexpected content type {content_type}")
        body = response.read(262145)
        if len(body) > 262144:
            raise ValueError("JSON response exceeds 256 KiB")
    return json.loads(body), url


def validate_router(port: int, expected_model: str) -> Check:
    check_id = "semantic_router_models"
    url = f"http://127.0.0.1:{port}/v1/models"
    try:
        payload, _ = get_json_loopback(url)
    except (OSError, ValueError, json.JSONDecodeError, urllib.error.URLError) as exc:
        return _unknown(check_id, f"router models probe failed: {type(exc).__name__}")
    data = payload.get("data") if isinstance(payload, dict) else None
    ids = {
        item.get("id")
        for item in data
        if isinstance(item, dict) and isinstance(item.get("id"), str)
    } if isinstance(data, list) else set()
    if expected_model not in ids:
        return _check(
            check_id,
            False,
            "router did not advertise the exact configured model",
            advertised=sorted(ids),
        )
    return _check(check_id, True, "semantic router model endpoint is ready", model=expected_model)


def validate_dashboard(port: int) -> Check:
    check_id = "dashboard_status"
    try:
        status, _ = get_json_loopback(f"http://127.0.0.1:{port}/api/status")
        health, _ = get_json_loopback(f"http://127.0.0.1:{port}/healthz")
    except (OSError, ValueError, json.JSONDecodeError, urllib.error.URLError) as exc:
        return _unknown(check_id, f"dashboard probe failed: {type(exc).__name__}")
    if not isinstance(status, dict) or not isinstance(health, dict):
        return _unknown(check_id, "dashboard responses are not JSON objects")
    bridge = status.get("bridge")
    if not isinstance(bridge, dict):
        return _unknown(check_id, "/api/status has no bridge object")
    if bridge.get("connected") is not True or health.get("ros_connected") is not True:
        return _check(check_id, False, "dashboard HTTP is up but ROS observer is disconnected")
    if status.get("telemetry_read_only") is not True or health.get("telemetry_read_only") is not True:
        return _check(check_id, False, "dashboard does not assert telemetry_read_only")
    topics = status.get("topics")
    if not isinstance(topics, dict):
        return _unknown(check_id, "/api/status has no topics object")
    camera_quality = status.get("camera_quality")
    if not isinstance(camera_quality, dict):
        return _check(
            check_id,
            False,
            "dashboard /api/status has no usable camera quality diagnostic",
            camera_quality=camera_quality,
        )
    quality_level = camera_quality.get("level")
    quality_message = camera_quality.get("message")
    quality_rate_hz = camera_quality.get("rate_hz")
    quality_error_ratio = camera_quality.get("quality_error_ratio")
    quality_ready = camera_quality.get("ready")
    quality_healthy = camera_quality.get("healthy")
    level_ok = (
        isinstance(quality_level, int)
        and not isinstance(quality_level, bool)
        and quality_level == 0
    )
    rate_ok = (
        isinstance(quality_rate_hz, (int, float))
        and not isinstance(quality_rate_hz, bool)
        and math.isfinite(float(quality_rate_hz))
        and float(quality_rate_hz) >= 0.0
    )
    ratio_ok = (
        isinstance(quality_error_ratio, (int, float))
        and not isinstance(quality_error_ratio, bool)
        and math.isfinite(float(quality_error_ratio))
        and 0.0 <= float(quality_error_ratio) <= 1.0
    )
    message_ok = quality_message == "camera quality gate passed"
    quality_evidence = {
        "ready": quality_ready,
        "healthy": quality_healthy,
        "level": quality_level,
        "message": quality_message,
        "rate_hz": quality_rate_hz,
        "quality_error_ratio": quality_error_ratio,
    }
    if (
        quality_ready is not True
        or quality_healthy is not True
        or not level_ok
        or not message_ok
        or not rate_ok
        or not ratio_ok
    ):
        return _check(
            check_id,
            False,
            "dashboard camera quality gate is not ready and healthy: "
            f"level={quality_level!r} message={quality_message!r} "
            f"rate_hz={quality_rate_hz!r} "
            f"quality_error_ratio={quality_error_ratio!r}",
            camera_quality=quality_evidence,
        )
    required = ("camera", "point_cloud", "map", "odom", "pose_map")
    not_fresh = {
        name: topics.get(name, {}).get("state")
        for name in required
        if not isinstance(topics.get(name), dict)
        or topics[name].get("state") != "fresh"
    }
    if not_fresh:
        return _check(
            check_id,
            False,
            "dashboard ROS observer has missing/stale required telemetry",
            topics=not_fresh,
        )
    return _check(
        check_id,
        True,
        "dashboard /api/status and /healthz report connected fresh telemetry "
        "with a ready and healthy camera quality gate",
        topics=list(required),
        camera_quality=quality_evidence,
    )


def parse_provider_records(payload: Any) -> dict[str, dict[str, Any]]:
    if not isinstance(payload, list):
        raise ValueError("caps JSON is not an array")
    providers: dict[str, dict[str, Any]] = {}
    for record in payload:
        if not isinstance(record, dict):
            raise ValueError("provider record is not an object")
        provider_id = record.get("provider_id")
        if not isinstance(provider_id, str) or not provider_id:
            raise ValueError("provider_id is absent")
        if provider_id in providers:
            raise ValueError(f"duplicate provider_id {provider_id}")
        providers[provider_id] = record
    return providers


def validate_providers(payload: Any) -> Check:
    check_id = "atlas_required_providers"
    try:
        providers = parse_provider_records(payload)
    except ValueError as exc:
        return _unknown(check_id, str(exc))
    problems: dict[str, Any] = {}
    accepted_namespace_diagnostics: list[dict[str, Any]] = []
    unexpected_namespace_diagnostics: dict[str, list[dict[str, Any]]] = {}
    invalid_namespace_diagnostic_flags: dict[str, list[dict[str, Any]]] = {}

    def record_problem(provider_id: str, key: str, value: Any) -> None:
        existing = problems.get(provider_id)
        if existing is None:
            problems[provider_id] = {key: value}
        elif isinstance(existing, dict):
            existing[key] = value
        else:
            problems[provider_id] = {"readiness": existing, key: value}

    for provider_id, record in providers.items():
        runtime_namespace = record.get("namespace")
        capabilities = record.get("capabilities")
        if not isinstance(capabilities, list):
            continue
        for index, item in enumerate(capabilities):
            if not isinstance(item, dict):
                invalid_namespace_diagnostic_flags.setdefault(
                    provider_id, []
                ).append(
                    {
                        "capability_index": index,
                        "reason": "capability_not_object",
                    }
                )
                continue
            mismatch = item.get("namespace_mismatch")
            if not isinstance(mismatch, bool):
                invalid_namespace_diagnostic_flags.setdefault(
                    provider_id, []
                ).append(
                    {
                        "capability_index": index,
                        "contract_id": item.get("contract_id"),
                        "field_present": "namespace_mismatch" in item,
                        "actual_type": type(mismatch).__name__,
                    }
                )
                continue
            if mismatch is not True:
                continue
            contract_id = item.get("contract_id")
            transport = item.get("transport")
            diagnostic = {
                "provider_id": provider_id,
                "runtime_namespace": runtime_namespace,
                "contract_id": contract_id,
                "transport": transport,
            }
            fields = (provider_id, runtime_namespace, contract_id, transport)
            if (
                all(isinstance(value, str) for value in fields)
                and fields in ALLOWED_NAMESPACE_DIAGNOSTICS
            ):
                accepted_namespace_diagnostics.append(diagnostic)
            else:
                unexpected_namespace_diagnostics.setdefault(provider_id, []).append(
                    diagnostic
                )

    diagnostic_sort_keys = (
        "provider_id",
        "runtime_namespace",
        "contract_id",
        "transport",
    )
    accepted_namespace_diagnostics.sort(
        key=lambda value: tuple(str(value.get(key)) for key in diagnostic_sort_keys)
    )
    for diagnostics in unexpected_namespace_diagnostics.values():
        diagnostics.sort(
            key=lambda value: tuple(
                str(value.get(key)) for key in diagnostic_sort_keys
            )
        )
    for diagnostics in invalid_namespace_diagnostic_flags.values():
        diagnostics.sort(key=lambda value: int(value["capability_index"]))

    for provider_id, required in REQUIRED_CAPABILITIES.items():
        record = providers.get(provider_id)
        if record is None:
            problems[provider_id] = "missing"
            continue
        provider_problems: dict[str, Any] = {}
        expected_namespace = EXPECTED_PROVIDER_NAMESPACES.get(provider_id)
        if (
            expected_namespace is not None
            and record.get("namespace") != expected_namespace
        ):
            provider_problems["provider_namespace"] = {
                "expected": expected_namespace,
                "actual": record.get("namespace"),
            }
        if record.get("state") != "ACTIVE":
            provider_problems.update(
                {
                    "state": record.get("state"),
                    "detail": str(record.get("state_detail", ""))[:200],
                }
            )
            problems[provider_id] = provider_problems
            continue
        capabilities = record.get("capabilities")
        if not isinstance(capabilities, list):
            provider_problems["capabilities"] = "unknown"
            problems[provider_id] = provider_problems
            continue
        found: set[str] = set()
        for item in capabilities:
            if not isinstance(item, dict):
                continue
            contract_id = item.get("contract_id")
            if isinstance(contract_id, str):
                found.add(contract_id)
        missing = sorted(required - found)
        if missing:
            provider_problems["missing_capabilities"] = missing
        if provider_problems:
            problems[provider_id] = provider_problems

    for provider_id, diagnostics in unexpected_namespace_diagnostics.items():
        record_problem(
            provider_id, "unexpected_namespace_diagnostics", diagnostics
        )
    for provider_id, diagnostics in invalid_namespace_diagnostic_flags.items():
        record_problem(
            provider_id, "invalid_namespace_diagnostic_flags", diagnostics
        )

    expected_namespace_diagnostics = [
        {
            "provider_id": provider_id,
            "runtime_namespace": runtime_namespace,
            "contract_id": contract_id,
            "transport": transport,
        }
        for provider_id, runtime_namespace, contract_id, transport in sorted(
            ALLOWED_NAMESPACE_DIAGNOSTICS
        )
    ]
    if accepted_namespace_diagnostics != expected_namespace_diagnostics:
        # The deployment contract is not merely an allowlist.  Atlas must
        # expose both audited sibling-domain diagnostics exactly once.  A
        # missing/false flag or duplicate capability is therefore a failure,
        # even when no third diagnostic is present.
        record_problem(
            "go2_sensors",
            "namespace_diagnostics_exact_set",
            {
                "expected": expected_namespace_diagnostics,
                "actual": accepted_namespace_diagnostics,
            },
        )

    for provider_id, record in providers.items():
        if record.get("state") in {"ERROR", "?", "TERMINATED"}:
            problems.setdefault(provider_id, {"state": record.get("state")})
    if problems:
        return _check(
            check_id,
            False,
            "Atlas provider/capability readiness is incomplete",
            problems=problems,
            accepted_namespace_diagnostics=accepted_namespace_diagnostics,
        )
    return _check(
        check_id,
        True,
        "all required Atlas providers are ACTIVE with exact capabilities "
        "and audited namespace diagnostics",
        provider_count=len(REQUIRED_CAPABILITIES),
        accepted_namespace_diagnostics=accepted_namespace_diagnostics,
    )


def read_atlas_caps(runner: CommandRunner) -> tuple[Check, Any | None]:
    result = runner.run(
        ("rbnx", "caps", "--json", "--server", "127.0.0.1:50051")
    )
    if result.returncode != 0 or result.timed_out:
        return (
            _unknown(
                "atlas_required_providers",
                "rbnx caps could not read Atlas",
                returncode=result.returncode,
            ),
            None,
        )
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError:
        return _unknown("atlas_required_providers", "rbnx caps returned invalid JSON"), None
    return validate_providers(payload), payload


def validate_speech_backend(log_path: Path) -> Check:
    check_id = "speech_asr_backend"
    if not log_path.is_file():
        return _unknown(check_id, f"speech log is missing: {log_path}")
    status_lines: list[str] = []
    try:
        for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            message = record.get("msg") if isinstance(record, dict) else None
            if isinstance(message, str) and "Backend status:" in message:
                status_lines.append(message)
    except OSError as exc:
        return _unknown(check_id, f"speech log cannot be read: {type(exc).__name__}")
    if not status_lines:
        return _unknown(check_id, "current speech log has no backend status record")
    latest = status_lines[-1]
    if "mode=mock" in latest:
        return _check(check_id, False, "mock speech backend is not physical readiness")
    if not re.search(r"asr_stream=.*\(OK\)", latest):
        return _check(check_id, False, "streaming ASR backend is not reported OK")
    return _check(check_id, True, "real streaming ASR backend is reported OK")


def yaml_documents(text: str) -> list[Any]:
    return [document for document in yaml.safe_load_all(text) if document is not None]


def parse_header_document(text: str) -> Mapping[str, Any]:
    documents = yaml_documents(text)
    if len(documents) != 1 or not isinstance(documents[0], dict):
        raise ValueError("expected exactly one YAML header object")
    return documents[0]


def _real_int(value: Any, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"{label} is not an integer")
    return value


def validate_topic_header(
    requirement: TopicRequirement,
    header: Mapping[str, Any],
    now_s: float,
    future_tolerance_s: float = 0.5,
) -> Check:
    check_id = f"ros_topic_{requirement.label}"
    stamp = header.get("stamp")
    if not isinstance(stamp, dict):
        return _unknown(check_id, "message header has no stamp")
    try:
        sec = _real_int(stamp.get("sec"), "stamp.sec")
        nanosec = _real_int(stamp.get("nanosec"), "stamp.nanosec")
    except ValueError as exc:
        return _unknown(check_id, str(exc))
    if sec <= 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        return _check(check_id, False, "message stamp is zero or invalid")
    frame_id = header.get("frame_id")
    if frame_id != requirement.expected_frame:
        return _check(
            check_id,
            False,
            "message frame_id does not match the audited deployment",
            expected=requirement.expected_frame,
            actual=frame_id,
        )
    stamp_s = sec + nanosec / 1_000_000_000.0
    age_s = now_s - stamp_s
    if age_s < -future_tolerance_s:
        return _check(
            check_id,
            False,
            "message stamp is too far in the future",
            age_s=round(age_s, 6),
        )
    if requirement.max_age_s is not None and age_s > requirement.max_age_s:
        return _check(
            check_id,
            False,
            "message is stale",
            age_s=round(age_s, 6),
            max_age_s=requirement.max_age_s,
        )
    detail = (
        "valid event-driven snapshot with exact type/frame/stamp"
        if requirement.max_age_s is None
        else "fresh message with exact type/frame/stamp"
    )
    return _check(
        check_id,
        True,
        detail,
        topic=requirement.topic,
        frame_id=frame_id,
        age_s=round(max(0.0, age_s), 6),
    )


def read_topic(requirement: TopicRequirement, runner: CommandRunner) -> Check:
    argv = [
        "ros2",
        "topic",
        "echo",
        requirement.topic,
        requirement.message_type,
        "--once",
        "--no-daemon",
        "--field",
        "header",
        "--qos-profile",
        "sensor_data",
    ]
    if requirement.transient_local:
        argv.extend(
            [
                "--qos-reliability",
                "reliable",
                "--qos-durability",
                "transient_local",
                "--qos-depth",
                "1",
            ]
        )
    result = runner.ros(argv)
    if result.returncode != 0 or result.timed_out:
        return _unknown(
            f"ros_topic_{requirement.label}",
            "bounded ros2 topic echo received no usable sample",
            topic=requirement.topic,
            returncode=result.returncode,
        )
    try:
        header = parse_header_document(result.stdout)
    except (ValueError, yaml.YAMLError) as exc:
        return _unknown(f"ros_topic_{requirement.label}", str(exc))
    return validate_topic_header(requirement, header, time.time())


def _diagnostic_integer(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{label} is not an integer")
    if isinstance(value, int):
        return value
    if isinstance(value, str) and value.strip().isdecimal():
        return int(value.strip(), 10)
    raise ValueError(f"{label} is not an integer")


def _diagnostic_uint8(value: Any, label: str) -> int:
    """Decode a ROS uint8 scalar without accepting ambiguous text values."""
    if isinstance(value, bool):
        raise ValueError(f"{label} is not a uint8")
    if isinstance(value, int):
        if 0 <= value <= 255:
            return value
        raise ValueError(f"{label} is outside uint8 range")
    if isinstance(value, str) and len(value) == 1 and ord(value) <= 255:
        return ord(value)
    raise ValueError(f"{label} is not a single-character uint8")


def _diagnostic_float(values: Mapping[str, str], key: str) -> float:
    value = values.get(key)
    if not isinstance(value, str):
        raise ValueError(f"{key} is absent")
    try:
        result = float(value)
    except ValueError as exc:
        raise ValueError(f"{key} is not numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{key} is not finite")
    return result


def validate_chassis_health_document(document: Mapping[str, Any]) -> Check:
    """Require fresh chassis diagnostics and a fail-closed marker policy."""
    check_id = "ros_chassis_health"
    statuses = document.get("status")
    if not isinstance(statuses, list):
        return _unknown(check_id, "diagnostic array has no status list")
    matches = [
        value
        for value in statuses
        if isinstance(value, dict) and value.get("name") == CHASSIS_DIAGNOSTIC_NAME
    ]
    if len(matches) != 1:
        return _unknown(
            check_id,
            "expected exactly one Go2 chassis diagnostic status",
            matches=len(matches),
        )
    status = matches[0]
    entries = status.get("values")
    if not isinstance(entries, list):
        return _unknown(check_id, "chassis diagnostic values are absent")
    values: dict[str, str] = {}
    for entry in entries:
        if not isinstance(entry, dict):
            return _unknown(check_id, "chassis diagnostic value is malformed")
        key = entry.get("key")
        value = entry.get("value")
        if not isinstance(key, str) or not isinstance(value, str) or key in values:
            return _unknown(
                check_id,
                "chassis diagnostic key/value is invalid or duplicated",
            )
        values[key] = value
    try:
        level = _diagnostic_uint8(status.get("level"), "diagnostic level")
        error_code = _diagnostic_integer(
            values.get("sport_error_code"), "sport_error_code"
        )
        state_age = _diagnostic_float(values, "state_age_sec")
        state_timeout = _diagnostic_float(values, "state_timeout_sec")
        source_age_limit = _diagnostic_float(values, "max_source_stamp_age_sec")
        future_skew_limit = _diagnostic_float(
            values, "max_source_stamp_future_skew_sec"
        )
    except ValueError as exc:
        return _unknown(check_id, str(exc))

    reasons: list[str] = []
    # The passive adapter reports WARN solely because motion is intentionally
    # disabled. ERROR still fails, as do the explicit health fields below.
    if level not in {0, 1}:
        reasons.append("diagnostic level is ERROR or invalid")
    # Unitree exposes ``SportModeState.error_code`` as an opaque firmware state
    # marker on currently observed Go2 firmware.  Zero is accepted directly.
    # A non-zero value is accepted only when the adapter proves that this exact
    # value was explicitly allowlisted for the current session and that no
    # marker transition has been latched since arming/disarming.  This mirrors
    # the adapter's fail-closed StateMarkerPolicy instead of treating every
    # non-zero marker as a fault or, conversely, trusting it without evidence.
    marker_explicitly_allowed = (
        values.get("opaque_state_marker_explicitly_allowed") == "true"
    )
    marker_change_latched = values.get("opaque_state_marker_change_latched")
    marker_bound_raw = values.get("opaque_state_marker_bound")
    marker_bound: int | None = None
    if marker_change_latched == "true":
        reasons.append("opaque firmware state marker change is latched")
    if error_code != 0:
        if not marker_explicitly_allowed:
            reasons.append("non-zero firmware state marker is not explicitly allowed")
        if marker_change_latched != "false":
            reasons.append("non-zero firmware state marker has no unlatched proof")
        try:
            marker_bound = _diagnostic_integer(
                marker_bound_raw, "opaque_state_marker_bound"
            )
        except ValueError:
            reasons.append("non-zero firmware state marker has no exact bound marker")
        else:
            if marker_bound != error_code:
                reasons.append("bound firmware state marker does not match current marker")
    if values.get("state_valid") != "true":
        reasons.append("state_valid is not true")
    if values.get("source_stamp_status") != "fresh":
        reasons.append("source timestamp is not fresh")
    if not 0.0 <= state_age <= CHASSIS_MAX_STATE_AGE_S:
        reasons.append("state age exceeds the 0.20 s navigation ceiling")
    if not 0.0 < state_timeout <= CHASSIS_MAX_STATE_AGE_S:
        reasons.append("state timeout is not only-tighten")
    if not 0.0 < source_age_limit <= CHASSIS_MAX_SOURCE_STAMP_AGE_S:
        reasons.append("source stamp age limit is not only-tighten")
    if not 0.0 <= future_skew_limit <= CHASSIS_MAX_SOURCE_FUTURE_SKEW_S:
        reasons.append("source future-skew limit is not only-tighten")
    if reasons:
        return _check(
            check_id,
            False,
            "; ".join(reasons),
            sport_error_code=error_code,
            opaque_state_marker_explicitly_allowed=marker_explicitly_allowed,
            opaque_state_marker_change_latched=marker_change_latched,
            opaque_state_marker_bound=marker_bound_raw,
            state_age_sec=state_age,
            state_timeout_sec=state_timeout,
            max_source_stamp_age_sec=source_age_limit,
            max_source_stamp_future_skew_sec=future_skew_limit,
        )
    return _check(
        check_id,
        True,
        "fresh chassis state with fail-closed marker policy and only-tighten runtime limits",
        sport_error_code=error_code,
        opaque_state_marker_explicitly_allowed=marker_explicitly_allowed,
        opaque_state_marker_change_latched=marker_change_latched,
        opaque_state_marker_bound=marker_bound_raw,
        state_age_sec=state_age,
    )


def read_chassis_health(runner: CommandRunner) -> Check:
    result = runner.ros(
        (
            "ros2",
            "topic",
            "echo",
            CHASSIS_DIAGNOSTICS_TOPIC,
            "diagnostic_msgs/msg/DiagnosticArray",
            "--once",
            "--no-daemon",
            "--filter",
            f"any(s.name == '{CHASSIS_DIAGNOSTIC_NAME}' for s in m.status)",
            "--qos-reliability",
            "reliable",
            "--qos-durability",
            "volatile",
            "--qos-depth",
            "10",
        )
    )
    if result.returncode != 0 or result.timed_out:
        return _unknown(
            "ros_chassis_health",
            "bounded chassis diagnostic read received no usable sample",
            returncode=result.returncode,
        )
    try:
        documents = yaml_documents(result.stdout)
    except yaml.YAMLError as exc:
        return _unknown("ros_chassis_health", f"invalid diagnostic YAML: {exc}")
    if len(documents) != 1 or not isinstance(documents[0], dict):
        return _unknown("ros_chassis_health", "expected one diagnostic array")
    return validate_chassis_health_document(documents[0])


def load_landmark_binding(path: Path) -> tuple[Check, dict[str, Any] | None]:
    check_id = "semantic_landmark_binding"
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        return _unknown(check_id, f"landmark file unavailable: {type(exc).__name__}"), None
    if not isinstance(document, dict):
        return _unknown(check_id, "landmark file is not a YAML object"), None
    map_id = document.get("map_id")
    generation = document.get("map_generation")
    frame_id = document.get("frame_id")
    if not isinstance(map_id, str) or not map_id.strip():
        return _unknown(check_id, "landmark map_id is absent"), None
    if not isinstance(generation, int) or isinstance(generation, bool) or generation <= 0:
        return _check(check_id, False, "landmark map_generation is not a measured positive epoch"), None
    if frame_id != "map":
        return _check(check_id, False, "landmark frame_id must be map"), None
    landmarks = document.get("landmarks")
    if not isinstance(landmarks, list):
        return _unknown(check_id, "landmarks is not an array"), None
    verified_navigation: list[str] = []
    for target in landmarks:
        if not isinstance(target, dict):
            continue
        # schema v2 entries written before multi-location support omitted kind
        # and are navigation destinations by default.
        if target.get("kind", "navigation") != "navigation":
            continue
        if target.get("verified") is not True:
            continue
        pose = target.get("pose")
        if not isinstance(pose, dict):
            continue
        try:
            raw_coordinates = [pose[key] for key in ("x", "y", "yaw")]
            if any(isinstance(value, bool) for value in raw_coordinates):
                continue
            coordinates = [float(value) for value in raw_coordinates]
            arrival_radius = target.get("arrival_radius", 0.35)
            if isinstance(arrival_radius, bool):
                continue
            arrival_radius = float(arrival_radius)
        except (KeyError, TypeError, ValueError):
            continue
        if (
            not all(math.isfinite(value) for value in coordinates)
            or not math.isfinite(arrival_radius)
            or not 0.05 <= arrival_radius <= 10.0
        ):
            continue
        target_id = target.get("id")
        if isinstance(target_id, str) and target_id.strip():
            verified_navigation.append(target_id.strip())
    if not verified_navigation:
        return (
            _check(
                check_id,
                False,
                "no physically verified navigation landmark has a finite point pose",
            ),
            None,
        )
    binding = {"map_id": map_id.strip(), "generation": generation, "mode": "localization"}
    return (
        _check(
            check_id,
            True,
            "verified navigation landmark pose is bound to a measured map epoch",
            verified_navigation_landmarks=sorted(verified_navigation),
            **binding,
        ),
        binding,
    )


def validate_map_lifecycle_payload(payload: Any, expected: Mapping[str, Any]) -> Check:
    check_id = "map_lifecycle_exact"
    if not isinstance(payload, dict):
        return _unknown(check_id, "MapLifecycle sample is not a YAML object")
    actual = {key: payload.get(key) for key in ("map_id", "mode", "generation")}
    generation = actual["generation"]
    if not isinstance(generation, int) or isinstance(generation, bool) or generation < 0:
        return _unknown(check_id, "MapLifecycle generation is not uint64-compatible")
    if actual != dict(expected):
        return _check(
            check_id,
            False,
            "live MapLifecycle tuple does not exactly match the verified landmark",
            expected=dict(expected),
            actual=actual,
        )
    return _check(
        check_id,
        True,
        "live map_id/mode/generation exactly match verified semantic data",
        **actual,
    )


def read_map_lifecycle(expected: Mapping[str, Any], runner: CommandRunner) -> Check:
    result = runner.ros(
        (
            "ros2",
            "topic",
            "echo",
            "/robonix/map/lifecycle",
            "map/msg/MapLifecycle",
            "--once",
            "--no-daemon",
            "--qos-reliability",
            "reliable",
            "--qos-durability",
            "transient_local",
            "--qos-depth",
            "1",
        )
    )
    if result.returncode != 0 or result.timed_out:
        return _unknown(
            "map_lifecycle_exact",
            "bounded MapLifecycle read received no sample",
            returncode=result.returncode,
        )
    try:
        documents = yaml_documents(result.stdout)
    except yaml.YAMLError:
        return _unknown("map_lifecycle_exact", "MapLifecycle output is invalid YAML")
    if len(documents) != 1:
        return _unknown("map_lifecycle_exact", "expected exactly one MapLifecycle sample")
    return validate_map_lifecycle_payload(documents[0], expected)


def validate_lifecycle_output(node: str, result: CommandResult) -> Check:
    check_id = f"nav_lifecycle_{node}"
    if result.returncode != 0 or result.timed_out:
        return _unknown(check_id, "bounded lifecycle query failed", returncode=result.returncode)
    normalized = result.stdout.strip().lower()
    if not re.fullmatch(r"active\s*\[3\]", normalized):
        return _check(
            check_id,
            False,
            "Nav2 lifecycle node is not exactly active [3]",
            actual=normalized[:120],
        )
    return _check(check_id, True, "Nav2 lifecycle node is active [3]", node=node)


def read_nav_lifecycle(node: str, runner: CommandRunner) -> Check:
    result = runner.ros(
        ("ros2", "lifecycle", "get", f"/{node}", "--no-daemon")
    )
    return validate_lifecycle_output(node, result)


def validate_action_output(action: str, expected_type: str, result: CommandResult) -> Check:
    check_id = f"nav_action_{action.strip('/').replace('/', '_')}"
    if result.returncode != 0 or result.timed_out:
        return _unknown(check_id, "bounded action graph query failed", returncode=result.returncode)
    server_match = re.search(r"^Action servers:\s*(\d+)\s*$", result.stdout, re.MULTILINE)
    if server_match is None:
        return _unknown(check_id, "action server count is absent")
    if int(server_match.group(1)) != 1:
        return _check(
            check_id,
            False,
            "expected exactly one Nav2 action server",
            count=int(server_match.group(1)),
        )
    lines = result.stdout.splitlines()
    server_heading = next(
        (index for index, line in enumerate(lines) if line.startswith("Action servers:")),
        None,
    )
    if server_heading is None:
        return _unknown(check_id, "action server section is absent")
    server_lines: list[str] = []
    for line in lines[server_heading + 1 :]:
        if line and not line[0].isspace():
            break
        if line.strip():
            server_lines.append(line.strip())
    expected_suffix = f"[{expected_type}]"
    if len(server_lines) != 1 or not server_lines[0].endswith(expected_suffix):
        return _check(
            check_id,
            False,
            "Nav2 action server type does not match",
            expected_type=expected_type,
            servers=server_lines,
        )
    return _check(check_id, True, "exactly one correctly typed Nav2 action server")


def read_nav_action(action: str, expected_type: str, runner: CommandRunner) -> Check:
    result = runner.ros(("ros2", "action", "info", action, "--show-types"))
    return validate_action_output(action, expected_type, result)


_AT_TIME = re.compile(r"^At time\s+([0-9]+(?:\.[0-9]+)?)\s*$", re.MULTILINE)


def validate_tf_output(
    name: str,
    result: CommandResult,
    *,
    dynamic: bool,
    now_s: float,
    max_age_s: float = 2.0,
) -> Check:
    check_id = f"tf_{name}"
    if result.returncode not in {0, 124}:
        return _unknown(check_id, "bounded tf2_echo failed", returncode=result.returncode)
    if "Translation:" not in result.stdout or "Rotation:" not in result.stdout:
        return _unknown(check_id, "tf2_echo produced no complete transform")
    times = [float(value) for value in _AT_TIME.findall(result.stdout)]
    if not times:
        return _unknown(check_id, "tf2_echo produced no timestamp")
    latest = times[-1]
    if dynamic:
        age_s = now_s - latest
        if latest <= 0 or age_s < -0.5 or age_s > max_age_s:
            return _check(
                check_id,
                False,
                "dynamic TF is stale or future-dated",
                stamp=latest,
                age_s=round(age_s, 6),
            )
        return _check(
            check_id,
            True,
            "dynamic TF is connected and fresh",
            age_s=round(max(0.0, age_s), 6),
        )
    return _check(check_id, True, "static/sensor TF is connected")


def read_tf(
    name: str, parent: str, child: str, dynamic: bool, runner: CommandRunner
) -> Check:
    result = runner.ros(
        ("ros2", "run", "tf2_ros", "tf2_echo", parent, child),
        accept_timeout=True,
    )
    return validate_tf_output(name, result, dynamic=dynamic, now_s=time.time())


def run_parallel(callables: Iterable[Callable[[], Check]], max_workers: int = 6) -> list[Check]:
    work = list(callables)
    results: list[Check] = []
    with ThreadPoolExecutor(max_workers=min(max_workers, max(1, len(work)))) as executor:
        futures = [executor.submit(callable_) for callable_ in work]
        for future in as_completed(futures):
            try:
                results.append(future.result())
            except Exception as exc:  # noqa: BLE001
                results.append(
                    _unknown("internal_probe", f"readiness probe raised {type(exc).__name__}")
                )
    return sorted(results, key=lambda value: value.check_id)


def validate_expected_environment(args: argparse.Namespace, binding: Mapping[str, Any] | None) -> list[Check]:
    checks: list[Check] = []
    checks.append(
        _check(
            "configured_map_id",
            bool(binding) and args.map_id == binding.get("map_id"),
            "GO2_MAP_ID matches landmark binding" if binding and args.map_id == binding.get("map_id") else "GO2_MAP_ID does not match landmark binding",
            configured=args.map_id,
            landmark=binding.get("map_id") if binding else None,
        )
    )
    checks.append(
        _check(
            "configured_map_mode",
            args.map_mode == "localization",
            "full navigation readiness requires localization mode",
            configured=args.map_mode,
        )
    )
    checks.append(
        _check(
            "motion_gate_locked",
            args.allow_motion.strip().lower() not in {"1", "true", "yes", "on"},
            "readiness inspection is running with motion transport locked",
        )
    )
    return checks


def collect(args: argparse.Namespace) -> dict[str, Any]:
    deploy_dir = args.deploy_dir.resolve()
    runtime_dir = args.runtime_dir.resolve() if args.runtime_dir else deploy_dir
    runner = CommandRunner(args.command_timeout_s)
    checks: list[Check] = []

    checks.append(
        validate_boot_state(runtime_dir / "rbnx-boot" / "state.json", runtime_dir)
    )

    ports = dict(LOOPBACK_PORTS)
    ports["semantic_router"] = args.semantic_port
    ports["dashboard"] = args.dashboard_port
    checks.extend(
        run_parallel(
            (
                lambda name=name, port=port: validate_loopback_listener(name, port)
                for name, port in ports.items()
            )
        )
    )
    checks.extend(
        run_parallel(
            (
                lambda: validate_router(args.semantic_port, args.model),
                lambda: validate_dashboard(args.dashboard_port),
            ),
            max_workers=2,
        )
    )

    provider_check, _provider_payload = read_atlas_caps(runner)
    checks.append(provider_check)
    checks.append(
        validate_speech_backend(
            runtime_dir / "rbnx-boot" / "logs" / "speech.log"
        )
    )

    landmark_check, binding = load_landmark_binding(args.landmarks_file)
    checks.append(landmark_check)
    checks.extend(validate_expected_environment(args, binding))

    checks.extend(
        run_parallel(
            (lambda requirement=requirement: read_topic(requirement, runner) for requirement in TOPICS)
        )
    )
    checks.append(read_chassis_health(runner))
    if binding is None:
        checks.append(_unknown("map_lifecycle_exact", "no trusted landmark binding to compare"))
    else:
        checks.append(read_map_lifecycle(binding, runner))

    checks.extend(
        run_parallel(
            (
                lambda node=node: read_nav_lifecycle(node, runner)
                for node in NAV_LIFECYCLE_NODES
            )
        )
    )
    checks.extend(
        run_parallel(
            (
                lambda action=action, expected=expected: read_nav_action(
                    action, expected, runner
                )
                for action, expected in NAV_ACTIONS.items()
            )
        )
    )

    tf_requirements = (
        ("map_to_odom", "map", "odom", True),
        ("odom_to_base_link", "odom", "base_link", True),
        ("map_to_base_link", "map", "base_link", True),
        ("base_to_lidar", "base_link", "utlidar_lidar", False),
        ("base_to_camera", "base_link", "front_camera", False),
        ("base_to_imu", "base_link", "utlidar_imu", False),
    )
    checks.extend(
        run_parallel(
            (
                lambda name=name, parent=parent, child=child, dynamic=dynamic: read_tf(
                    name, parent, child, dynamic, runner
                )
                for name, parent, child, dynamic in tf_requirements
            )
        )
    )

    checks.sort(key=lambda value: value.check_id)
    ready = bool(checks) and all(value.status == PASS for value in checks)
    return {
        "schema_version": 1,
        "ready": ready,
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "mode": "read-only-fail-closed",
        "expected": {
            "map_id": args.map_id,
            "map_mode": args.map_mode,
            "semantic_model": args.model,
            "dashboard_port": args.dashboard_port,
            "semantic_port": args.semantic_port,
            "runtime_dir": str(runtime_dir),
        },
        "summary": {
            status: sum(1 for value in checks if value.status == status)
            for status in (PASS, FAIL, UNKNOWN)
        },
        "checks": [asdict(value) for value in checks],
    }


def write_report(report: Mapping[str, Any], report_dir: Path) -> Path:
    report_dir.mkdir(parents=True, exist_ok=True)
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = report_dir / f"stack-readiness-{timestamp}.json"
    temporary = report_dir / f".{path.name}.{os.getpid()}.tmp"
    payload = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return path


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument("--deploy-dir", type=Path, required=True)
    value.add_argument(
        "--runtime-dir",
        type=Path,
        help="directory containing the exact runtime manifest and rbnx-boot state",
    )
    value.add_argument("--landmarks-file", type=Path, required=True)
    value.add_argument("--map-id", required=True)
    value.add_argument("--map-mode", required=True, choices=("mapping", "localization"))
    value.add_argument("--model", default="go2-semantic-router")
    value.add_argument("--semantic-port", type=int, default=18080)
    value.add_argument("--dashboard-port", type=int, default=8092)
    value.add_argument("--allow-motion", default="false")
    value.add_argument("--command-timeout-s", type=float, default=6.0)
    value.add_argument("--report-dir", type=Path)
    return value


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if not (1 <= args.semantic_port <= 65535 and 1 <= args.dashboard_port <= 65535):
        print("readiness: invalid HTTP port", file=sys.stderr)
        return 2
    if not math.isfinite(args.command_timeout_s) or not (1.0 <= args.command_timeout_s <= 30.0):
        print("readiness: command timeout must be in 1..30 seconds", file=sys.stderr)
        return 2
    args.landmarks_file = args.landmarks_file.resolve()
    report = collect(args)
    report_dir = args.report_dir or args.deploy_dir / "logs" / "readiness"
    report_path = write_report(report, report_dir.resolve())
    for item in report["checks"]:
        print(f"[{item['status']:^7}] {item['check_id']}: {item['detail']}")
    summary = report["summary"]
    print(
        f"readiness={str(report['ready']).lower()} "
        f"pass={summary[PASS]} fail={summary[FAIL]} unknown={summary[UNKNOWN]}"
    )
    print(f"report={report_path}")
    return 0 if report["ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
