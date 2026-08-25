#!/usr/bin/env python3
"""Run one private RTAB-Map ICP producer and a bounded read-only observer.

This is a diagnostic path, not a replacement for canonical odometry.  The
single evaluated process subscribes only to ``/scanner/cloud``, an explicitly
selected IMU input and TF, publishes odometry only below
``/robonix/nomotion/icp_eval``, and has ``publish_tf=false``.  This supervising
process creates subscriptions only, records bounded stationary-drift evidence,
and stops the owned ICP child on every exit path.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence

import yaml


ROOT = Path(__file__).resolve().parents[1]
RUN_ROOT = ROOT / "rbnx-build" / "run"
LOG_ROOT = ROOT / "logs" / "icp-odom-nomotion"
CURRENT_SESSION = RUN_ROOT / "workstation-nomotion-current.session"
CONFIG_PATH = ROOT / "config" / "icp_odometry_nomotion_eval.yaml"
CLOUD_ONLY_CONFIG_PATH = (
    ROOT / "config" / "icp_odometry_nomotion_cloud_only_eval.yaml"
)
POINT_TO_POINT_CONFIG_PATH = (
    ROOT / "config" / "icp_odometry_nomotion_point_to_point_eval.yaml"
)
POINT_TO_POINT_FINE_CONFIG_PATH = (
    ROOT / "config" / "icp_odometry_nomotion_point_to_point_fine_eval.yaml"
)
POINT_TO_POINT_KEYFRAME_CONFIG_PATH = (
    ROOT / "config" / "icp_odometry_nomotion_point_to_point_keyframe_eval.yaml"
)
LOCK_PATH = RUN_ROOT / "icp-odometry-nomotion-eval.lock"

ICP_NODE_NAME = "icp_odometry"
ICP_NAMESPACE = "/robonix/nomotion/icp_eval"
ICP_NODE_FQN = f"{ICP_NAMESPACE}/{ICP_NODE_NAME}"
ICP_ODOM_TOPIC = f"{ICP_NAMESPACE}/odom"
ICP_ODOM_INFO_TOPIC = f"{ICP_NAMESPACE}/odom_info"
ICP_TF_DISABLED_TOPIC = f"{ICP_NAMESPACE}/tf_disabled"
ICP_ODOM_FRAME = "robonix_nomotion_icp_odom"
BASE_FRAME = "base_link"
SOURCE_CLOUD_TOPIC = "/scanner/cloud"
SOURCE_IMU_TOPIC = "/scanner/imu"
DISABLED_IMU_TOPIC = f"{ICP_NAMESPACE}/imu_disabled"
REFERENCE_ODOM_TOPIC = "/odom"
FORBIDDEN_MOTION_TOPICS = (
    "/cmd_vel",
    "/api/sport/request",
    "/lowcmd",
)
ACK_TOKEN = "I_APPROVE_PRIVATE_READONLY_ICP"
MIN_DURATION_SECONDS = 60
MAX_DURATION_SECONDS = 900
APPROVAL_MARGIN_SECONDS = 90
STARTUP_TIMEOUT_SECONDS = 30
SHUTDOWN_TIMEOUT_SECONDS = 10
RECORD_INTERVAL_NS = 200_000_000

INPUT_PROFILES: dict[str, dict[str, Any]] = {
    "imu-assisted": {
        "config_path": CONFIG_PATH,
        "imu_topic": SOURCE_IMU_TOPIC,
        "wait_imu_to_init": True,
        "consumes_source_imu": True,
    },
    "cloud-only": {
        "config_path": CLOUD_ONLY_CONFIG_PATH,
        "imu_topic": DISABLED_IMU_TOPIC,
        "wait_imu_to_init": False,
        "consumes_source_imu": False,
    },
    "point-to-point": {
        "config_path": POINT_TO_POINT_CONFIG_PATH,
        "imu_topic": SOURCE_IMU_TOPIC,
        "wait_imu_to_init": True,
        "consumes_source_imu": True,
    },
    "point-to-point-fine": {
        "config_path": POINT_TO_POINT_FINE_CONFIG_PATH,
        "imu_topic": SOURCE_IMU_TOPIC,
        "wait_imu_to_init": True,
        "consumes_source_imu": True,
    },
    "point-to-point-keyframe": {
        "config_path": POINT_TO_POINT_KEYFRAME_CONFIG_PATH,
        "imu_topic": SOURCE_IMU_TOPIC,
        "wait_imu_to_init": True,
        "consumes_source_imu": True,
    },
}


class GateError(RuntimeError):
    """A fail-closed no-motion or evidence-integrity check failed."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise GateError(message)


def utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _bounded_int(value: str, minimum: int, maximum: int, label: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{label} must be an integer") from exc
    if not minimum <= parsed <= maximum:
        raise argparse.ArgumentTypeError(
            f"{label} must be in {minimum}..{maximum}"
        )
    return parsed


def _positive_float(value: str, label: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{label} must be numeric") from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"{label} must be finite and positive")
    return parsed


def _bounded_float(
    value: str, minimum: float, maximum: float, label: str
) -> float:
    parsed = _positive_float(value, label)
    if not minimum <= parsed <= maximum:
        raise argparse.ArgumentTypeError(
            f"{label} must be in {minimum}..{maximum}"
        )
    return parsed


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run exactly one isolated RTAB-Map ICP odometry producer and a "
            "bounded subscription-only stationary drift observer"
        )
    )
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--approval", type=Path, required=True)
    parser.add_argument(
        "--duration-seconds",
        type=lambda value: _bounded_int(
            value, MIN_DURATION_SECONDS, MAX_DURATION_SECONDS, "duration-seconds"
        ),
        default=600,
    )
    parser.add_argument(
        "--warmup-seconds",
        type=lambda value: _bounded_int(value, 10, 120, "warmup-seconds"),
        default=30,
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--ack", required=True)
    parser.add_argument(
        "--input-profile",
        choices=tuple(INPUT_PROFILES),
        default="imu-assisted",
        help=(
            "imu-assisted uses point-to-plane ICP with /scanner/imu; "
            "cloud-only remaps IMU to a private publisher-free topic; "
            "point-to-point keeps IMU and changes only the ICP error model; "
            "point-to-point-fine additionally changes only voxel size; "
            "point-to-point-keyframe changes only local-map keyframe rate"
        ),
    )
    parser.add_argument(
        "--max-translation-drift-m",
        type=lambda value: _bounded_float(
            value, 0.001, 0.05, "max-translation-drift-m"
        ),
        default=0.05,
    )
    parser.add_argument(
        "--max-yaw-drift-deg",
        type=lambda value: _bounded_float(
            value, 0.1, 2.0, "max-yaw-drift-deg"
        ),
        default=2.0,
    )
    parser.add_argument(
        "--max-yaw-rate-deg-per-min",
        type=lambda value: _bounded_float(
            value, 0.01, 0.25, "max-yaw-rate-deg-per-min"
        ),
        default=0.25,
    )
    parser.add_argument(
        "--min-output-rate-hz",
        type=lambda value: _bounded_float(
            value, 5.0, 30.0, "min-output-rate-hz"
        ),
        default=5.0,
    )
    parser.add_argument(
        "--max-output-gap-seconds",
        type=lambda value: _bounded_float(
            value, 0.05, 1.0, "max-output-gap-seconds"
        ),
        default=1.0,
    )
    return parser


def safe_regular(path: Path, *, below: Path = ROOT) -> Path:
    resolved = Path(os.path.abspath(os.fspath(path.expanduser())))
    try:
        resolved.relative_to(below.resolve())
    except ValueError as exc:
        raise GateError(f"path must remain below {below.resolve()}: {resolved}") from exc
    try:
        info = os.lstat(resolved)
    except OSError as exc:
        raise GateError(f"required file unavailable: {resolved}: {exc}") from exc
    require(stat.S_ISREG(info.st_mode), f"required path is not a regular file: {resolved}")
    require(not stat.S_ISLNK(info.st_mode), f"required file is a symlink: {resolved}")
    require(info.st_uid == os.geteuid(), f"required file has a different owner: {resolved}")
    return resolved


def safe_json(path: Path) -> dict[str, Any]:
    regular = safe_regular(path)
    try:
        payload = json.loads(regular.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise GateError(f"invalid JSON evidence {regular}: {exc}") from exc
    require(isinstance(payload, dict), f"JSON evidence is not an object: {regular}")
    return payload


def parse_current_session(path: Path = CURRENT_SESSION) -> dict[str, str]:
    regular = safe_regular(path)
    values: dict[str, str] = {}
    allowed = {"format", "token", "wrapper_pid", "wrapper_start_ticks", "run_dir"}
    for line in regular.read_text(encoding="utf-8").splitlines():
        require("=" in line, f"malformed current session metadata: {regular}")
        key, value = line.split("=", 1)
        require(key in allowed and key not in values, f"invalid session metadata key: {key}")
        values[key] = value
    require(set(values) == allowed, "current session metadata is incomplete")
    require(values["format"] == "go2-workstation-nomotion-session-v1", "wrong session format")
    require(values["wrapper_pid"].isdigit(), "invalid no-motion wrapper pid")
    require(values["wrapper_start_ticks"].isdigit(), "invalid wrapper start ticks")
    return values


def process_start_ticks(pid: int) -> int:
    try:
        line = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    except OSError as exc:
        raise GateError(f"no-motion wrapper is not alive: pid={pid}: {exc}") from exc
    marker = line.rfind(") ")
    require(marker >= 0, "malformed wrapper process stat")
    fields = line[marker + 2 :].split()
    require(len(fields) > 19 and fields[0] != "Z", "no-motion wrapper is a zombie")
    return int(fields[19])


def _named_manifest_entry(entries: Any, name: str, section: str) -> Mapping[str, Any]:
    require(isinstance(entries, list), f"manifest {section} is not a list")
    matches = [
        entry
        for entry in entries
        if isinstance(entry, dict) and entry.get("name") == name
    ]
    require(len(matches) == 1, f"manifest must contain exactly one {section}.{name}")
    return matches[0]


def validate_evidence_payloads(
    approval: Mapping[str, Any],
    ready: Mapping[str, Any],
    identity: Mapping[str, Any],
    manifest: Mapping[str, Any],
    *,
    now_ns: int,
    duration_seconds: int,
) -> dict[str, Any]:
    require(
        approval.get("schema") == "robonix-go2-workstation-nomotion-stamp-offset-v3",
        "approval is not the affine workstation no-motion v3 schema",
    )
    require(approval.get("motion_enabled") is False, "approval is not explicitly no-motion")
    require(approval.get("identity_evidence_verified") is True, "approval identity is unverified")
    require(isinstance(approval.get("not_before_unix_ns"), int), "approval not-before is missing")
    require(isinstance(approval.get("expires_unix_ns"), int), "approval expiry is missing")
    require(approval["not_before_unix_ns"] <= now_ns, "approval is not active yet")
    minimum_expiry_ns = now_ns + (
        duration_seconds + APPROVAL_MARGIN_SECONDS
    ) * 1_000_000_000
    require(
        approval["expires_unix_ns"] >= minimum_expiry_ns,
        "approval does not cover observation duration plus 90 s startup/shutdown margin",
    )

    require(ready.get("time_discipline_ready") is True, "timestamp discipline is not ready")
    require(ready.get("motion_ready") is False, "ready evidence unexpectedly enables motion")
    require(ready.get("canonical_odom_ready") is False, "ready evidence unexpectedly authorizes canonical odom")
    require(identity.get("identity_bound") is True, "writer identity is not bound")
    require(identity.get("motion_ready") is False, "identity evidence unexpectedly enables motion")
    require(identity.get("canonical_odom_ready") is False, "identity evidence unexpectedly authorizes canonical odom")

    session_ids = {
        approval.get("session_id"), ready.get("session_id"), identity.get("session_id")
    }
    require(None not in session_ids and len(session_ids) == 1, "approval/ready/identity session IDs differ")
    require(approval.get("writer_gids") == identity.get("writer_gids"), "approval/identity writer GIDs differ")

    chassis = _named_manifest_entry(manifest.get("primitive"), "go2_chassis", "primitive")
    chassis_config = chassis.get("config")
    require(isinstance(chassis_config, dict), "go2_chassis config is missing")
    require(chassis_config.get("allow_motion") is False, "rendered manifest enables chassis motion")
    require(chassis_config.get("operator_present") is False, "rendered manifest marks an operator-present motion profile")
    require(
        chassis_config.get("twist_in_topic") == "/robonix/nomotion/chassis_input_disabled",
        "rendered manifest does not isolate chassis command input",
    )

    sensors = _named_manifest_entry(manifest.get("primitive"), "go2_sensors", "primitive")
    sensor_config = sensors.get("config")
    require(isinstance(sensor_config, dict), "go2_sensors config is missing")
    require(sensor_config.get("lidar_output_topic") == SOURCE_CLOUD_TOPIC, "unexpected lidar relay output")
    require(sensor_config.get("imu_output_topic") == SOURCE_IMU_TOPIC, "unexpected IMU relay output")
    return {
        "session_id": next(iter(session_ids)),
        "expires_unix_ns": approval["expires_unix_ns"],
        "minimum_expiry_unix_ns": minimum_expiry_ns,
        "writer_gids": approval["writer_gids"],
    }


def validate_live_session(
    session_dir: Path, approval_path: Path, duration_seconds: int
) -> tuple[Path, dict[str, Any]]:
    resolved_session = Path(os.path.abspath(os.fspath(session_dir.expanduser())))
    try:
        resolved_session.relative_to(RUN_ROOT.resolve())
    except ValueError as exc:
        raise GateError(f"session directory must remain below {RUN_ROOT}") from exc
    try:
        info = os.lstat(resolved_session)
    except OSError as exc:
        raise GateError(f"session directory unavailable: {resolved_session}: {exc}") from exc
    require(stat.S_ISDIR(info.st_mode) and not stat.S_ISLNK(info.st_mode), "unsafe session directory")
    require(resolved_session.name.startswith("workstation-nomotion-stamp."), "wrong session directory type")

    current = parse_current_session()
    require(Path(current["run_dir"]).resolve() == resolved_session, "requested session is not current")
    pid = int(current["wrapper_pid"])
    require(process_start_ticks(pid) == int(current["wrapper_start_ticks"]), "wrapper PID was reused")
    require(not (resolved_session / "fault.json").exists(), "current timestamp session is faulted")

    approval = safe_json(approval_path)
    ready = safe_json(resolved_session / "ready.json")
    identity = safe_json(resolved_session / "identity-ready.json")
    manifest_path = safe_regular(resolved_session / "robonix_manifest.yaml")
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise GateError(f"invalid rendered manifest: {exc}") from exc
    require(isinstance(manifest, dict), "rendered manifest is not an object")
    validated = validate_evidence_payloads(
        approval,
        ready,
        identity,
        manifest,
        now_ns=time.time_ns(),
        duration_seconds=duration_seconds,
    )
    validated.update(
        {
            "session_dir": str(resolved_session),
            "approval_path": str(safe_regular(approval_path)),
            "wrapper_pid": pid,
            "wrapper_start_ticks": int(current["wrapper_start_ticks"]),
        }
    )
    return resolved_session, validated


def prepare_output(requested: Path | None) -> Path:
    LOG_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(LOG_ROOT, 0o700)
    target = requested or (LOG_ROOT / f"{utc_stamp()}-stationary")
    resolved = Path(os.path.abspath(os.fspath(target.expanduser())))
    try:
        relative = resolved.relative_to(LOG_ROOT.resolve())
    except ValueError as exc:
        raise GateError(f"output must remain below {LOG_ROOT}") from exc
    require(relative != Path("."), "output must be a new child directory")
    require(not resolved.exists() and not resolved.is_symlink(), f"refusing to overwrite {resolved}")
    resolved.mkdir(mode=0o700, parents=False)
    os.chmod(resolved, 0o700)
    return resolved


def write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def resolve_icp_executable() -> Path:
    prefixes = [Path(value) for value in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if value]
    prefixes.extend((Path("/opt/ros/humble"),))
    for prefix in prefixes:
        candidate = prefix / "lib" / "rtabmap_odom" / "icp_odometry"
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    raise GateError("rtabmap_odom/icp_odometry is unavailable; source ROS 2 Humble first")


def build_icp_command(
    executable: Path,
    config_path: Path = CONFIG_PATH,
    imu_topic: str = SOURCE_IMU_TOPIC,
) -> list[str]:
    return [
        str(executable),
        "--ros-args",
        "-r", f"__node:={ICP_NODE_NAME}",
        "-r", f"__ns:={ICP_NAMESPACE}",
        "--params-file", str(config_path),
        "-r", f"scan_cloud:={SOURCE_CLOUD_TOPIC}",
        "-r", f"imu:={imu_topic}",
        "-r", f"odom:={ICP_ODOM_TOPIC}",
        "-r", f"odom_info:={ICP_ODOM_INFO_TOPIC}",
        # RTAB-Map eagerly creates a TF publisher endpoint even when
        # publish_tf=false. Isolate that endpoint and its matching dynamic-TF
        # listener. Static sensor extrinsics remain available on /tf_static.
        "-r", f"/tf:={ICP_TF_DISABLED_TOPIC}",
        "-r", "/tf_static:=/tf_static",
        "--disable-rosout-logs",
    ]


@dataclass(frozen=True)
class PoseSample:
    receipt_ns: int
    source_ns: int
    frame_id: str
    child_frame_id: str
    x: float
    y: float
    z: float
    yaw: float

    def record(self, stream: str) -> dict[str, Any]:
        return {
            "stream": stream,
            "receipt_monotonic_ns": self.receipt_ns,
            "source_stamp_ns": self.source_ns,
            "frame_id": self.frame_id,
            "child_frame_id": self.child_frame_id,
            "position": {"x": self.x, "y": self.y, "z": self.z},
            "yaw_rad": self.yaw,
        }


class PoseSeries:
    def __init__(self, expected_frame: str, expected_child: str) -> None:
        self.expected_frame = expected_frame
        self.expected_child = expected_child
        self.received = 0
        self.valid = 0
        self.invalid = 0
        self.invalid_reasons = {
            "non_finite_pose": 0,
            "invalid_quaternion_norm": 0,
        }
        self.frame_mismatches = 0
        self.first_receipt_ns: int | None = None
        self.last_receipt_ns: int | None = None
        self.maximum_gap_ns = 0
        self.first_valid_receipt_ns: int | None = None
        self.last_valid_receipt_ns: int | None = None
        self.maximum_valid_gap_ns = 0
        self.current_invalid_run = 0
        self.longest_invalid_run = 0
        self.recovery_count = 0
        self.first_loss_receipt_ns: int | None = None
        self.last_record_ns: int | None = None
        self.valid_samples: list[PoseSample] = []
        self.samples: list[PoseSample] = []

    def mark_invalid(self, reason: str, receipt_ns: int) -> None:
        self.invalid += 1
        self.invalid_reasons[reason] += 1
        self.current_invalid_run += 1
        self.longest_invalid_run = max(
            self.longest_invalid_run, self.current_invalid_run
        )
        if self.last_valid_receipt_ns is not None and self.first_loss_receipt_ns is None:
            self.first_loss_receipt_ns = receipt_ns

    def observe(self, message: Any, receipt_ns: int) -> PoseSample | None:
        self.received += 1
        if self.first_receipt_ns is None:
            self.first_receipt_ns = receipt_ns
        if self.last_receipt_ns is not None:
            self.maximum_gap_ns = max(self.maximum_gap_ns, receipt_ns - self.last_receipt_ns)
        self.last_receipt_ns = receipt_ns
        pose = message.pose.pose
        values = (
            float(pose.position.x), float(pose.position.y), float(pose.position.z),
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        )
        if not all(math.isfinite(value) for value in values):
            self.mark_invalid("non_finite_pose", receipt_ns)
            return None
        qx, qy, qz, qw = values[3:]
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not 0.98 <= norm <= 1.02:
            self.mark_invalid("invalid_quaternion_norm", receipt_ns)
            return None
        frame_id = str(message.header.frame_id)
        child_frame_id = str(message.child_frame_id)
        if frame_id != self.expected_frame or child_frame_id != self.expected_child:
            self.frame_mismatches += 1
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        sample = PoseSample(
            receipt_ns=receipt_ns,
            source_ns=int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec),
            frame_id=frame_id,
            child_frame_id=child_frame_id,
            x=values[0], y=values[1], z=values[2], yaw=yaw,
        )
        self.valid += 1
        if self.first_valid_receipt_ns is None:
            self.first_valid_receipt_ns = receipt_ns
        if self.last_valid_receipt_ns is not None:
            self.maximum_valid_gap_ns = max(
                self.maximum_valid_gap_ns,
                receipt_ns - self.last_valid_receipt_ns,
            )
            if self.current_invalid_run:
                self.recovery_count += 1
        self.last_valid_receipt_ns = receipt_ns
        self.current_invalid_run = 0
        self.valid_samples.append(sample)
        if self.last_record_ns is not None and receipt_ns - self.last_record_ns < RECORD_INTERVAL_NS:
            return None
        self.last_record_ns = receipt_ns
        self.samples.append(sample)
        return sample

    def summary(self, warmup_seconds: int, finished_ns: int | None = None) -> dict[str, Any]:
        last_receipt_age_ns = (
            None
            if finished_ns is None or self.last_receipt_ns is None
            else max(0, finished_ns - self.last_receipt_ns)
        )
        last_valid_age_ns = (
            None
            if finished_ns is None or self.last_valid_receipt_ns is None
            else max(0, finished_ns - self.last_valid_receipt_ns)
        )
        receipt_duration_ns = (
            0
            if self.first_receipt_ns is None or self.last_receipt_ns is None
            else max(0, self.last_receipt_ns - self.first_receipt_ns)
        )
        base_summary = {
            "received_messages": self.received,
            "valid_messages": self.valid,
            "retained_samples": len(self.samples),
            "invalid_messages": self.invalid,
            "invalid_reasons": dict(self.invalid_reasons),
            "frame_mismatches": self.frame_mismatches,
            "maximum_receipt_gap_ns": self.maximum_gap_ns,
            "last_receipt_age_ns": last_receipt_age_ns,
            "maximum_valid_gap_ns": self.maximum_valid_gap_ns,
            "last_valid_age_ns": last_valid_age_ns,
            "longest_invalid_run": self.longest_invalid_run,
            "trailing_invalid_run": self.current_invalid_run,
            "recovery_count": self.recovery_count,
            "first_loss_receipt_monotonic_ns": self.first_loss_receipt_ns,
            "valid_output_rate_hz": (
                self.valid / max(1e-9, receipt_duration_ns / 1e9)
                if receipt_duration_ns > 0
                else 0.0
            ),
            "observed_rate_hz": (
                (self.received - 1) / max(1e-9, receipt_duration_ns / 1e9)
                if self.received > 1
                else 0.0
            ),
        }
        if not self.valid_samples:
            return {**base_summary, "analyzed": False}
        assert self.first_receipt_ns is not None
        cutoff = self.first_receipt_ns + warmup_seconds * 1_000_000_000
        selected = [
            sample for sample in self.valid_samples if sample.receipt_ns >= cutoff
        ]
        if len(selected) < 2:
            return {
                **base_summary,
                "analyzed_samples": len(selected),
                "analyzed": False,
            }
        unwrapped = [selected[0].yaw]
        for sample in selected[1:]:
            delta = math.atan2(math.sin(sample.yaw - unwrapped[-1]), math.cos(sample.yaw - unwrapped[-1]))
            unwrapped.append(unwrapped[-1] + delta)
        elapsed = [(sample.receipt_ns - selected[0].receipt_ns) / 1e9 for sample in selected]
        duration_s = elapsed[-1]
        mean_t = sum(elapsed) / len(elapsed)
        mean_yaw = sum(unwrapped) / len(unwrapped)
        denominator = sum((value - mean_t) ** 2 for value in elapsed)
        slope = 0.0 if denominator == 0.0 else sum(
            (t_value - mean_t) * (yaw - mean_yaw)
            for t_value, yaw in zip(elapsed, unwrapped)
        ) / denominator
        first, last = selected[0], selected[-1]
        translation = math.sqrt(
            (last.x - first.x) ** 2 + (last.y - first.y) ** 2 + (last.z - first.z) ** 2
        )
        maximum_translation = max(
            math.sqrt(
                (sample.x - first.x) ** 2
                + (sample.y - first.y) ** 2
                + (sample.z - first.z) ** 2
            )
            for sample in selected
        )
        full_first = self.valid_samples[0]
        full_unwrapped = [full_first.yaw]
        for sample in self.valid_samples[1:]:
            delta = math.atan2(
                math.sin(sample.yaw - full_unwrapped[-1]),
                math.cos(sample.yaw - full_unwrapped[-1]),
            )
            full_unwrapped.append(full_unwrapped[-1] + delta)
        full_maximum_translation = max(
            math.sqrt(
                (sample.x - full_first.x) ** 2
                + (sample.y - full_first.y) ** 2
                + (sample.z - full_first.z) ** 2
            )
            for sample in self.valid_samples
        )
        return {
            **base_summary,
            "analyzed_samples": len(selected),
            "analyzed": True,
            "warmup_seconds": warmup_seconds,
            "analyzed_duration_seconds": duration_s,
            "translation_drift_m": translation,
            "maximum_translation_from_start_m": maximum_translation,
            "full_run_maximum_translation_from_start_m": full_maximum_translation,
            "yaw_drift_rad": unwrapped[-1] - unwrapped[0],
            "yaw_drift_deg": math.degrees(unwrapped[-1] - unwrapped[0]),
            "yaw_excursion_deg": math.degrees(max(unwrapped) - min(unwrapped)),
            "full_run_yaw_excursion_deg": math.degrees(
                max(full_unwrapped) - min(full_unwrapped)
            ),
            "yaw_slope_rad_per_second": slope,
            "yaw_slope_deg_per_minute": math.degrees(slope) * 60.0,
            "start": first.record("start"),
            "end": last.record("end"),
        }


class OdomInfoSeries:
    """Keep loss/registration evidence without parsing interleaved stdout."""

    def __init__(self) -> None:
        self.received = 0
        self.lost = 0
        self.current_lost_run = 0
        self.longest_lost_run = 0
        self.recovery_count = 0
        self.first_receipt_ns: int | None = None
        self.last_receipt_ns: int | None = None
        self.maximum_gap_ns = 0
        self.ratios: list[float] = []
        self.translations: list[float] = []
        self.rotations: list[float] = []
        self.correspondences: list[int] = []
        self.guess_translation_magnitudes: list[float] = []

    def observe(self, message: Any, receipt_ns: int) -> dict[str, Any]:
        self.received += 1
        if self.first_receipt_ns is None:
            self.first_receipt_ns = receipt_ns
        if self.last_receipt_ns is not None:
            self.maximum_gap_ns = max(
                self.maximum_gap_ns, receipt_ns - self.last_receipt_ns
            )
        self.last_receipt_ns = receipt_ns
        lost = bool(message.lost)
        if lost:
            self.lost += 1
            self.current_lost_run += 1
            self.longest_lost_run = max(
                self.longest_lost_run, self.current_lost_run
            )
        else:
            if self.current_lost_run:
                self.recovery_count += 1
            self.current_lost_run = 0
        ratio = float(message.icp_inliers_ratio)
        translation = float(message.icp_translation)
        rotation = float(message.icp_rotation)
        correspondences = int(message.icp_correspondences)
        guess = message.guess.translation
        guess_translation = math.sqrt(
            float(guess.x) ** 2 + float(guess.y) ** 2 + float(guess.z) ** 2
        )
        for values, value in (
            (self.ratios, ratio),
            (self.translations, translation),
            (self.rotations, rotation),
            (self.guess_translation_magnitudes, guess_translation),
        ):
            if math.isfinite(value):
                values.append(value)
        self.correspondences.append(correspondences)
        return {
            "stream": "private_icp_info",
            "receipt_monotonic_ns": receipt_ns,
            "source_stamp_ns": (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            ),
            "lost": lost,
            "icp_inliers_ratio": ratio,
            "icp_translation": translation,
            "icp_rotation": rotation,
            "icp_correspondences": correspondences,
            "local_scan_map_size": int(message.local_scan_map_size),
            "guess_translation_m": guess_translation,
        }

    def summary(self, finished_ns: int | None = None) -> dict[str, Any]:
        duration_ns = (
            0
            if self.first_receipt_ns is None or self.last_receipt_ns is None
            else max(0, self.last_receipt_ns - self.first_receipt_ns)
        )
        return {
            "received_messages": self.received,
            "lost_messages": self.lost,
            "lost_fraction": self.lost / self.received if self.received else None,
            "longest_lost_run": self.longest_lost_run,
            "trailing_lost_run": self.current_lost_run,
            "recovery_count": self.recovery_count,
            "observed_rate_hz": (
                (self.received - 1) / max(1e-9, duration_ns / 1e9)
                if self.received > 1
                else 0.0
            ),
            "maximum_receipt_gap_ns": self.maximum_gap_ns,
            "last_receipt_age_ns": (
                None
                if finished_ns is None or self.last_receipt_ns is None
                else max(0, finished_ns - self.last_receipt_ns)
            ),
            "icp_inliers_ratio_min": min(self.ratios) if self.ratios else None,
            "icp_inliers_ratio_max": max(self.ratios) if self.ratios else None,
            "icp_inliers_ratio_mean": (
                sum(self.ratios) / len(self.ratios) if self.ratios else None
            ),
            "icp_translation_max": (
                max(self.translations) if self.translations else None
            ),
            "icp_rotation_max": max(self.rotations) if self.rotations else None,
            "icp_correspondences_min": (
                min(self.correspondences) if self.correspondences else None
            ),
            "icp_correspondences_max": (
                max(self.correspondences) if self.correspondences else None
            ),
            "guess_translation_max_m": (
                max(self.guess_translation_magnitudes)
                if self.guess_translation_magnitudes
                else None
            ),
        }


def endpoint_record(endpoint: Any) -> dict[str, Any]:
    gid = bytes(endpoint.endpoint_gid)
    return {
        "node_name": str(endpoint.node_name),
        "node_namespace": str(endpoint.node_namespace),
        "topic_type": str(endpoint.topic_type),
        "endpoint_gid": gid.hex(),
        "participant_prefix": gid[:12].hex() if len(gid) >= 12 else "",
    }


def is_icp_endpoint(endpoint: Any) -> bool:
    return endpoint.node_name == ICP_NODE_NAME and endpoint.node_namespace.rstrip("/") == ICP_NAMESPACE


def endpoint_participant_prefix(endpoint: Any) -> bytes:
    gid = bytes(endpoint.endpoint_gid)
    require(len(gid) >= 12, "ROS endpoint GID is shorter than a DDS GUID prefix")
    return gid[:12]


def same_participant(endpoint: Any, prefix: bytes) -> bool:
    require(len(prefix) == 12, "DDS participant prefix must be exactly 12 bytes")
    return endpoint_participant_prefix(endpoint) == prefix


def stop_owned_process(process: subprocess.Popen[Any], cleanup_errors: list[str]) -> None:
    if process.poll() is not None:
        return
    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=SHUTDOWN_TIMEOUT_SECONDS)
        return
    except subprocess.TimeoutExpired:
        cleanup_errors.append("icp process ignored SIGINT")
    except OSError as exc:
        cleanup_errors.append(f"icp SIGINT failed: {exc}")
    if process.poll() is None:
        try:
            process.terminate()
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            cleanup_errors.append("icp process ignored SIGTERM; SIGKILL used")
            process.kill()
            process.wait(timeout=3)
        except OSError as exc:
            cleanup_errors.append(f"icp termination failed: {exc}")


def static_config_contract(
    config_path: Path = CONFIG_PATH,
    *,
    expected_wait_imu: bool = True,
) -> dict[str, Any]:
    regular = safe_regular(config_path)
    payload = yaml.safe_load(regular.read_text(encoding="utf-8"))
    require(isinstance(payload, dict) and set(payload) == {ICP_NODE_FQN}, "ICP config must target only the private node")
    node = payload[ICP_NODE_FQN]
    require(isinstance(node, dict) and isinstance(node.get("ros__parameters"), dict), "ICP config parameters missing")
    parameters = node["ros__parameters"]
    require(parameters.get("publish_tf") is False, "ICP config must disable TF publication")
    require(parameters.get("odom_frame_id") == ICP_ODOM_FRAME, "ICP parent frame is not private")
    require(parameters.get("frame_id") == BASE_FRAME, "ICP input transform frame changed")
    require(
        parameters.get("publish_null_when_lost") is True,
        "ICP must publish explicit null/lost odometry for diagnosis",
    )
    require(parameters.get("deskewing") is False, "unqualified lidar deskewing must stay disabled")
    require(parameters.get("Reg/Force3DoF") == "true", "ICP must remain three-DoF")
    require(parameters.get("Icp/MaxTranslation") == "0.20", "ICP translation limit changed")
    require(parameters.get("Icp/MaxRotation") == "0.35", "ICP rotation limit changed")
    require(parameters.get("Odom/GuessMotion") == "false", "unverified constant-motion guessing must stay disabled")
    require(parameters.get("Odom/ResetCountdown") == "1", "lost ICP must reset immediately")
    require(
        parameters.get("wait_imu_to_init") is expected_wait_imu,
        "ICP wait_imu_to_init does not match the selected input profile",
    )
    return {
        "path": str(regular),
        "sha256": hashlib.sha256(regular.read_bytes()).hexdigest(),
        "node_fqn": ICP_NODE_FQN,
        "odom_topic": ICP_ODOM_TOPIC,
        "odom_frame": ICP_ODOM_FRAME,
        "child_frame": BASE_FRAME,
        "publish_tf": False,
        "wait_imu_to_init": expected_wait_imu,
    }


def evaluate_gates(summary: Mapping[str, Any], arguments: argparse.Namespace) -> list[dict[str, Any]]:
    private = summary["streams"]["private_icp"]
    info = summary["streams"].get("private_icp_info", {})
    def numeric(key: str, fallback: float) -> float:
        value = private.get(key)
        return float(value) if isinstance(value, (int, float)) else fallback
    def info_numeric(key: str, fallback: float) -> float:
        value = info.get(key)
        return float(value) if isinstance(value, (int, float)) else fallback

    checks = [
        ("private_series_analyzed", private.get("analyzed") is True, private.get("analyzed")),
        ("invalid_messages_zero", private.get("invalid_messages") == 0, private.get("invalid_messages")),
        ("frame_mismatches_zero", private.get("frame_mismatches") == 0, private.get("frame_mismatches")),
        ("valid_output_rate", numeric("valid_output_rate_hz", 0.0) >= arguments.min_output_rate_hz, private.get("valid_output_rate_hz")),
        ("valid_output_gap", numeric("maximum_valid_gap_ns", math.inf) <= arguments.max_output_gap_seconds * 1e9, private.get("maximum_valid_gap_ns")),
        ("valid_output_fresh_at_finish", numeric("last_valid_age_ns", math.inf) <= arguments.max_output_gap_seconds * 1e9, private.get("last_valid_age_ns")),
        ("trailing_invalid_run_zero", private.get("trailing_invalid_run") == 0, private.get("trailing_invalid_run")),
        ("odom_info_received", int(info.get("received_messages", 0)) > 0, info.get("received_messages")),
        ("odom_info_lost_zero", info.get("lost_messages") == 0, info.get("lost_messages")),
        ("odom_info_trailing_lost_zero", info.get("trailing_lost_run") == 0, info.get("trailing_lost_run")),
        ("odom_info_gap", info_numeric("maximum_receipt_gap_ns", math.inf) <= arguments.max_output_gap_seconds * 1e9, info.get("maximum_receipt_gap_ns")),
        ("odom_info_fresh_at_finish", info_numeric("last_receipt_age_ns", math.inf) <= arguments.max_output_gap_seconds * 1e9, info.get("last_receipt_age_ns")),
        ("observation_duration", numeric("analyzed_duration_seconds", 0.0) >= arguments.duration_seconds - arguments.warmup_seconds - 2.0, private.get("analyzed_duration_seconds")),
        ("translation_drift", numeric("translation_drift_m", math.inf) <= arguments.max_translation_drift_m, private.get("translation_drift_m")),
        ("translation_excursion", numeric("maximum_translation_from_start_m", math.inf) <= arguments.max_translation_drift_m, private.get("maximum_translation_from_start_m")),
        ("full_run_translation_excursion", numeric("full_run_maximum_translation_from_start_m", math.inf) <= arguments.max_translation_drift_m, private.get("full_run_maximum_translation_from_start_m")),
        ("yaw_drift", abs(numeric("yaw_drift_deg", math.inf)) <= arguments.max_yaw_drift_deg, private.get("yaw_drift_deg")),
        ("yaw_excursion", numeric("yaw_excursion_deg", math.inf) <= arguments.max_yaw_drift_deg, private.get("yaw_excursion_deg")),
        ("full_run_yaw_excursion", numeric("full_run_yaw_excursion_deg", math.inf) <= arguments.max_yaw_drift_deg, private.get("full_run_yaw_excursion_deg")),
        ("yaw_rate", abs(numeric("yaw_slope_deg_per_minute", math.inf)) <= arguments.max_yaw_rate_deg_per_min, private.get("yaw_slope_deg_per_minute")),
        (
            "private_tf_disabled_messages_zero",
            summary.get("private_tf_disabled_messages") == 0,
            summary.get("private_tf_disabled_messages"),
        ),
    ]
    return [
        {"name": name, "passed": bool(passed), "observed": observed}
        for name, passed, observed in checks
    ]


def run(arguments: argparse.Namespace) -> int:
    require(arguments.ack == ACK_TOKEN, f"--ack must equal {ACK_TOKEN}")
    require(arguments.warmup_seconds < arguments.duration_seconds - 10, "warmup leaves too little observation time")
    os.umask(0o077)
    RUN_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
    lock_stream = LOCK_PATH.open("a+", encoding="utf-8")
    try:
        try:
            fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise GateError("another ICP no-motion evaluation owns the lock") from exc

        session_dir, gate_evidence = validate_live_session(
            arguments.session_dir, arguments.approval, arguments.duration_seconds
        )
        input_profile = INPUT_PROFILES[arguments.input_profile]
        config_contract = static_config_contract(
            input_profile["config_path"],
            expected_wait_imu=input_profile["wait_imu_to_init"],
        )
        output_dir = prepare_output(arguments.output_dir)
        ros_log_dir = output_dir / "ros-logs"
        ros_log_dir.mkdir(mode=0o700)
        os.environ["ROS_LOG_DIR"] = str(ros_log_dir)
        child_log_path = output_dir / "icp-odometry.log"
        samples_path = output_dir / "samples.jsonl"
        executable = resolve_icp_executable()
        command = build_icp_command(
            executable,
            input_profile["config_path"],
            input_profile["imu_topic"],
        )

        environment = dict(os.environ)
        environment["ROS_LOG_DIR"] = str(ros_log_dir)
        metadata = {
            "schema": "robonix-go2-private-icp-nomotion-eval-v1",
            "mode": "single-private-icp-producer-with-readonly-bounded-observer",
            "safe_for_motion": False,
            "duration_seconds": arguments.duration_seconds,
            "warmup_seconds": arguments.warmup_seconds,
            "approval_margin_seconds": APPROVAL_MARGIN_SECONDS,
            "input_profile": arguments.input_profile,
            "inputs": [
                SOURCE_CLOUD_TOPIC,
                input_profile["imu_topic"],
                "/tf_static",
            ],
            "source_imu_health_checked": SOURCE_IMU_TOPIC,
            "source_imu_consumed": input_profile["consumes_source_imu"],
            "outputs": [
                ICP_ODOM_TOPIC,
                ICP_ODOM_INFO_TOPIC,
                ICP_TF_DISABLED_TOPIC,
            ],
            "reference_subscription": REFERENCE_ODOM_TOPIC,
            "config": config_contract,
            "gate_evidence": gate_evidence,
            "command": command,
            "motion_topics": "not published or subscribed",
            "canonical_odom_or_tf_authority": False,
            "thresholds": {
                "max_translation_drift_m": arguments.max_translation_drift_m,
                "max_yaw_drift_deg": arguments.max_yaw_drift_deg,
                "max_yaw_rate_deg_per_min": arguments.max_yaw_rate_deg_per_min,
                "min_output_rate_hz": arguments.min_output_rate_hz,
                "max_output_gap_seconds": arguments.max_output_gap_seconds,
            },
            "started_realtime_ns": time.time_ns(),
        }
        write_json_exclusive(output_dir / "metadata.json", metadata)

        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from rtabmap_msgs.msg import OdomInfo
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from rclpy.signals import SignalHandlerOptions
            from tf2_msgs.msg import TFMessage
        except ImportError as exc:
            raise GateError(f"ROS 2 Python dependencies unavailable; source Humble first: {exc}") from exc

        private_series = PoseSeries(ICP_ODOM_FRAME, BASE_FRAME)
        reference_series = PoseSeries("odom", BASE_FRAME)
        odom_info_series = OdomInfoSeries()
        tf_disabled_message_count = 0
        samples_stream = samples_path.open("x", encoding="utf-8", buffering=1)
        os.chmod(samples_path, 0o600)

        class ReadOnlyObserver(Node):
            def __init__(self) -> None:
                super().__init__(
                    f"icp_nomotion_observer_{os.getpid()}",
                    namespace=ICP_NAMESPACE,
                    start_parameter_services=False,
                    enable_rosout=False,
                )
                qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                )

                def observe(series: PoseSeries, stream_name: str):
                    def callback(message: Any) -> None:
                        sample = series.observe(message, time.perf_counter_ns())
                        if sample is not None:
                            samples_stream.write(json.dumps(sample.record(stream_name), sort_keys=True, separators=(",", ":")) + "\n")
                    return callback

                def observe_disabled_tf(_message: Any) -> None:
                    nonlocal tf_disabled_message_count
                    tf_disabled_message_count += 1

                def observe_odom_info(message: Any) -> None:
                    record = odom_info_series.observe(
                        message, time.perf_counter_ns()
                    )
                    samples_stream.write(
                        json.dumps(
                            record, sort_keys=True, separators=(",", ":")
                        )
                        + "\n"
                    )

                self.readonly_subscriptions = [
                    self.create_subscription(Odometry, ICP_ODOM_TOPIC, observe(private_series, "private_icp"), qos),
                    self.create_subscription(OdomInfo, ICP_ODOM_INFO_TOPIC, observe_odom_info, qos),
                    self.create_subscription(Odometry, REFERENCE_ODOM_TOPIC, observe(reference_series, "reference_odom"), qos),
                    self.create_subscription(TFMessage, ICP_TF_DISABLED_TOPIC, observe_disabled_tf, qos),
                ]

        initialized = False
        node = None
        process: subprocess.Popen[Any] | None = None
        child_log = None
        cleanup_errors: list[str] = []
        runtime_errors: list[str] = []
        graph_evidence: dict[str, Any] = {}
        exit_reason = "unknown"
        measurement_finished_ns: int | None = None
        try:
            rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
            initialized = True
            node = ReadOnlyObserver()
            source_deadline = time.perf_counter() + 10.0
            while time.perf_counter() < source_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if (
                    len(node.get_publishers_info_by_topic(SOURCE_CLOUD_TOPIC)) == 1
                    and len(node.get_publishers_info_by_topic(SOURCE_IMU_TOPIC)) == 1
                ):
                    break
            source_cloud_publishers = node.get_publishers_info_by_topic(
                SOURCE_CLOUD_TOPIC
            )
            source_imu_publishers = node.get_publishers_info_by_topic(
                SOURCE_IMU_TOPIC
            )
            require(len(source_cloud_publishers) == 1, "expected exactly one /scanner/cloud publisher")
            require(len(source_imu_publishers) == 1, "expected exactly one /scanner/imu publisher")
            source_cloud_record = endpoint_record(source_cloud_publishers[0])
            source_imu_record = endpoint_record(source_imu_publishers[0])
            require(len(node.get_publishers_info_by_topic(ICP_ODOM_TOPIC)) == 0, "private ICP output already has a publisher")
            require(len(node.get_publishers_info_by_topic(ICP_ODOM_INFO_TOPIC)) == 0, "private ICP odom_info already has a publisher")
            require(len(node.get_publishers_info_by_topic(ICP_TF_DISABLED_TOPIC)) == 0, "private disabled TF topic already has a publisher")

            child_log = child_log_path.open("x", encoding="utf-8", buffering=1)
            os.chmod(child_log_path, 0o600)
            process = subprocess.Popen(
                command,
                cwd=ROOT,
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=child_log,
                stderr=subprocess.STDOUT,
                text=True,
            )
            startup_deadline = time.perf_counter() + STARTUP_TIMEOUT_SECONDS
            while time.perf_counter() < startup_deadline and private_series.received == 0:
                if process.poll() is not None:
                    raise GateError(f"ICP process exited during startup with status {process.returncode}")
                if (session_dir / "fault.json").exists():
                    raise GateError("timestamp session faulted during ICP startup")
                rclpy.spin_once(node, timeout_sec=0.1)
            require(private_series.received > 0, "private ICP produced no odometry before startup timeout")

            private_publishers = node.get_publishers_info_by_topic(ICP_ODOM_TOPIC)
            require(len(private_publishers) == 1 and is_icp_endpoint(private_publishers[0]), "private odom publisher ownership mismatch")
            icp_participant_prefix = endpoint_participant_prefix(private_publishers[0])
            private_odom_info_publishers = [
                value
                for value in node.get_publishers_info_by_topic(
                    ICP_ODOM_INFO_TOPIC
                )
                if same_participant(value, icp_participant_prefix)
            ]
            require(
                len(private_odom_info_publishers) == 1,
                "private odom_info publisher ownership mismatch",
            )
            require(not any(same_participant(value, icp_participant_prefix) for value in node.get_publishers_info_by_topic(REFERENCE_ODOM_TOPIC)), "ICP participant unexpectedly publishes canonical /odom")
            require(not any(same_participant(value, icp_participant_prefix) for value in node.get_publishers_info_by_topic("/tf")), "ICP participant unexpectedly publishes /tf")
            require(not any(same_participant(value, icp_participant_prefix) for value in node.get_subscriptions_info_by_topic("/tf")), "ICP participant unexpectedly subscribes canonical /tf")
            private_tf_publishers = [
                value
                for value in node.get_publishers_info_by_topic(ICP_TF_DISABLED_TOPIC)
                if same_participant(value, icp_participant_prefix)
            ]
            require(
                len(private_tf_publishers) == 1,
                "private disabled TF endpoint ownership mismatch",
            )
            private_tf_subscriptions = [
                value
                for value in node.get_subscriptions_info_by_topic(ICP_TF_DISABLED_TOPIC)
                if same_participant(value, icp_participant_prefix)
            ]
            require(
                len(private_tf_subscriptions) >= 1,
                "ICP participant lacks its isolated dynamic-TF listener",
            )
            static_tf_subscriptions = [
                value
                for value in node.get_subscriptions_info_by_topic("/tf_static")
                if same_participant(value, icp_participant_prefix)
            ]
            require(
                len(static_tf_subscriptions) >= 1,
                "ICP is not subscribed to required static sensor transforms",
            )
            source_imu_subscriptions = [
                value
                for value in node.get_subscriptions_info_by_topic(SOURCE_IMU_TOPIC)
                if same_participant(value, icp_participant_prefix)
            ]
            selected_imu_subscriptions = [
                value
                for value in node.get_subscriptions_info_by_topic(
                    input_profile["imu_topic"]
                )
                if same_participant(value, icp_participant_prefix)
            ]
            if input_profile["consumes_source_imu"]:
                require(
                    len(source_imu_subscriptions) >= 1,
                    "IMU-assisted ICP lacks its /scanner/imu subscription",
                )
            else:
                require(
                    len(source_imu_subscriptions) == 0,
                    "cloud-only ICP unexpectedly subscribes to /scanner/imu",
                )
            forbidden_graph = {}
            for topic in FORBIDDEN_MOTION_TOPICS:
                publishers = node.get_publishers_info_by_topic(topic)
                subscriptions = node.get_subscriptions_info_by_topic(topic)
                require(not any(same_participant(value, icp_participant_prefix) for value in publishers), f"ICP participant unexpectedly publishes {topic}")
                require(not any(same_participant(value, icp_participant_prefix) for value in subscriptions), f"ICP participant unexpectedly subscribes {topic}")
                forbidden_graph[topic] = {
                    "icp_publishers": 0,
                    "icp_subscriptions": 0,
                }
            graph_evidence = {
                "private_odom_publishers": [endpoint_record(value) for value in private_publishers],
                "private_odom_info_publishers": [
                    endpoint_record(value)
                    for value in private_odom_info_publishers
                ],
                "icp_participant_prefix": icp_participant_prefix.hex(),
                "canonical_odom_icp_publishers": 0,
                "tf_icp_publishers": 0,
                "tf_icp_subscriptions": 0,
                "private_tf_disabled_topic": ICP_TF_DISABLED_TOPIC,
                "private_tf_disabled_publishers": [
                    endpoint_record(value) for value in private_tf_publishers
                ],
                "private_tf_disabled_subscriptions": [
                    endpoint_record(value) for value in private_tf_subscriptions
                ],
                "tf_static_icp_subscriptions": [
                    endpoint_record(value) for value in static_tf_subscriptions
                ],
                "input_profile": arguments.input_profile,
                "selected_imu_topic": input_profile["imu_topic"],
                "source_imu_subscriptions": [
                    endpoint_record(value) for value in source_imu_subscriptions
                ],
                "selected_imu_subscriptions": [
                    endpoint_record(value) for value in selected_imu_subscriptions
                ],
                "source_cloud_publisher": source_cloud_record,
                "source_imu_publisher": source_imu_record,
                "continuous_graph_checks": 0,
                "forbidden_motion_topics": forbidden_graph,
            }

            observation_deadline = time.perf_counter_ns() + arguments.duration_seconds * 1_000_000_000
            next_wrapper_check = time.perf_counter()
            while time.perf_counter_ns() < observation_deadline:
                if process.poll() is not None:
                    raise GateError(f"ICP process exited early with status {process.returncode}")
                if (session_dir / "fault.json").exists():
                    raise GateError("timestamp session faulted during observation")
                if time.perf_counter() >= next_wrapper_check:
                    require(
                        process_start_ticks(gate_evidence["wrapper_pid"])
                        == gate_evidence["wrapper_start_ticks"],
                        "no-motion wrapper stopped or PID was reused",
                    )
                    require(
                        [
                            endpoint_record(value)
                            for value in node.get_publishers_info_by_topic(
                                SOURCE_CLOUD_TOPIC
                            )
                        ]
                        == [source_cloud_record],
                        "/scanner/cloud publisher identity changed",
                    )
                    require(
                        [
                            endpoint_record(value)
                            for value in node.get_publishers_info_by_topic(
                                SOURCE_IMU_TOPIC
                            )
                        ]
                        == [source_imu_record],
                        "/scanner/imu publisher identity changed",
                    )
                    require(
                        not any(
                            same_participant(value, icp_participant_prefix)
                            for value in node.get_publishers_info_by_topic("/tf")
                        ),
                        "ICP acquired a canonical /tf publisher during observation",
                    )
                    require(
                        not any(
                            same_participant(value, icp_participant_prefix)
                            for value in node.get_subscriptions_info_by_topic("/tf")
                        ),
                        "ICP acquired a canonical /tf subscription during observation",
                    )
                    current_private_tf = [
                        value
                        for value in node.get_publishers_info_by_topic(
                            ICP_TF_DISABLED_TOPIC
                        )
                        if same_participant(value, icp_participant_prefix)
                    ]
                    require(
                        len(current_private_tf) == 1,
                        "private disabled TF endpoint changed during observation",
                    )
                    current_odom_info_publishers = [
                        value
                        for value in node.get_publishers_info_by_topic(
                            ICP_ODOM_INFO_TOPIC
                        )
                        if same_participant(value, icp_participant_prefix)
                    ]
                    require(
                        len(current_odom_info_publishers) == 1,
                        "private odom_info publisher changed during observation",
                    )
                    current_private_tf_subscriptions = [
                        value
                        for value in node.get_subscriptions_info_by_topic(
                            ICP_TF_DISABLED_TOPIC
                        )
                        if same_participant(value, icp_participant_prefix)
                    ]
                    require(
                        len(current_private_tf_subscriptions) >= 1,
                        "isolated dynamic-TF listener disappeared during observation",
                    )
                    require(
                        any(
                            same_participant(value, icp_participant_prefix)
                            for value in node.get_subscriptions_info_by_topic(
                                "/tf_static"
                            )
                        ),
                        "static-TF listener disappeared during observation",
                    )
                    current_source_imu_subscriptions = [
                        value
                        for value in node.get_subscriptions_info_by_topic(
                            SOURCE_IMU_TOPIC
                        )
                        if same_participant(value, icp_participant_prefix)
                    ]
                    if input_profile["consumes_source_imu"]:
                        require(
                            len(current_source_imu_subscriptions) >= 1,
                            "IMU-assisted ICP lost its /scanner/imu subscription",
                        )
                    else:
                        require(
                            len(current_source_imu_subscriptions) == 0,
                            "cloud-only ICP acquired a /scanner/imu subscription",
                        )
                    for topic in FORBIDDEN_MOTION_TOPICS:
                        require(
                            not any(
                                same_participant(value, icp_participant_prefix)
                                for value in node.get_publishers_info_by_topic(topic)
                            ),
                            f"ICP participant acquired a {topic} publisher",
                        )
                        require(
                            not any(
                                same_participant(value, icp_participant_prefix)
                                for value in node.get_subscriptions_info_by_topic(topic)
                            ),
                            f"ICP participant acquired a {topic} subscription",
                        )
                    require(
                        tf_disabled_message_count == 0,
                        "publish_tf=false process emitted a private disabled TF message",
                    )
                    graph_evidence["continuous_graph_checks"] += 1
                    next_wrapper_check = time.perf_counter() + 5.0
                remaining_s = max(0.0, (observation_deadline - time.perf_counter_ns()) / 1e9)
                rclpy.spin_once(node, timeout_sec=min(0.1, remaining_s))
            require(
                process.poll() is None,
                "ICP process exited at the observation boundary",
            )
            require(
                not (session_dir / "fault.json").exists(),
                "timestamp session faulted at the observation boundary",
            )
            require(
                [
                    endpoint_record(value)
                    for value in node.get_publishers_info_by_topic(
                        ICP_ODOM_TOPIC
                    )
                    if same_participant(value, icp_participant_prefix)
                ]
                == [endpoint_record(private_publishers[0])],
                "private odom publisher changed at the observation boundary",
            )
            graph_evidence["final_graph_check"] = True
            exit_reason = "duration_elapsed"
        except KeyboardInterrupt:
            exit_reason = "operator_interrupt"
            runtime_errors.append("operator interrupted evaluation")
        except Exception as exc:
            exit_reason = "gate_or_runtime_error"
            runtime_errors.append(f"{type(exc).__name__}: {exc}")
        finally:
            # Freeze the observation boundary before process shutdown.  A
            # clean ICP SIGINT may take seconds and must not look like a sensor
            # freshness failure in the stationary evidence.
            measurement_finished_ns = time.perf_counter_ns()
            if process is not None:
                stop_owned_process(process, cleanup_errors)
            if child_log is not None:
                child_log.flush()
                os.fsync(child_log.fileno())
                child_log.close()
            if node is not None:
                try:
                    node.destroy_node()
                except Exception as exc:
                    cleanup_errors.append(f"destroy_node: {type(exc).__name__}: {exc}")
            if initialized:
                try:
                    rclpy.shutdown(uninstall_handlers=False)
                except Exception as exc:
                    cleanup_errors.append(f"rclpy_shutdown: {type(exc).__name__}: {exc}")
            samples_stream.flush()
            os.fsync(samples_stream.fileno())
            samples_stream.close()

        assert measurement_finished_ns is not None
        stream_summaries = {
            "private_icp": private_series.summary(arguments.warmup_seconds, measurement_finished_ns),
            "private_icp_info": odom_info_series.summary(measurement_finished_ns),
            "reference_odom": reference_series.summary(arguments.warmup_seconds, measurement_finished_ns),
        }
        result: dict[str, Any] = {
            "schema": "robonix-go2-private-icp-nomotion-result-v1",
            "safe_for_motion": False,
            "exit_reason": exit_reason,
            "runtime_errors": runtime_errors,
            "cleanup_errors": cleanup_errors,
            "graph_evidence": graph_evidence,
            "private_tf_disabled_messages": tf_disabled_message_count,
            "streams": stream_summaries,
            "finished_realtime_ns": time.time_ns(),
            "note": (
                "A PASS only qualifies this stationary private diagnostic. It "
                "does not replace /odom, publish TF, authorize Nav2, or authorize motion."
            ),
        }
        gates = evaluate_gates(result, arguments)
        result["gates"] = gates
        result["passed"] = (
            exit_reason == "duration_elapsed"
            and not runtime_errors
            and not cleanup_errors
            and all(check["passed"] for check in gates)
        )
        write_json_exclusive(output_dir / "summary.json", result)
        print(f"Private ICP no-motion evidence: {output_dir}")
        print("PASS" if result["passed"] else "FAIL")
        return 0 if result["passed"] else 1
    finally:
        lock_stream.close()


def main(argv: Sequence[str] | None = None) -> int:
    try:
        return run(build_argument_parser().parse_args(argv))
    except GateError as exc:
        print(f"ICP no-motion gate refused: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
