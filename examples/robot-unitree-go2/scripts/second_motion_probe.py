#!/usr/bin/env python3
"""Supervised 20 cm second-motion probe on an independent command topic.

The first-motion probe remains unchanged and is loaded as a private policy
core.  This process supplies a distinct profile, topic, acknowledgement,
envelope, ownership checker and evidence schema before that core can import
ROS.  It never calls Unitree SDK APIs directly.
"""

from __future__ import annotations

import builtins
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Mapping


CORE_PATH = Path(__file__).with_name("first_motion_probe.py")
_SPEC = importlib.util.spec_from_file_location(
    "_robonix_second_motion_probe_core", CORE_PATH
)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError("could not load the audited commissioning probe core")
_core = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _core
_SPEC.loader.exec_module(_core)

COMMAND_TOPIC = "/go2/second_motion/cmd_vel"
MOTION_PROFILE = "workstation-second-motion-corrected-v1"
PROBE_NODE_NAME = "go2_second_motion_probe"
SECOND_MOTION_ACK = "I_APPROVE_GO2_SECOND_20CM_MOTION"
EVIDENCE_SCHEMA = "robonix-go2-second-motion-v1"

SPEED_MPS = 0.30
DISTANCE_LIMIT_M = 0.30
DURATION_LIMIT_S = 1.5
PROBE_STOP_DISTANCE_M = 0.20
PROBE_STOP_DURATION_S = 1.2
MIN_MEASURED_DISPLACEMENT_M = 0.15
MAX_MEASURED_LATERAL_M = 0.05
MAX_MEASURED_YAW_CHANGE_RAD = 0.20

# Re-export the pure policy types and unchanged helpers for focused tests.
CommissioningError = _core.CommissioningError
MotionDecision = _core.MotionDecision
PostStopDecision = _core.PostStopDecision
MeasuredMotion = _core.MeasuredMotion
CANONICAL_COMMAND_TOPIC = _core.CANONICAL_COMMAND_TOPIC
SPORT_REQUEST_TOPIC = _core.SPORT_REQUEST_TOPIC
SPORT_LEASE_REQUEST_TOPIC = _core.SPORT_LEASE_REQUEST_TOPIC
STATE_TOPIC = _core.STATE_TOPIC
ODOM_TOPIC = _core.ODOM_TOPIC
EXTERNAL_ODOM_TOPIC = _core.EXTERNAL_ODOM_TOPIC
DIAGNOSTICS_TOPIC = _core.DIAGNOSTICS_TOPIC
ARM_SERVICE = _core.ARM_SERVICE
SPORT_GRAPH_BASELINE_SCHEMA = _core.SPORT_GRAPH_BASELINE_SCHEMA
COMMAND_STALE_S = _core.COMMAND_STALE_S
DDS_GID_SIZE = _core.DDS_GID_SIZE
ZERO_PREAMBLE_S = _core.ZERO_PREAMBLE_S
POST_STOP_OBSERVATION_S = _core.POST_STOP_OBSERVATION_S
POST_STOP_MAX_LINEAR_MPS = _core.POST_STOP_MAX_LINEAR_MPS
POST_STOP_MAX_YAW_RPS = _core.POST_STOP_MAX_YAW_RPS
POST_STOP_MAX_DRIFT_M = _core.POST_STOP_MAX_DRIFT_M
POST_STOP_MIN_ODOM_SAMPLES = _core.POST_STOP_MIN_ODOM_SAMPLES


def validate_environment(
    environ: Mapping[str, str],
) -> tuple[frozenset[int], frozenset[int]]:
    """Require the independent second-stage operator acknowledgement."""

    required = {
        "GO2_ALLOW_MOTION": "true",
        "GO2_OPERATOR_PRESENT": "true",
        "GO2_SAFETY_ACK": "I_UNDERSTAND_GO2_CAN_MOVE",
        "GO2_SECOND_MOTION_ACK": SECOND_MOTION_ACK,
    }
    for key, expected in required.items():
        if environ.get(key) != expected:
            raise CommissioningError(f"{key} must exactly equal {expected!r}")
    interface = environ.get("GO2_NETWORK_INTERFACE", "")
    if not interface or any(character.isspace() for character in interface):
        raise CommissioningError(
            "GO2_NETWORK_INTERFACE must name the audited NIC"
        )
    modes = _core._parse_decimal_set(
        environ.get("GO2_ALLOWED_MODES", ""),
        "GO2_ALLOWED_MODES",
        minimum=0,
        maximum=254,
    )
    marker_text = environ.get("GO2_ALLOWED_STATE_MARKERS", "").strip()
    markers = (
        _core._parse_decimal_set(
            marker_text,
            "GO2_ALLOWED_STATE_MARKERS",
            minimum=1,
            maximum=4_294_967_295,
        )
        if marker_text
        else frozenset()
    )
    return modes, markers


def require_measured_displacement(measurement: MeasuredMotion) -> None:
    """Accept only a bounded, mostly-forward measured second-stage result."""

    values = (
        measurement.distance_m,
        measurement.forward_m,
        measurement.lateral_m,
        measurement.yaw_change_rad,
    )
    if not all(math.isfinite(value) for value in values):
        raise CommissioningError("measured odometry displacement is invalid")
    if measurement.distance_m < 0.0:
        raise CommissioningError("measured odometry displacement is invalid")
    if measurement.distance_m >= DISTANCE_LIMIT_M:
        raise CommissioningError(
            "measured total odometry displacement "
            f"{measurement.distance_m:.6f} m reached the "
            f"{DISTANCE_LIMIT_M:.6f} m hard envelope"
        )
    if measurement.forward_m < MIN_MEASURED_DISPLACEMENT_M:
        raise CommissioningError(
            "measured forward odometry displacement "
            f"{measurement.forward_m:.6f} m is below the "
            f"{MIN_MEASURED_DISPLACEMENT_M:.6f} m acceptance floor"
        )
    if abs(measurement.lateral_m) > MAX_MEASURED_LATERAL_M:
        raise CommissioningError(
            "measured lateral odometry displacement "
            f"{measurement.lateral_m:.6f} m exceeds the "
            f"{MAX_MEASURED_LATERAL_M:.6f} m straight-line limit"
        )
    if abs(measurement.yaw_change_rad) > MAX_MEASURED_YAW_CHANGE_RAD:
        raise CommissioningError(
            "measured odometry yaw change "
            f"{measurement.yaw_change_rad:.6f} rad exceeds the "
            f"{MAX_MEASURED_YAW_CHANGE_RAD:.6f} rad straight-line limit"
        )


def _write_evidence(payload: Mapping[str, object]) -> Path:
    root = Path(__file__).resolve().parents[1]
    directory = root / "logs" / "go2-motion"
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    unix_ns = time.time_ns()
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(unix_ns / 1e9))
    path = directory / f"second-motion-{stamp}-{unix_ns}.json"
    second_payload = dict(payload)
    second_payload["schema"] = EVIDENCE_SCHEMA
    encoded = (
        json.dumps(
            second_payload,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")
    descriptor = os.open(
        path,
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | os.O_CLOEXEC
        | os.O_NOFOLLOW,
        0o600,
    )
    try:
        view = memoryview(encoded)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise OSError(
                    "short write while persisting second-motion evidence"
                )
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return path


def _second_motion_print(*args, **kwargs) -> None:
    if args == (
        "FIRST-MOTION PROBE: max 0.05 m/s; soft stop 0.09 m / 1.8 s; "
        "hard envelope 0.10 m / 2.0 s",
    ):
        args = (
            "SECOND-MOTION PROBE: max 0.30 m/s; soft stop 0.20 m / "
            "1.2 s; hard envelope 0.30 m / 1.5 s",
        )
    builtins.print(*args, **kwargs)


_core.COMMAND_TOPIC = COMMAND_TOPIC
_core.MOTION_PROFILE = MOTION_PROFILE
_core.PROBE_NODE_NAME = PROBE_NODE_NAME
_core.SPEED_MPS = SPEED_MPS
_core.DISTANCE_LIMIT_M = DISTANCE_LIMIT_M
_core.DURATION_LIMIT_S = DURATION_LIMIT_S
_core.PROBE_STOP_DISTANCE_M = PROBE_STOP_DISTANCE_M
_core.PROBE_STOP_DURATION_S = PROBE_STOP_DURATION_S
_core.MIN_MEASURED_DISPLACEMENT_M = MIN_MEASURED_DISPLACEMENT_M
_core.MAX_MEASURED_LATERAL_M = MAX_MEASURED_LATERAL_M
_core.MAX_MEASURED_YAW_CHANGE_RAD = MAX_MEASURED_YAW_CHANGE_RAD
_core.require_measured_displacement = require_measured_displacement
_core._write_evidence = _write_evidence
_core.print = _second_motion_print

motion_decision = _core.motion_decision
normalize_angle = _core.normalize_angle
quaternion_yaw = _core.quaternion_yaw
measured_motion = _core.measured_motion
post_stop_decision = _core.post_stop_decision
assert_fresh = _core.assert_fresh
diagnostic_level = _core.diagnostic_level
diagnostic_evidence = _core.diagnostic_evidence
sport_state_evidence_snapshot = _core.sport_state_evidence_snapshot
sport_state_pair_evidence = _core.sport_state_pair_evidence
RUNTIME_GRAPH_CHECK_PERIOD_S = _core.RUNTIME_GRAPH_CHECK_PERIOD_S
bind_added_sport_writer = _core.bind_added_sport_writer
load_sport_request_baseline = _core.load_sport_request_baseline
_run_ros = _core._run_ros


def main() -> int:
    if len(sys.argv) != 1:
        print(f"usage: {Path(sys.argv[0]).name}", file=sys.stderr)
        return 2
    try:
        allowed_modes, allowed_markers = validate_environment(os.environ)
        sport_request_baseline_gids = load_sport_request_baseline(os.environ)
    except CommissioningError as exc:
        print(f"refusing second motion: {exc}", file=sys.stderr)
        return 2
    return _run_ros(
        allowed_modes,
        allowed_markers,
        sport_request_baseline_gids,
    )


if __name__ == "__main__":
    raise SystemExit(main())
