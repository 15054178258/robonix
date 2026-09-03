#!/usr/bin/env python3
"""Supervised, bounded first-motion probe for the Go2 chassis adapter.

This is deliberately not a general teleoperation tool. It can publish only a
forward 0.05 m/s Twist on a dedicated commissioning topic. Its normal soft
stop is 1.8 s or 0.09 m, inside the adapter/daemon hard envelope of 2.0 s or
0.10 m. Every exit path publishes zero and explicitly requests adapter disarm.
It never calls Unitree SDK APIs directly.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from typing import Mapping, Sequence


COMMAND_TOPIC = "/go2/commissioning/cmd_vel"
CANONICAL_COMMAND_TOPIC = "/cmd_vel"
SPORT_REQUEST_TOPIC = "/api/sport/request"
SPORT_LEASE_REQUEST_TOPIC = "/api/sport_lease/request"
STATE_TOPIC = "/robonix/time_corrected/motion/sportmodestate"
ODOM_TOPIC = "/odom"
EXTERNAL_ODOM_TOPIC = "/robonix/time_corrected/raw/utlidar/robot_odom"
DIAGNOSTICS_TOPIC = "/diagnostics"
ARM_SERVICE = "/go2_chassis/arm"
MOTION_PROFILE = "workstation-first-motion-corrected-v1"
PROBE_NODE_NAME = "go2_first_motion_probe"
ADAPTER_NODE_NAME = "go2_chassis_adapter"
ADAPTER_NAMESPACE = "/"
DIAGNOSTIC_NAME = "go2_chassis_adapter"
SPORT_GRAPH_BASELINE_SCHEMA = "robonix-go2-motion-rpc-graph-baseline-v1"

SPEED_MPS = 0.05
# Adapter/daemon hard envelope; never use it as the normal probe target.
DISTANCE_LIMIT_M = 0.10
DURATION_LIMIT_S = 2.0
# Normal probe target, leaving room for zero, Stop/Disarm and odometry
# differences before the independent hard envelope.
PROBE_STOP_DISTANCE_M = 0.09
PROBE_STOP_DURATION_S = 1.8
# A successful SDK/DDS emission is not physical-motion evidence. Require a
# small but clearly non-zero displacement from the independent odometry chain.
MIN_MEASURED_DISPLACEMENT_M = 0.02
MAX_MEASURED_LATERAL_M = 0.03
MAX_MEASURED_YAW_CHANGE_RAD = 0.20
CONTROL_PERIOD_S = 0.02
ZERO_PREAMBLE_S = 0.60
ARMED_ZERO_TIMEOUT_S = 2.0
STOP_ZERO_HOLD_S = 0.35
POST_STOP_OBSERVATION_S = 1.0
POST_STOP_MAX_LINEAR_MPS = 0.03
POST_STOP_MAX_YAW_RPS = 0.03
POST_STOP_MAX_DRIFT_M = 0.02
POST_STOP_MIN_ODOM_SAMPLES = 10
STATE_STALE_S = 0.20
COMMAND_STALE_S = 0.20
ODOM_STALE_S = 0.20
DIAGNOSTIC_STALE_S = 0.35
GRAPH_STABILITY_SAMPLES = 3
GRAPH_SAMPLE_PERIOD_S = 0.10
RUNTIME_GRAPH_CHECK_PERIOD_S = 0.50
DDS_GID_SIZE = 24
SPORT_GRAPH_BASELINE_MAX_AGE_S = 120.0
SPORT_STATE_POSITION_ABS_LIMIT_M = 1_000_000.0
SPORT_STATE_VELOCITY_ABS_LIMIT_MPS = 100.0
SPORT_STATE_YAW_SPEED_ABS_LIMIT_RPS = 100.0
FIRST_MOTION_ACK = "I_APPROVE_GO2_FIRST_10CM_MOTION"
DIAGNOSTIC_EVIDENCE_KEYS = (
    "guard_state",
    "allow_motion",
    "motion_profile",
    "odom_source",
    "external_odom_topic",
    "external_odom_valid",
    "external_odom_age_sec",
    "external_odom_timeout_sec",
    "external_odom_status",
    "external_odom_fault_latched",
    "daemon_armed",
    "commissioning_permit_spent",
    "commissioning_motion_active",
    "commissioning_stop_reason",
    "commissioning_elapsed_sec",
    "commissioning_distance_m",
    "state_age_sec",
    "state_timeout_sec",
    "state_valid",
    "source_stamp_status",
    "source_stamp_last_fault",
    "source_stamp_rejection_count",
    "max_source_stamp_age_sec",
    "max_source_stamp_future_skew_sec",
)


class CommissioningError(RuntimeError):
    """A fail-closed first-motion precondition or runtime failure."""


@dataclass(frozen=True)
class MotionDecision:
    continue_motion: bool
    reason: str
    elapsed_s: float
    distance_m: float


@dataclass(frozen=True)
class PostStopDecision:
    stationary: bool
    reason: str
    drift_m: float
    linear_speed_mps: float
    yaw_rate_rps: float


@dataclass(frozen=True)
class MeasuredMotion:
    distance_m: float
    forward_m: float
    lateral_m: float
    yaw_change_rad: float


def _bounded_evidence_scalar(
    value: object,
    label: str,
    absolute_limit: float,
) -> float:
    if isinstance(value, bool):
        raise CommissioningError(f"{label} is not numeric")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise CommissioningError(f"{label} is not numeric") from exc
    if not math.isfinite(number) or abs(number) > absolute_limit:
        raise CommissioningError(f"{label} is outside its evidence bound")
    return number


def sport_state_evidence_snapshot(
    position: Sequence[object],
    velocity: Sequence[object],
    yaw_speed: object,
    mode: object,
    gait_type: object,
) -> dict[str, object]:
    """Return a finite, bounded firmware-state snapshot for evidence only."""

    if (
        isinstance(position, (str, bytes, bytearray))
        or len(position) < 2
        or isinstance(velocity, (str, bytes, bytearray))
        or len(velocity) < 2
    ):
        raise CommissioningError(
            "SportModeState position/velocity evidence is incomplete"
        )
    if (
        not isinstance(mode, int)
        or isinstance(mode, bool)
        or not 0 <= mode <= 255
        or not isinstance(gait_type, int)
        or isinstance(gait_type, bool)
        or not 0 <= gait_type <= 255
    ):
        raise CommissioningError(
            "SportModeState mode/gait evidence is outside uint8"
        )
    return {
        "position_xy_m": [
            _bounded_evidence_scalar(
                position[index],
                f"SportModeState position[{index}]",
                SPORT_STATE_POSITION_ABS_LIMIT_M,
            )
            for index in range(2)
        ],
        "velocity_xy_mps": [
            _bounded_evidence_scalar(
                velocity[index],
                f"SportModeState velocity[{index}]",
                SPORT_STATE_VELOCITY_ABS_LIMIT_MPS,
            )
            for index in range(2)
        ],
        "yaw_speed_rps": _bounded_evidence_scalar(
            yaw_speed,
            "SportModeState yaw_speed",
            SPORT_STATE_YAW_SPEED_ABS_LIMIT_RPS,
        ),
        "mode": mode,
        "gait_type": gait_type,
    }


def sport_state_pair_evidence(
    start: Mapping[str, object] | None,
    end: Mapping[str, object] | None,
    validation_issue: str,
) -> dict[str, object]:
    """Build bounded start/end and raw-position-delta evidence."""

    payload: dict[str, object] = {
        "start": start,
        "end": end,
        "raw_position_delta_xy_m": None,
        "raw_position_delta_norm_m": None,
        "validation_issue": str(validation_issue)[:256],
    }
    if start is None or end is None:
        return payload
    start_position = start.get("position_xy_m")
    end_position = end.get("position_xy_m")
    if (
        not isinstance(start_position, Sequence)
        or isinstance(start_position, (str, bytes, bytearray))
        or not isinstance(end_position, Sequence)
        or isinstance(end_position, (str, bytes, bytearray))
        or len(start_position) != 2
        or len(end_position) != 2
    ):
        raise CommissioningError(
            "SportModeState position delta evidence is malformed"
        )
    delta = [
        _bounded_evidence_scalar(
            end_position[index],
            f"SportModeState end position[{index}]",
            SPORT_STATE_POSITION_ABS_LIMIT_M,
        )
        - _bounded_evidence_scalar(
            start_position[index],
            f"SportModeState start position[{index}]",
            SPORT_STATE_POSITION_ABS_LIMIT_M,
        )
        for index in range(2)
    ]
    if not all(
        math.isfinite(value)
        and abs(value) <= 2.0 * SPORT_STATE_POSITION_ABS_LIMIT_M
        for value in delta
    ):
        raise CommissioningError(
            "SportModeState raw position delta is outside its evidence bound"
        )
    payload["raw_position_delta_xy_m"] = delta
    payload["raw_position_delta_norm_m"] = math.hypot(*delta)
    return payload


def _parse_decimal_set(
    value: str, label: str, *, minimum: int, maximum: int
) -> frozenset[int]:
    if not value.strip():
        raise CommissioningError(f"{label} must be a completed audit value")
    parsed: set[int] = set()
    for item in value.split(","):
        item = item.strip()
        if not item.isdecimal():
            raise CommissioningError(f"{label} must contain decimal integers")
        number = int(item, 10)
        if not minimum <= number <= maximum:
            raise CommissioningError(f"{label} value is outside its audited range")
        parsed.add(number)
    return frozenset(parsed)


def validate_environment(environ: Mapping[str, str]) -> tuple[frozenset[int], frozenset[int]]:
    """Validate explicit operator/session gates without accepting fuzzy values."""
    required = {
        "GO2_ALLOW_MOTION": "true",
        "GO2_OPERATOR_PRESENT": "true",
        "GO2_SAFETY_ACK": "I_UNDERSTAND_GO2_CAN_MOVE",
        "GO2_FIRST_MOTION_ACK": FIRST_MOTION_ACK,
    }
    for key, expected in required.items():
        if environ.get(key) != expected:
            raise CommissioningError(f"{key} must exactly equal {expected!r}")
    interface = environ.get("GO2_NETWORK_INTERFACE", "")
    if not interface or any(character.isspace() for character in interface):
        raise CommissioningError("GO2_NETWORK_INTERFACE must name the audited NIC")
    modes = _parse_decimal_set(
        environ.get("GO2_ALLOWED_MODES", ""),
        "GO2_ALLOWED_MODES",
        minimum=0,
        maximum=254,
    )
    marker_text = environ.get("GO2_ALLOWED_STATE_MARKERS", "").strip()
    markers = (
        _parse_decimal_set(
            marker_text,
            "GO2_ALLOWED_STATE_MARKERS",
            minimum=1,
            maximum=4_294_967_295,
        )
        if marker_text
        else frozenset()
    )
    return modes, markers


def load_sport_request_baseline(environ: Mapping[str, str]) -> frozenset[bytes]:
    text = environ.get("GO2_SPORT_REQUEST_BASELINE_FILE", "")
    path = Path(text)
    if not text or not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise CommissioningError(
            "GO2_SPORT_REQUEST_BASELINE_FILE must be an absolute regular "
            "non-symlink file"
        )
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CommissioningError("motion RPC graph baseline is unreadable") from exc
    if (
        not isinstance(payload, dict)
        or payload.get("schema") != SPORT_GRAPH_BASELINE_SCHEMA
    ):
        raise CommissioningError("motion RPC graph baseline schema is invalid")
    if payload.get("sport_request_topic") != SPORT_REQUEST_TOPIC:
        raise CommissioningError("motion RPC graph baseline topic is invalid")
    if payload.get("sport_lease_request_topic") != SPORT_LEASE_REQUEST_TOPIC:
        raise CommissioningError("motion RPC lease baseline topic is invalid")
    if payload.get("sport_lease_request_writer_count") != 0:
        raise CommissioningError("motion RPC baseline contains a lease writer")
    captured_unix_ns = payload.get("captured_unix_ns")
    if not isinstance(captured_unix_ns, int) or isinstance(captured_unix_ns, bool):
        raise CommissioningError("motion RPC graph baseline timestamp is invalid")
    age_s = (time.time_ns() - captured_unix_ns) / 1_000_000_000.0
    if (
        not math.isfinite(age_s)
        or age_s < 0.0
        or age_s > SPORT_GRAPH_BASELINE_MAX_AGE_S
    ):
        raise CommissioningError("motion RPC graph baseline is stale")
    encoded_gids = payload.get("sport_request_writer_gids")
    if not isinstance(encoded_gids, list) or not all(
        isinstance(value, str) for value in encoded_gids
    ):
        raise CommissioningError("motion RPC graph baseline GID list is invalid")
    try:
        gids = tuple(bytes.fromhex(value) for value in encoded_gids)
    except ValueError as exc:
        raise CommissioningError("motion RPC graph baseline GID is malformed") from exc
    if any(len(gid) != DDS_GID_SIZE or not any(gid) for gid in gids):
        raise CommissioningError("motion RPC graph baseline GID is invalid")
    if len(gids) != len(set(gids)):
        raise CommissioningError("motion RPC graph baseline GIDs are duplicated")
    if payload.get("sport_request_writer_count") != len(gids):
        raise CommissioningError("motion RPC graph baseline count is inconsistent")
    return frozenset(gids)


def bind_added_sport_writer(
    baseline_gids: frozenset[bytes],
    endpoint_gids: Sequence[object],
    bound_gid: bytes | None,
) -> tuple[str, bytes | None]:
    normalized: list[bytes] = []
    for raw_gid in endpoint_gids:
        try:
            gid = bytes(raw_gid)
        except (TypeError, ValueError, OverflowError):
            return "sport request writer has no usable DDS GID", bound_gid
        if len(gid) != DDS_GID_SIZE or not any(gid):
            return "sport request writer DDS GID is invalid", bound_gid
        normalized.append(gid)
    current_gids = frozenset(normalized)
    if len(current_gids) != len(normalized):
        return "sport request graph contains duplicate DDS GIDs", bound_gid
    if not baseline_gids.issubset(current_gids):
        return "pre-boot sport request writer baseline changed", bound_gid
    additions = current_gids - baseline_gids
    if len(additions) != 1:
        return "motion boot did not add exactly one sport request writer", bound_gid
    candidate = next(iter(additions))
    if bound_gid is not None and candidate != bound_gid:
        return "sport request writer DDS GID changed during the probe", bound_gid
    return "", candidate


def motion_decision(
    start_monotonic_s: float,
    now_monotonic_s: float,
    start_xy: tuple[float, float],
    current_xy: tuple[float, float],
) -> MotionDecision:
    values = (*start_xy, *current_xy, start_monotonic_s, now_monotonic_s)
    if not all(math.isfinite(value) for value in values):
        raise CommissioningError("non-finite odometry or monotonic time")
    elapsed_s = now_monotonic_s - start_monotonic_s
    if elapsed_s < 0.0:
        raise CommissioningError("monotonic clock regressed")
    distance_m = math.hypot(
        current_xy[0] - start_xy[0], current_xy[1] - start_xy[1]
    )
    if distance_m >= PROBE_STOP_DISTANCE_M:
        return MotionDecision(False, "distance_limit", elapsed_s, distance_m)
    if elapsed_s >= PROBE_STOP_DURATION_S:
        return MotionDecision(False, "duration_limit", elapsed_s, distance_m)
    return MotionDecision(True, "within_limits", elapsed_s, distance_m)


def normalize_angle(angle_rad: float) -> float:
    if not math.isfinite(angle_rad):
        raise CommissioningError("odometry yaw is non-finite")
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise CommissioningError("odometry quaternion is non-finite")
    norm = math.sqrt(sum(value * value for value in values))
    if not 0.99 <= norm <= 1.01:
        raise CommissioningError("odometry quaternion is not normalized")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def measured_motion(
    start_xy: tuple[float, float],
    start_yaw_rad: float,
    current_xy: tuple[float, float],
    current_yaw_rad: float,
) -> MeasuredMotion:
    values = (*start_xy, start_yaw_rad, *current_xy, current_yaw_rad)
    if not all(math.isfinite(value) for value in values):
        raise CommissioningError("measured odometry motion is non-finite")
    dx = current_xy[0] - start_xy[0]
    dy = current_xy[1] - start_xy[1]
    cos_yaw = math.cos(start_yaw_rad)
    sin_yaw = math.sin(start_yaw_rad)
    return MeasuredMotion(
        distance_m=math.hypot(dx, dy),
        forward_m=cos_yaw * dx + sin_yaw * dy,
        lateral_m=-sin_yaw * dx + cos_yaw * dy,
        yaw_change_rad=normalize_angle(current_yaw_rad - start_yaw_rad),
    )


def require_measured_displacement(measurement: MeasuredMotion) -> None:
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


def post_stop_decision(
    start_xy: tuple[float, float],
    current_xy: tuple[float, float],
    linear_speed_mps: float,
    yaw_rate_rps: float,
) -> PostStopDecision:
    values = (*start_xy, *current_xy, linear_speed_mps, yaw_rate_rps)
    if not all(math.isfinite(value) for value in values):
        raise CommissioningError("non-finite post-stop odometry")
    drift_m = math.hypot(
        current_xy[0] - start_xy[0], current_xy[1] - start_xy[1]
    )
    linear_speed_mps = abs(linear_speed_mps)
    yaw_rate_rps = abs(yaw_rate_rps)
    if linear_speed_mps > POST_STOP_MAX_LINEAR_MPS:
        reason = "post_stop_linear_speed"
    elif yaw_rate_rps > POST_STOP_MAX_YAW_RPS:
        reason = "post_stop_yaw_rate"
    elif drift_m > POST_STOP_MAX_DRIFT_M:
        reason = "post_stop_drift"
    else:
        return PostStopDecision(
            True,
            "stationary",
            drift_m,
            linear_speed_mps,
            yaw_rate_rps,
        )
    return PostStopDecision(
        False, reason, drift_m, linear_speed_mps, yaw_rate_rps
    )


def assert_fresh(now_s: float, receipt_s: float | None, limit_s: float, label: str) -> None:
    if receipt_s is None:
        raise CommissioningError(f"no {label} sample received")
    age_s = now_s - receipt_s
    if not math.isfinite(age_s) or age_s < 0.0 or age_s > limit_s:
        raise CommissioningError(f"{label} is stale ({age_s:.3f} s)")


def diagnostic_level(value: object) -> int:
    """Normalize ROS 2 ``byte`` and integer diagnostic levels."""

    if isinstance(value, int) and not isinstance(value, bool):
        level = value
    elif isinstance(value, (bytes, bytearray, memoryview)):
        raw = bytes(value)
        if len(raw) != 1:
            raise CommissioningError(
                "chassis diagnostic level byte must contain exactly one octet"
            )
        level = raw[0]
    else:
        raise CommissioningError(
            "chassis diagnostic level has an unsupported representation"
        )
    if not 0 <= level <= 255:
        raise CommissioningError("chassis diagnostic level is outside uint8")
    return level


def diagnostic_evidence(
    status_message: str,
    level: int | None,
    validation_issue: str,
    values: Mapping[str, str],
) -> dict[str, object]:
    """Return a bounded, actionable snapshot of the last chassis diagnostic."""

    return {
        "message": str(status_message),
        "level": level,
        "validation_issue": str(validation_issue),
        "values": {
            key: values[key]
            for key in DIAGNOSTIC_EVIDENCE_KEYS
            if key in values
        },
    }


def _write_evidence(payload: Mapping[str, object]) -> Path:
    root = Path(__file__).resolve().parents[1]
    directory = root / "logs" / "go2-motion"
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    unix_ns = time.time_ns()
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(unix_ns / 1e9))
    path = directory / f"first-motion-{stamp}-{unix_ns}.json"
    encoded = (
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC | os.O_NOFOLLOW,
        0o600,
    )
    try:
        view = memoryview(encoded)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise OSError("short write while persisting motion evidence")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return path


def _run_ros(
    allowed_modes: frozenset[int],
    allowed_markers: frozenset[int],
    sport_request_baseline_gids: frozenset[bytes],
) -> int:
    # ROS imports are intentionally lazy so the pure policy can be unit-tested
    # without a sourced ROS installation.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import SetBool
    from unitree_go.msg import SportModeState

    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    old_sigint = signal.signal(signal.SIGINT, request_stop)
    old_sigterm = signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)

    class FirstMotionNode(Node):
        def __init__(self) -> None:
            super().__init__(PROBE_NODE_NAME)
            self.state_receipt_s: float | None = None
            self.odom_receipt_s: float | None = None
            self.diagnostic_receipt_s: float | None = None
            self.state_stamp_ns: int | None = None
            self.odom_stamp_ns: int | None = None
            self.state_marker: int | None = None
            self.sport_state_evidence: dict[str, object] | None = None
            self.sport_state_evidence_issue = (
                "waiting for bounded SportModeState evidence"
            )
            self.odom_xy: tuple[float, float] | None = None
            self.odom_yaw_rad: float | None = None
            self.odom_linear_speed_mps: float | None = None
            self.odom_yaw_rate_rps: float | None = None
            self.odom_sample_count = 0
            self.state_issue = "waiting for SportModeState"
            self.odom_issue = "waiting for odometry"
            self.diagnostic_issue = "waiting for chassis diagnostics"
            self.latched_issue: str | None = None
            self.guard_state = "UNKNOWN"
            self.daemon_armed = False
            self.commissioning_permit_spent = False
            self.commissioning_motion_active = False
            self.commissioning_motion_active_observed = False
            self.commissioning_stop_reason = "not_received"
            self.sport_request_gid: bytes | None = None
            self.next_runtime_graph_check_s = 0.0
            self.last_diagnostic_evidence = diagnostic_evidence(
                "",
                None,
                self.diagnostic_issue,
                {},
            )

            command_qos = QoSProfile(depth=1)
            command_qos.reliability = ReliabilityPolicy.RELIABLE
            command_qos.durability = DurabilityPolicy.VOLATILE
            self.command_publisher = self.create_publisher(
                Twist, COMMAND_TOPIC, command_qos
            )
            state_qos = QoSProfile(depth=10)
            state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            state_qos.durability = DurabilityPolicy.VOLATILE
            self.create_subscription(
                SportModeState, STATE_TOPIC, self._on_state, state_qos
            )
            self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
            self.create_subscription(
                DiagnosticArray,
                DIAGNOSTICS_TOPIC,
                self._on_diagnostics,
                10,
            )
            self.arm_client = self.create_client(SetBool, ARM_SERVICE)

        def _latch(self, message: str) -> None:
            if self.latched_issue is None:
                self.latched_issue = message

        def _on_state(self, message: SportModeState) -> None:
            now_s = time.monotonic()
            sec = int(message.stamp.sec)
            nanosec = int(message.stamp.nanosec)
            stamp_ns = sec * 1_000_000_000 + nanosec
            issue: str | None = None
            if sec < 0 or not 0 <= nanosec < 1_000_000_000 or stamp_ns <= 0:
                issue = "SportModeState source stamp is invalid"
            elif self.state_stamp_ns is not None and stamp_ns <= self.state_stamp_ns:
                self._latch("SportModeState source stamp stopped progressing")
                issue = self.latched_issue
            mode = int(message.mode)
            gait_type = int(message.gait_type)
            marker = int(message.error_code)
            try:
                self.sport_state_evidence = sport_state_evidence_snapshot(
                    message.position,
                    message.velocity,
                    message.yaw_speed,
                    mode,
                    gait_type,
                )
                self.sport_state_evidence_issue = ""
            except CommissioningError as exc:
                # This snapshot exists only to compare firmware state with the
                # independent external odometry evidence. It is never a
                # motion, stop, or acceptance gate.
                self.sport_state_evidence_issue = str(exc)
            if mode not in allowed_modes:
                issue = f"SportModeState mode {mode} is not session-allowlisted"
            if marker != 0 and marker not in allowed_markers:
                issue = f"opaque state marker {marker} is not session-allowlisted"
            if self.state_marker is None:
                self.state_marker = marker
            elif marker != self.state_marker:
                self._latch(
                    f"opaque state marker changed from {self.state_marker} to {marker}"
                )
                issue = self.latched_issue
            if issue is None and self.latched_issue is None:
                self.state_stamp_ns = stamp_ns
                self.state_receipt_s = now_s
                self.state_issue = ""
            else:
                self.state_issue = issue or self.latched_issue or "invalid state"

        def _on_odom(self, message: Odometry) -> None:
            now_s = time.monotonic()
            sec = int(message.header.stamp.sec)
            nanosec = int(message.header.stamp.nanosec)
            stamp_ns = sec * 1_000_000_000 + nanosec
            x = float(message.pose.pose.position.x)
            y = float(message.pose.pose.position.y)
            qx = float(message.pose.pose.orientation.x)
            qy = float(message.pose.pose.orientation.y)
            qz = float(message.pose.pose.orientation.z)
            qw = float(message.pose.pose.orientation.w)
            vx = float(message.twist.twist.linear.x)
            vy = float(message.twist.twist.linear.y)
            wz = float(message.twist.twist.angular.z)
            issue: str | None = None
            yaw_rad: float | None = None
            if message.header.frame_id != "odom" or message.child_frame_id != "base_link":
                issue = "odometry frame is not odom -> base_link"
            elif not all(
                math.isfinite(value)
                for value in (x, y, qx, qy, qz, qw, vx, vy, wz)
            ):
                issue = "odometry pose or velocity is non-finite"
            else:
                try:
                    yaw_rad = quaternion_yaw(qx, qy, qz, qw)
                except CommissioningError as exc:
                    issue = str(exc)
            if issue is None and (
                sec < 0
                or not 0 <= nanosec < 1_000_000_000
                or stamp_ns <= 0
            ):
                issue = "odometry source stamp is invalid"
            elif (
                issue is None
                and self.odom_stamp_ns is not None
                and stamp_ns <= self.odom_stamp_ns
            ):
                self._latch("odometry source stamp stopped progressing")
                issue = self.latched_issue
            if issue is None and self.latched_issue is None:
                if yaw_rad is None:
                    self._latch("odometry yaw was not computed")
                    self.odom_issue = self.latched_issue
                    return
                self.odom_stamp_ns = stamp_ns
                self.odom_receipt_s = now_s
                self.odom_xy = (x, y)
                self.odom_yaw_rad = yaw_rad
                self.odom_linear_speed_mps = math.hypot(vx, vy)
                self.odom_yaw_rate_rps = abs(wz)
                self.odom_sample_count += 1
                self.odom_issue = ""
            else:
                self.odom_issue = issue or self.latched_issue or "invalid odometry"

        def _on_diagnostics(self, message: DiagnosticArray) -> None:
            matches = [status for status in message.status if status.name == DIAGNOSTIC_NAME]
            issue: str | None = None
            if len(matches) != 1:
                self.diagnostic_issue = "expected exactly one chassis diagnostic status"
                self.last_diagnostic_evidence = diagnostic_evidence(
                    "",
                    None,
                    self.diagnostic_issue,
                    {},
                )
                return
            status = matches[0]
            values = {entry.key: entry.value for entry in status.values}
            if len(values) != len(status.values):
                issue = "chassis diagnostic keys are duplicated"
            state_age_s = math.inf
            current_marker = -1
            max_duration_s = math.inf
            max_distance_m = math.inf
            command_timeout_s = math.inf
            try:
                state_age_s = float(values.get("state_age_sec", "nan"))
                current_marker = int(values.get("sport_error_code", "-1"), 10)
                max_duration_s = float(
                    values.get("commissioning_max_duration_sec", "nan")
                )
                max_distance_m = float(
                    values.get("commissioning_max_distance_m", "nan")
                )
                command_timeout_s = float(
                    values.get("commissioning_command_timeout_sec", "nan")
                )
            except ValueError:
                issue = "chassis diagnostic numeric value is malformed"
            try:
                level = diagnostic_level(status.level)
            except CommissioningError as exc:
                level = None
                if issue is None:
                    issue = str(exc)
            permit_spent_text = values.get("commissioning_permit_spent")
            motion_active_text = values.get("commissioning_motion_active")
            daemon_armed_text = values.get("daemon_armed")
            if issue is not None:
                pass
            elif permit_spent_text not in {"true", "false"}:
                issue = "adapter commissioning permit diagnostic is malformed"
            elif motion_active_text not in {"true", "false"}:
                issue = "adapter commissioning activity diagnostic is malformed"
            elif daemon_armed_text not in {"true", "false"}:
                issue = "adapter daemon arm diagnostic is malformed"
            elif not values.get("commissioning_stop_reason"):
                issue = "adapter commissioning stop reason is missing"
            elif level is None:
                issue = "chassis diagnostic level is malformed"
            elif level > 1:
                issue = "chassis diagnostic level is ERROR"
            elif values.get("allow_motion") != "true":
                issue = "chassis adapter motion graph is not configured"
            elif values.get("motion_profile") != MOTION_PROFILE:
                issue = (
                    "chassis adapter motion profile is not "
                    f"{MOTION_PROFILE}"
                )
            elif values.get("odom_source") != "external_verified":
                issue = "chassis adapter odometry is not externally verified"
            elif values.get("external_odom_topic") != EXTERNAL_ODOM_TOPIC:
                issue = "chassis adapter external odometry topic is unexpected"
            elif values.get("external_odom_valid") != "true":
                issue = "chassis adapter external odometry is not fresh/valid"
            elif values.get("external_odom_status") != "fresh":
                issue = "chassis adapter external odometry status is not fresh"
            elif values.get("external_odom_fault_latched") != "false":
                issue = "chassis adapter external odometry fault is latched"
            elif values.get("state_valid") != "true":
                issue = "chassis state_valid is not true"
            elif values.get("source_stamp_status") != "fresh":
                issue = "chassis source stamp is not fresh"
            elif not 0.0 <= state_age_s <= STATE_STALE_S:
                issue = "chassis diagnostic reports stale state"
            elif values.get("opaque_state_marker_change_latched") != "false":
                issue = "opaque state marker change is latched"
            elif max_duration_s != DURATION_LIMIT_S:
                issue = (
                    "adapter commissioning duration envelope is not "
                    f"{DURATION_LIMIT_S:.1f} s"
                )
            elif max_distance_m != DISTANCE_LIMIT_M:
                issue = (
                    "adapter commissioning distance envelope is not "
                    f"{DISTANCE_LIMIT_M:.2f} m"
                )
            elif command_timeout_s != COMMAND_STALE_S:
                issue = "adapter command watchdog is not 0.20 s"
            elif current_marker != 0:
                if values.get("opaque_state_marker_explicitly_allowed") != "true":
                    issue = "non-zero marker lacks explicit adapter proof"
                elif values.get("opaque_state_marker_bound") != str(current_marker):
                    issue = "adapter bound marker does not match current marker"
            self.guard_state = values.get("guard_state", "UNKNOWN")
            self.daemon_armed = daemon_armed_text == "true"
            self.commissioning_permit_spent = (
                permit_spent_text == "true"
            )
            self.commissioning_motion_active = (
                motion_active_text == "true"
            )
            if issue is None and self.commissioning_motion_active:
                self.commissioning_motion_active_observed = True
            self.commissioning_stop_reason = values.get(
                "commissioning_stop_reason", "missing"
            )
            self.diagnostic_issue = issue or ""
            self.last_diagnostic_evidence = diagnostic_evidence(
                status.message,
                level,
                self.diagnostic_issue,
                values,
            )
            if issue is None:
                self.diagnostic_receipt_s = time.monotonic()

        def publish_velocity(self, vx: float) -> None:
            if not math.isfinite(vx) or not 0.0 <= vx <= SPEED_MPS:
                raise CommissioningError(
                    f"probe attempted velocity outside 0..{SPEED_MPS:.2f} m/s"
                )
            message = Twist()
            message.linear.x = vx
            self.command_publisher.publish(message)

        def graph_issue(self) -> str:
            publishers = self.get_publishers_info_by_topic(COMMAND_TOPIC)
            subscriptions = self.get_subscriptions_info_by_topic(COMMAND_TOPIC)
            canonical_publishers = self.get_publishers_info_by_topic(
                CANONICAL_COMMAND_TOPIC
            )
            sport_publishers = self.get_publishers_info_by_topic(
                SPORT_REQUEST_TOPIC
            )
            lease_publishers = self.get_publishers_info_by_topic(
                SPORT_LEASE_REQUEST_TOPIC
            )
            odom_publishers = self.get_publishers_info_by_topic(ODOM_TOPIC)
            expected_publisher = [
                endpoint
                for endpoint in publishers
                if endpoint.node_name == PROBE_NODE_NAME
                and endpoint.node_namespace == "/"
            ]
            expected_subscription = [
                endpoint
                for endpoint in subscriptions
                if endpoint.node_name == ADAPTER_NODE_NAME
                and endpoint.node_namespace == ADAPTER_NAMESPACE
            ]
            if len(publishers) != 1 or len(expected_publisher) != 1:
                return "dedicated command topic does not have this probe as sole publisher"
            if len(subscriptions) != 1 or len(expected_subscription) != 1:
                return "dedicated command topic does not have the adapter as sole subscriber"
            if canonical_publishers:
                return "canonical /cmd_vel still has a publisher; Nav2 must be motion-isolated"
            expected_odom_publisher = [
                endpoint
                for endpoint in odom_publishers
                if endpoint.node_name == ADAPTER_NODE_NAME
                and endpoint.node_namespace == ADAPTER_NAMESPACE
            ]
            if len(odom_publishers) != 1 or len(expected_odom_publisher) != 1:
                return "canonical /odom does not have the adapter as sole publisher"
            issue, sport_gid = bind_added_sport_writer(
                sport_request_baseline_gids,
                [endpoint.endpoint_gid for endpoint in sport_publishers],
                self.sport_request_gid,
            )
            if issue:
                return issue
            self.sport_request_gid = sport_gid
            if lease_publishers:
                return "a positive-lease request writer is present"
            adapter_nodes = [
                item
                for item in self.get_node_names_and_namespaces()
                if item == (ADAPTER_NODE_NAME, ADAPTER_NAMESPACE)
            ]
            if len(adapter_nodes) != 1:
                return "expected exactly one go2_chassis_adapter node"
            try:
                services = self.get_service_names_and_types_by_node(
                    ADAPTER_NODE_NAME, ADAPTER_NAMESPACE
                )
            except RuntimeError as exc:
                return f"could not inspect adapter services: {exc}"
            matches = [
                types
                for name, types in services
                if name == ARM_SERVICE and "std_srvs/srv/SetBool" in types
            ]
            if len(matches) != 1:
                return "adapter has no unique correctly typed arm service"
            return ""

        def health_issue(
            self,
            allowed_guard_states: Sequence[str],
            *,
            expected_permit_spent: bool | None = None,
            expected_motion_active: bool | None = None,
        ) -> str:
            now_s = time.monotonic()
            if self.latched_issue:
                return self.latched_issue
            for issue in (self.state_issue, self.odom_issue, self.diagnostic_issue):
                if issue:
                    return issue
            try:
                assert_fresh(now_s, self.state_receipt_s, STATE_STALE_S, "state")
                assert_fresh(now_s, self.odom_receipt_s, ODOM_STALE_S, "odometry")
                assert_fresh(
                    now_s,
                    self.diagnostic_receipt_s,
                    DIAGNOSTIC_STALE_S,
                    "chassis diagnostics",
                )
            except CommissioningError as exc:
                return str(exc)
            if self.guard_state not in allowed_guard_states:
                return f"unexpected chassis guard state {self.guard_state}"
            if (
                expected_permit_spent is not None
                and self.commissioning_permit_spent != expected_permit_spent
            ):
                return "adapter commissioning permit state does not match phase"
            if (
                expected_motion_active is not None
                and self.commissioning_motion_active != expected_motion_active
            ):
                return "adapter commissioning motion state does not match phase"
            return ""

        def reset_runtime_graph_check(self) -> None:
            self.next_runtime_graph_check_s = 0.0

        def runtime_issue(
            self,
            allowed_guard_states: Sequence[str],
            *,
            expected_permit_spent: bool | None = None,
            expected_motion_active: bool | None = None,
            force_graph: bool = False,
        ) -> str:
            """Check telemetry every cycle and graph ownership at 2 Hz."""

            now_s = time.monotonic()
            if force_graph or now_s >= self.next_runtime_graph_check_s:
                self.next_runtime_graph_check_s = (
                    now_s + RUNTIME_GRAPH_CHECK_PERIOD_S
                )
                issue = self.graph_issue()
                if issue:
                    return issue
            return self.health_issue(
                allowed_guard_states,
                expected_permit_spent=expected_permit_spent,
                expected_motion_active=expected_motion_active,
            )

        def call_arm(self, arm: bool, timeout_s: float = 1.5) -> tuple[bool, str]:
            if not self.arm_client.wait_for_service(timeout_sec=0.0):
                return False, "arm service is unavailable"
            request = SetBool.Request()
            request.data = arm
            future = self.arm_client.call_async(request)
            deadline = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < deadline:
                self.publish_velocity(0.0)
                rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD_S)
            if not future.done():
                return False, "arm service response timed out"
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001
                return False, f"arm service failed: {type(exc).__name__}"
            if response is None:
                return False, "arm service returned no response"
            return bool(response.success), str(response.message)

        def zero_and_disarm(self) -> tuple[bool, str]:
            try:
                self.publish_velocity(0.0)
            except Exception:  # noqa: BLE001
                pass
            try:
                success, message = self.call_arm(False)
            except Exception as exc:  # noqa: BLE001
                success = False
                message = (
                    "disarm service path raised "
                    f"{type(exc).__name__}: {exc}"
                )
            deadline = time.monotonic() + STOP_ZERO_HOLD_S
            while rclpy.ok() and time.monotonic() < deadline:
                try:
                    self.publish_velocity(0.0)
                    rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD_S)
                except Exception:  # noqa: BLE001
                    break
            return success, message

        def verify_post_stop(self) -> tuple[bool, str, dict[str, object]]:
            if (
                self.odom_xy is None
                or self.odom_linear_speed_mps is None
                or self.odom_yaw_rate_rps is None
            ):
                return False, "post-stop odometry is unavailable", {}
            start_xy = self.odom_xy
            start_sample_count = self.odom_sample_count
            self.reset_runtime_graph_check()
            deadline = time.monotonic() + POST_STOP_OBSERVATION_S
            max_drift_m = 0.0
            max_linear_mps = 0.0
            max_yaw_rps = 0.0
            while rclpy.ok() and time.monotonic() < deadline:
                self.publish_velocity(0.0)
                rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD_S)
                issue = self.runtime_issue(
                    ("DISARMED",),
                    expected_permit_spent=True,
                    expected_motion_active=False,
                )
                if issue:
                    return False, issue, {}
                if self.daemon_armed:
                    return False, "SDK daemon re-armed after explicit stop", {}
                if (
                    self.odom_xy is None
                    or self.odom_linear_speed_mps is None
                    or self.odom_yaw_rate_rps is None
                ):
                    return False, "post-stop odometry disappeared", {}
                decision = post_stop_decision(
                    start_xy,
                    self.odom_xy,
                    self.odom_linear_speed_mps,
                    self.odom_yaw_rate_rps,
                )
                max_drift_m = max(max_drift_m, decision.drift_m)
                max_linear_mps = max(
                    max_linear_mps, decision.linear_speed_mps
                )
                max_yaw_rps = max(max_yaw_rps, decision.yaw_rate_rps)
                if not decision.stationary:
                    return False, decision.reason, {
                        "max_drift_m": max_drift_m,
                        "max_linear_speed_mps": max_linear_mps,
                        "max_yaw_rate_rps": max_yaw_rps,
                    }
            samples = self.odom_sample_count - start_sample_count
            metrics: dict[str, object] = {
                "observation_s": POST_STOP_OBSERVATION_S,
                "odom_samples": samples,
                "max_drift_m": round(max_drift_m, 6),
                "max_linear_speed_mps": round(max_linear_mps, 6),
                "max_yaw_rate_rps": round(max_yaw_rps, 6),
                "daemon_armed": self.daemon_armed,
                "guard_state": self.guard_state,
                "commissioning_motion_active": self.commissioning_motion_active,
                "commissioning_stop_reason": self.commissioning_stop_reason,
            }
            if samples < POST_STOP_MIN_ODOM_SAMPLES:
                return False, "too few fresh post-stop odometry samples", metrics
            return True, "continuous stationary observation passed", metrics

    node: FirstMotionNode | None = None
    evidence: dict[str, object] = {
        "schema": "robonix-go2-first-motion-v1",
        "command_topic": COMMAND_TOPIC,
        "speed_limit_mps": SPEED_MPS,
        "distance_limit_m": DISTANCE_LIMIT_M,
        "duration_limit_s": DURATION_LIMIT_S,
        "probe_stop_distance_m": PROBE_STOP_DISTANCE_M,
        "probe_stop_duration_s": PROBE_STOP_DURATION_S,
        "minimum_measured_displacement_m": MIN_MEASURED_DISPLACEMENT_M,
        "maximum_measured_lateral_m": MAX_MEASURED_LATERAL_M,
        "maximum_measured_yaw_change_rad": MAX_MEASURED_YAW_CHANGE_RAD,
        "required_odom_source": "external_verified",
        "required_external_odom_topic": EXTERNAL_ODOM_TOPIC,
        "sport_request_baseline_writer_count": len(
            sport_request_baseline_gids
        ),
        "sport_request_baseline_writer_gids": sorted(
            gid.hex() for gid in sport_request_baseline_gids
        ),
        "status": "FAIL",
        "started_unix_ns": time.time_ns(),
    }
    failure: str | None = None
    arm_accepted = False
    disarm_success = False
    disarm_message = "disarm was not attempted"
    post_stop_success = False
    post_stop_message = "post-stop verification was not attempted"
    post_stop_metrics: dict[str, object] = {}
    sport_state_start: dict[str, object] | None = None
    sport_state_end: dict[str, object] | None = None
    exit_code = 1
    try:
        node = FirstMotionNode()
        print(
            "FIRST-MOTION PROBE: max 0.05 m/s; soft stop 0.09 m / 1.8 s; "
            "hard envelope 0.10 m / 2.0 s"
        )
        print("Official remote stop must remain in the operator's hand.")

        # Acquire fresh telemetry and a stable, isolated ROS command graph.
        deadline = time.monotonic() + 8.0
        stable = 0
        next_graph_check = 0.0
        while time.monotonic() < deadline:
            if stop_requested:
                raise CommissioningError("operator interrupt before arm")
            node.publish_velocity(0.0)
            rclpy.spin_once(node, timeout_sec=CONTROL_PERIOD_S)
            now_s = time.monotonic()
            if now_s >= next_graph_check:
                issue = node.graph_issue() or node.health_issue(
                    ("DISARMED",),
                    expected_permit_spent=False,
                    expected_motion_active=False,
                )
                stable = stable + 1 if not issue else 0
                next_graph_check = now_s + GRAPH_SAMPLE_PERIOD_S
                if stable >= GRAPH_STABILITY_SAMPLES:
                    break
        if stable < GRAPH_STABILITY_SAMPLES:
            issue = node.graph_issue() or node.health_issue(
                ("DISARMED",),
                expected_permit_spent=False,
                expected_motion_active=False,
            )
            raise CommissioningError(f"pre-arm graph/telemetry gate failed: {issue}")
        if node.sport_request_gid is None:
            raise CommissioningError("sport request writer DDS GID was not bound")
        evidence["command_ownership"] = "PASS"
        evidence["sport_request_writer_gid"] = node.sport_request_gid.hex()
        evidence["positive_lease_request_publishers"] = 0

        # Continuous zero is required both before the arm request and while the
        # adapter runs its own independent preparing state.
        node.reset_runtime_graph_check()
        zero_deadline = time.monotonic() + ZERO_PREAMBLE_S
        while time.monotonic() < zero_deadline:
            if stop_requested:
                raise CommissioningError("operator interrupt during zero preamble")
            issue = node.runtime_issue(
                ("DISARMED",),
                expected_permit_spent=False,
                expected_motion_active=False,
            )
            if issue:
                raise CommissioningError(issue)
            node.publish_velocity(0.0)
            rclpy.spin_once(node, timeout_sec=CONTROL_PERIOD_S)

        arm_success, arm_message = node.call_arm(True)
        evidence["arm_message"] = arm_message
        if not arm_success:
            raise CommissioningError(f"adapter rejected arm: {arm_message}")
        arm_accepted = True

        armed_deadline = time.monotonic() + ARMED_ZERO_TIMEOUT_S
        while time.monotonic() < armed_deadline:
            if stop_requested:
                raise CommissioningError("operator interrupt while awaiting armed zero")
            # The cached 5 Hz diagnostic can still say DISARMED briefly after
            # the arm service accepted the transition to PREPARING.  Continue
            # publishing zero while waiting; the deadline below still requires
            # a measured ARMED + daemon_armed state before any non-zero Twist.
            issue = node.runtime_issue(
                ("DISARMED", "PREPARING", "ARMED")
            )
            if issue:
                raise CommissioningError(issue)
            node.publish_velocity(0.0)
            rclpy.spin_once(node, timeout_sec=CONTROL_PERIOD_S)
            if node.guard_state == "ARMED" and node.daemon_armed:
                break
        if node.guard_state != "ARMED" or not node.daemon_armed:
            raise CommissioningError("adapter/SDK daemon did not reach armed zero")
        issue = node.runtime_issue(
            ("ARMED",),
            expected_permit_spent=True,
            expected_motion_active=False,
            force_graph=True,
        )
        if issue:
            raise CommissioningError(issue)
        if node.odom_xy is None or node.odom_yaw_rad is None:
            raise CommissioningError("odometry disappeared before motion")

        start_xy = node.odom_xy
        start_yaw_rad = node.odom_yaw_rad
        if node.sport_state_evidence is not None:
            sport_state_start = dict(node.sport_state_evidence)
        start_s = time.monotonic()
        final_decision = MotionDecision(True, "not_started", 0.0, 0.0)
        while True:
            if stop_requested:
                raise CommissioningError("operator interrupt during motion")
            issue = node.runtime_issue(("ARMED",))
            if issue:
                raise CommissioningError(issue)
            if not node.daemon_armed:
                raise CommissioningError("SDK daemon disarmed during motion")
            if node.odom_xy is None or node.odom_yaw_rad is None:
                raise CommissioningError("odometry disappeared during motion")
            final_decision = motion_decision(
                start_s, time.monotonic(), start_xy, node.odom_xy
            )
            if not final_decision.continue_motion:
                break
            node.publish_velocity(SPEED_MPS)
            rclpy.spin_once(node, timeout_sec=CONTROL_PERIOD_S)

        node.publish_velocity(0.0)
        if node.sport_state_evidence is not None:
            sport_state_end = dict(node.sport_state_evidence)
        if node.odom_xy is None or node.odom_yaw_rad is None:
            raise CommissioningError("odometry disappeared at motion stop")
        measurement = measured_motion(
            start_xy,
            start_yaw_rad,
            node.odom_xy,
            node.odom_yaw_rad,
        )
        evidence.update(
            {
                "stop_reason": final_decision.reason,
                "commanded_duration_s": round(final_decision.elapsed_s, 6),
                "measured_odom_distance_m": round(final_decision.distance_m, 6),
                "measured_odom_forward_m": round(measurement.forward_m, 6),
                "measured_odom_lateral_m": round(measurement.lateral_m, 6),
                "measured_odom_yaw_change_rad": round(
                    measurement.yaw_change_rad, 6
                ),
                "commissioning_motion_active_observed": (
                    node.commissioning_motion_active_observed
                ),
            }
        )
        if not node.commissioning_motion_active_observed:
            raise CommissioningError(
                "adapter diagnostics never confirmed commissioning motion active"
            )
        require_measured_displacement(measurement)
        exit_code = 0
    except Exception as exc:  # noqa: BLE001
        failure = f"{type(exc).__name__}: {exc}"
        evidence["failure"] = failure
        if node is not None:
            evidence["last_chassis_diagnostic"] = (
                node.last_diagnostic_evidence
            )
        print(f"FAIL-CLOSED: {failure}", file=sys.stderr)
    finally:
        if node is not None and rclpy.ok():
            if (
                sport_state_end is None
                and node.sport_state_evidence is not None
            ):
                sport_state_end = dict(node.sport_state_evidence)
            disarm_success, disarm_message = node.zero_and_disarm()
            if arm_accepted and disarm_success:
                try:
                    (
                        post_stop_success,
                        post_stop_message,
                        post_stop_metrics,
                    ) = node.verify_post_stop()
                except Exception as exc:  # noqa: BLE001
                    post_stop_message = (
                        f"post-stop verification raised {type(exc).__name__}: {exc}"
                    )
            evidence["sport_mode_state"] = sport_state_pair_evidence(
                sport_state_start,
                sport_state_end,
                node.sport_state_evidence_issue,
            )
            node.destroy_node()
        else:
            evidence["sport_mode_state"] = sport_state_pair_evidence(
                sport_state_start,
                sport_state_end,
                "SportModeState evidence node unavailable",
            )
        evidence["disarm_success"] = disarm_success
        evidence["disarm_message"] = disarm_message
        evidence["post_stop_success"] = post_stop_success
        evidence["post_stop_message"] = post_stop_message
        evidence["post_stop"] = post_stop_metrics
        if exit_code == 0 and disarm_success and post_stop_success:
            evidence["status"] = "PASS"
        else:
            exit_code = 1
            if failure is None and not disarm_success:
                evidence["failure"] = "explicit adapter disarm was not acknowledged"
            elif failure is None and not post_stop_success:
                evidence["failure"] = post_stop_message
        evidence["finished_unix_ns"] = time.time_ns()
        try:
            evidence_path = _write_evidence(evidence)
            print(f"Evidence: {evidence_path}")
        except OSError as exc:
            print(f"could not write evidence: {exc}", file=sys.stderr)
            exit_code = 1
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
    return exit_code


def main() -> int:
    if len(sys.argv) != 1:
        print(f"usage: {Path(sys.argv[0]).name}", file=sys.stderr)
        return 2
    try:
        allowed_modes, allowed_markers = validate_environment(os.environ)
        sport_request_baseline_gids = load_sport_request_baseline(os.environ)
    except CommissioningError as exc:
        print(f"refusing first motion: {exc}", file=sys.stderr)
        return 2
    return _run_ros(
        allowed_modes, allowed_markers, sport_request_baseline_gids
    )


if __name__ == "__main__":
    raise SystemExit(main())
