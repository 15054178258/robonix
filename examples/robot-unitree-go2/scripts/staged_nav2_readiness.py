#!/usr/bin/env python3
"""Subscription-only readiness receipt for staged physical Nav2.

The check has two explicit phases:

``pre_guard``
    Nav2 is up, the chassis adapter is configured but disarmed, and no process
    publishes the private staged chassis topic yet.

``post_guard``
    The staged motion guard owns the private staged publisher and is the only
    subscriber to the velocity smoother output.  The chassis is still
    configured but disarmed.

This program deliberately does not use :class:`rclpy.node.Node`.  On ROS 2
Humble that class creates a ``/parameter_events`` publisher even for an
otherwise subscription-only program.  The live collector below owns only a
low-level node handle, subscriptions, and a wait set.  It never constructs a
publisher, service, service client, action server, action client, timer, or
Unitree SDK object.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import stat
import time
from typing import Any, Callable, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = "robonix-go2-staged-nav2-readiness-v1"
PHASES = ("pre_guard", "post_guard")

PROFILE = "workstation-staged-nav2-corrected-v1"
EXPECTED_CONTROLLER = "/velocity_smoother"
EXPECTED_GUARD = "/go2_staged_nav2_motion_guard"
EXPECTED_CHASSIS = "/go2_chassis_adapter"

GUARD_INPUT_TOPIC = "/cmd_vel_guard_input"
STAGED_OUTPUT_TOPIC = "/go2/staged_nav2/cmd_vel"
CANONICAL_OUTPUT_TOPIC = "/cmd_vel"
BEHAVIOR_SINK_TOPIC = "/robonix/staged_nav2/behavior_cmd_vel_forbidden"

MAP_TOPIC = "/map"
LIFECYCLE_TOPIC = "/robonix/map/lifecycle"
LOCALIZATION_TOPIC = "/robonix/map/pose"
ODOM_TOPIC = "/odom"
SCAN_TOPIC = "/scanner/scan"
STATE_TOPIC = "/robonix/time_corrected/motion/sportmodestate"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
GOAL_STATUS_TOPIC = "/navigate_to_pose/_action/status"
GOAL_FEEDBACK_TOPIC = "/navigate_to_pose/_action/feedback"
CHASSIS_STATUS_TOPIC = "/go2_chassis/status"

REQUIRED_LIFECYCLE_NODES = (
    "/controller_server",
    "/planner_server",
    "/smoother_server",
    "/behavior_server",
    "/bt_navigator",
    "/waypoint_follower",
    "/velocity_smoother",
)

TOPOLOGY_TOPICS = (
    GUARD_INPUT_TOPIC,
    STAGED_OUTPUT_TOPIC,
    CANONICAL_OUTPUT_TOPIC,
    BEHAVIOR_SINK_TOPIC,
)

RECEIPT_FRESHNESS_NS = {
    # This collector performs graph inspection after sampling.  These are
    # startup-observation windows, not motion watchdogs; the motion guard and
    # chassis keep their independent 200/250 ms live limits.
    "map": 2_000_000_000,
    "map_lifecycle": 2_000_000_000,
    "localization": 1_000_000_000,
    "odom": 1_000_000_000,
    # Projected scans arrive in wireless bursts on the private Go2 LAN. Keep
    # source-stamp freshness at one second, but do not reject a still-current
    # scan solely because local delivery jitter exceeds one second.
    "scan": 1_500_000_000,
    "state": 1_000_000_000,
    "tf": 1_000_000_000,
    "goal_status": 1_000_000_000,
    "chassis": 2_000_000_000,
}
SOURCE_FRESHNESS_NS = {
    # Snapshot evaluation happens after graph inspection.  Runtime source-age
    # enforcement remains in the motion guard and chassis at 200/250 ms.
    "localization": 1_000_000_000,
    "odom": 1_000_000_000,
    "scan": 1_000_000_000,
    "state": 1_000_000_000,
    "tf": 1_000_000_000,
}
MAX_SOURCE_FUTURE_NS = 50_000_000
MAX_START_DISTANCE_M = 0.05
ALLOWED_OUTPUT_ROOTS = (ROOT / "logs", ROOT / "rbnx-build")
SESSION_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{7,63}$")
MAP_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{7,63}$")
INTERFACE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,14}$")


class ReadinessError(RuntimeError):
    """The bounded audit could not produce a passing readiness receipt."""


def _bounded_seconds(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("duration must be a number") from error
    if not math.isfinite(parsed) or not 1.0 <= parsed <= 30.0:
        raise argparse.ArgumentTypeError("duration must be in 1..30 seconds")
    return parsed


def _positive_generation(value: str) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "map generation must be a positive integer"
        ) from error
    if parsed <= 0:
        raise argparse.ArgumentTypeError(
            "map generation must be a positive integer"
        )
    return parsed


def _bounded_uint32(label: str) -> Callable[[str], int]:
    def parse(value: str) -> int:
        try:
            parsed = int(value, 10)
        except ValueError as error:
            raise argparse.ArgumentTypeError(
                f"{label} must be an unsigned integer"
            ) from error
        if not 0 <= parsed <= 0xFFFFFFFF:
            raise argparse.ArgumentTypeError(
                f"{label} must be in 0..4294967295"
            )
        return parsed

    return parse


def _bounded_mode(value: str) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "allowed mode must be an unsigned integer"
        ) from error
    if not 0 <= parsed <= 254:
        raise argparse.ArgumentTypeError("allowed mode must be in 0..254")
    return parsed


def _finite_float(label: str) -> Callable[[str], float]:
    def parse(value: str) -> float:
        try:
            parsed = float(value)
        except ValueError as error:
            raise argparse.ArgumentTypeError(f"{label} must be finite") from error
        if not math.isfinite(parsed):
            raise argparse.ArgumentTypeError(f"{label} must be finite")
        return parsed

    return parse


def _safe_value(
    pattern: re.Pattern[str], label: str
) -> Callable[[str], str]:
    def parse(value: str) -> str:
        if pattern.fullmatch(value) is None:
            raise argparse.ArgumentTypeError(f"{label} has invalid format")
        return value

    return parse


def _absolute_path(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("output must be an absolute path")
    return path


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "READ-ONLY staged Nav2 readiness audit. Creates subscriptions and "
            "reads the ROS graph; never creates a command endpoint or client."
        )
    )
    parser.add_argument("--phase", required=True, choices=PHASES)
    parser.add_argument(
        "--session-id",
        required=True,
        type=_safe_value(SESSION_RE, "session id"),
    )
    parser.add_argument(
        "--network-interface",
        required=True,
        type=_safe_value(INTERFACE_RE, "network interface"),
    )
    parser.add_argument(
        "--map-id",
        required=True,
        type=_safe_value(MAP_ID_RE, "map id"),
    )
    parser.add_argument(
        "--map-generation", required=True, type=_positive_generation
    )
    parser.add_argument(
        "--allowed-mode",
        required=True,
        type=_bounded_mode,
    )
    parser.add_argument(
        "--allowed-state-marker",
        required=True,
        type=_bounded_uint32("allowed state marker"),
    )
    parser.add_argument(
        "--expected-start-x",
        type=_finite_float("expected start x"),
    )
    parser.add_argument(
        "--expected-start-y",
        type=_finite_float("expected start y"),
    )
    parser.add_argument(
        "--duration", type=_bounded_seconds, default=5.0
    )
    parser.add_argument(
        "--output",
        "--receipt",
        dest="output",
        required=True,
        type=_absolute_path,
    )
    return parser


def _output_path(path: Path) -> Path:
    """Return a new private receipt path below repository evidence roots."""

    absolute = Path(os.path.abspath(os.fspath(path)))
    try:
        parent = absolute.parent.resolve(strict=True)
        parent_info = os.lstat(parent)
    except OSError as error:
        raise ReadinessError(f"cannot inspect output parent: {error}") from error
    if absolute.parent != parent:
        raise ReadinessError("output parent must not contain symlinks")
    if not stat.S_ISDIR(parent_info.st_mode):
        raise ReadinessError("output parent must be a directory")
    if parent_info.st_uid != os.geteuid():
        raise ReadinessError("output parent must be owned by the current user")
    if stat.S_IMODE(parent_info.st_mode) & 0o077:
        raise ReadinessError(
            "output parent must not be accessible by group or others"
        )
    roots: list[Path] = []
    for root in ALLOWED_OUTPUT_ROOTS:
        try:
            roots.append(root.resolve(strict=True))
        except OSError:
            continue
    candidate = parent / absolute.name
    if not any(
        candidate != root and candidate.is_relative_to(root) for root in roots
    ):
        raise ReadinessError(
            "output must be below repository logs/ or rbnx-build/"
        )
    if os.path.lexists(candidate):
        raise ReadinessError(f"refusing to overwrite output: {candidate}")
    return candidate


def _write_exclusive_private(path: Path, payload: Mapping[str, Any]) -> None:
    encoded = (
        json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(path, flags, 0o600)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        path.unlink(missing_ok=True)
        raise
    finally:
        os.close(descriptor)


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _all_finite(values: Sequence[Any]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def _quaternion_valid(quaternion: Any) -> bool:
    values = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    if not _all_finite(values):
        return False
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    return 0.5 <= norm <= 1.5


def _yaw_from_quaternion(quaternion: Any) -> float:
    norm = math.sqrt(
        float(quaternion.x) ** 2
        + float(quaternion.y) ** 2
        + float(quaternion.z) ** 2
        + float(quaternion.w) ** 2
    )
    if not math.isfinite(norm) or norm <= 0.0:
        return math.nan
    x = float(quaternion.x) / norm
    y = float(quaternion.y) / norm
    z = float(quaternion.z) / norm
    w = float(quaternion.w) / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _sample_base(received_monotonic_ns: int) -> dict[str, Any]:
    return {"received_monotonic_ns": int(received_monotonic_ns)}


def _map_sample(message: Any, received_ns: int) -> dict[str, Any]:
    width = int(message.info.width)
    height = int(message.info.height)
    resolution = float(message.info.resolution)
    origin = message.info.origin
    result = _sample_base(received_ns)
    result.update(
        {
            "width": width,
            "height": height,
            "resolution": resolution,
            "data_length": len(message.data),
            "frame_id": str(message.header.frame_id),
            "valid": (
                width > 0
                and height > 0
                and math.isfinite(resolution)
                and resolution > 0.0
                and len(message.data) == width * height
                and str(message.header.frame_id) == "map"
                and _all_finite(
                    (
                        origin.position.x,
                        origin.position.y,
                        origin.position.z,
                    )
                )
                and _quaternion_valid(origin.orientation)
            ),
        }
    )
    return result


def _lifecycle_sample(message: Any, received_ns: int) -> dict[str, Any]:
    result = _sample_base(received_ns)
    result.update(
        {
            "map_id": str(message.map_id),
            "mode": str(message.mode),
            "generation": int(message.generation),
        }
    )
    return result


def _pose_sample(message: Any, received_ns: int) -> dict[str, Any]:
    pose = message.pose.pose
    yaw = _yaw_from_quaternion(pose.orientation)
    result = _sample_base(received_ns)
    result.update(
        {
            "source_stamp_ns": _stamp_ns(message.header.stamp),
            "frame_id": str(message.header.frame_id),
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_rad": yaw,
            "valid": (
                str(message.header.frame_id) == "map"
                and _all_finite(
                    (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        yaw,
                        *message.pose.covariance,
                    )
                )
                and _quaternion_valid(pose.orientation)
            ),
        }
    )
    return result


def _odom_sample(message: Any, received_ns: int) -> dict[str, Any]:
    pose = message.pose.pose
    twist = message.twist.twist
    result = _sample_base(received_ns)
    result.update(
        {
            "source_stamp_ns": _stamp_ns(message.header.stamp),
            "frame_id": str(message.header.frame_id),
            "child_frame_id": str(message.child_frame_id),
            "valid": (
                str(message.header.frame_id) == "odom"
                and str(message.child_frame_id) == "base_link"
                and _all_finite(
                    (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        twist.linear.x,
                        twist.linear.y,
                        twist.linear.z,
                        twist.angular.x,
                        twist.angular.y,
                        twist.angular.z,
                        *message.pose.covariance,
                        *message.twist.covariance,
                    )
                )
                and _quaternion_valid(pose.orientation)
            ),
        }
    )
    return result


def _scan_sample(message: Any, received_ns: int) -> dict[str, Any]:
    range_min = float(message.range_min)
    range_max = float(message.range_max)
    finite_ranges = [
        float(value)
        for value in message.ranges
        if math.isfinite(float(value))
    ]
    valid_ranges = [
        value for value in finite_ranges if range_min <= value <= range_max
    ]
    result = _sample_base(received_ns)
    result.update(
        {
            "source_stamp_ns": _stamp_ns(message.header.stamp),
            "frame_id": str(message.header.frame_id),
            "range_count": len(message.ranges),
            "valid_range_count": len(valid_ranges),
            "valid": (
                bool(str(message.header.frame_id))
                and _all_finite(
                    (
                        message.angle_min,
                        message.angle_max,
                        message.angle_increment,
                        range_min,
                        range_max,
                    )
                )
                and float(message.angle_increment) > 0.0
                and float(message.angle_max) > float(message.angle_min)
                and 0.0 <= range_min < range_max
                and len(message.ranges) > 0
                and len(valid_ranges) > 0
            ),
        }
    )
    return result


def _state_sample(message: Any, received_ns: int) -> dict[str, Any]:
    velocity = tuple(float(value) for value in message.velocity)
    result = _sample_base(received_ns)
    result.update(
        {
            "source_stamp_ns": _stamp_ns(message.stamp),
            "error_code": int(message.error_code),
            "mode": int(message.mode),
            "gait_type": int(message.gait_type),
            "valid": _all_finite((*velocity, message.yaw_speed)),
        }
    )
    return result


def _tf_samples(message: Any, received_ns: int) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for transform in message.transforms:
        parent = str(transform.header.frame_id).lstrip("/")
        child = str(transform.child_frame_id).lstrip("/")
        edge = f"{parent}->{child}"
        if edge not in {"map->odom", "odom->base_link"}:
            continue
        translation = transform.transform.translation
        result[edge] = {
            "received_monotonic_ns": int(received_ns),
            "source_stamp_ns": _stamp_ns(transform.header.stamp),
            "valid": (
                _all_finite((translation.x, translation.y, translation.z))
                and _quaternion_valid(transform.transform.rotation)
            ),
        }
    return result


def _goal_status_sample(message: Any, received_ns: int) -> dict[str, Any]:
    statuses = [int(item.status) for item in message.status_list]
    result = _sample_base(received_ns)
    result["statuses"] = statuses
    return result


def _chassis_sample(message: Any, received_ns: int) -> dict[str, Any]:
    text = str(message.data)
    state, separator, remainder = text.partition(":")
    values: dict[str, str] = {}
    if separator:
        for item in remainder.split(";")[1:]:
            key, equals, value = item.strip().partition("=")
            if equals and key and key not in values:
                values[key] = value.strip()
    result = _sample_base(received_ns)
    result.update(
        {
            "state": state.strip().upper(),
            "daemon_armed": values.get("daemon_armed"),
            "motion_configured": values.get("motion_configured"),
            "motion_profile": values.get("motion_profile"),
            "odom_source": values.get("odom_source"),
            "status_excerpt": text[:300],
        }
    )
    return result


def _canonical_endpoint(node_name: str, namespace: str) -> str:
    parts = [
        part.strip("/")
        for part in (str(namespace), str(node_name))
        if part.strip("/")
    ]
    return "/" + "/".join(parts)


def _node_parts(full_name: str) -> tuple[str, str]:
    stripped = full_name.strip("/")
    node_name = stripped.rsplit("/", 1)[-1]
    namespace_tail = stripped[: -len(node_name)].strip("/")
    namespace = "/" + namespace_tail if namespace_tail else "/"
    return node_name, namespace


def _check(
    checks: list[dict[str, Any]],
    check_id: str,
    passed: bool,
    detail: str,
    observed: Any = None,
) -> None:
    item: dict[str, Any] = {
        "id": check_id,
        "passed": bool(passed),
        "detail": detail,
    }
    if observed is not None:
        item["observed"] = observed
    checks.append(item)


def _fresh(
    sample: object,
    *,
    key: str,
    now_monotonic_ns: int,
    now_realtime_ns: int,
    source_required: bool,
) -> tuple[bool, str]:
    if not isinstance(sample, Mapping):
        return False, "sample missing"
    try:
        received_ns = int(sample["received_monotonic_ns"])
    except (KeyError, TypeError, ValueError):
        return False, "receipt timestamp missing"
    receipt_age = now_monotonic_ns - received_ns
    if receipt_age < 0:
        return False, "receipt timestamp is in the future"
    if receipt_age > RECEIPT_FRESHNESS_NS[key]:
        return False, f"receipt age {receipt_age} ns exceeds limit"
    if source_required:
        try:
            source_ns = int(sample["source_stamp_ns"])
        except (KeyError, TypeError, ValueError):
            return False, "source timestamp missing"
        source_age = now_realtime_ns - source_ns
        if source_ns <= 0:
            return False, "source timestamp is not positive"
        if source_age < -MAX_SOURCE_FUTURE_NS:
            return False, "source timestamp is too far in the future"
        if source_age > SOURCE_FRESHNESS_NS[key]:
            return False, f"source age {source_age} ns exceeds limit"
    return True, "fresh"


def _endpoint_names(
    graph: Mapping[str, Any], direction: str, topic: str
) -> tuple[str, ...]:
    value = graph.get(direction, {})
    if not isinstance(value, Mapping):
        return ()
    names = value.get(topic, ())
    if not isinstance(names, Sequence) or isinstance(names, (str, bytes)):
        return ()
    return tuple(sorted(str(name) for name in names))


def _types(
    graph: Mapping[str, Any], collection: str, name: str
) -> tuple[str, ...]:
    value = graph.get(collection, {})
    if not isinstance(value, Mapping):
        return ()
    names = value.get(name, ())
    if not isinstance(names, Sequence) or isinstance(names, (str, bytes)):
        return ()
    return tuple(sorted(str(item) for item in names))


def evaluate_snapshot(
    snapshot: Mapping[str, Any],
    *,
    phase: str,
    session_id: str,
    network_interface: str,
    expected_map_id: str,
    expected_generation: int,
    allowed_mode: int,
    allowed_state_marker: int,
    expected_start_x: float | None,
    expected_start_y: float | None,
    now_monotonic_ns: int,
    now_realtime_ns: int,
) -> dict[str, Any]:
    """Purely evaluate one bounded sample/graph snapshot."""

    if phase not in PHASES:
        raise ReadinessError(f"unsupported phase: {phase}")
    if SESSION_RE.fullmatch(session_id) is None:
        raise ReadinessError("session id has invalid format")
    if INTERFACE_RE.fullmatch(network_interface) is None:
        raise ReadinessError("network interface has invalid format")
    if MAP_ID_RE.fullmatch(expected_map_id) is None or expected_generation <= 0:
        raise ReadinessError("expected map identity must be non-empty and positive")
    if not 0 <= allowed_mode <= 254:
        raise ReadinessError("allowed mode is outside 0..254")
    if not 0 <= allowed_state_marker <= 0xFFFFFFFF:
        raise ReadinessError("allowed state marker is outside uint32")
    if phase == "post_guard":
        if (
            expected_start_x is None
            or expected_start_y is None
            or not _all_finite((expected_start_x, expected_start_y))
        ):
            raise ReadinessError(
                "post_guard requires a finite expected path start"
            )
    elif expected_start_x is not None or expected_start_y is not None:
        raise ReadinessError(
            "pre_guard must not claim a post_guard path-start comparison"
        )
    samples_value = snapshot.get("samples", {})
    graph_value = snapshot.get("graph", {})
    samples = samples_value if isinstance(samples_value, Mapping) else {}
    graph = graph_value if isinstance(graph_value, Mapping) else {}
    checks: list[dict[str, Any]] = []

    map_sample = samples.get("map")
    map_fresh, map_fresh_detail = _fresh(
        map_sample,
        key="map",
        now_monotonic_ns=now_monotonic_ns,
        now_realtime_ns=now_realtime_ns,
        source_required=False,
    )
    map_valid = bool(
        isinstance(map_sample, Mapping) and map_sample.get("valid") is True
    )
    _check(
        checks,
        "map_valid_and_fresh",
        map_valid and map_fresh,
        map_fresh_detail if map_valid else "map payload invalid",
        map_sample,
    )

    lifecycle = samples.get("map_lifecycle")
    lifecycle_fresh, lifecycle_detail = _fresh(
        lifecycle,
        key="map_lifecycle",
        now_monotonic_ns=now_monotonic_ns,
        now_realtime_ns=now_realtime_ns,
        source_required=False,
    )
    lifecycle_exact = bool(
        isinstance(lifecycle, Mapping)
        and lifecycle.get("map_id") == expected_map_id
        and lifecycle.get("mode") == "localization"
        and lifecycle.get("generation") == expected_generation
    )
    _check(
        checks,
        "map_lifecycle_exact",
        lifecycle_fresh and lifecycle_exact,
        lifecycle_detail if lifecycle_exact else "map lifecycle mismatch",
        lifecycle,
    )

    for key in ("localization", "odom", "scan", "state"):
        sample = samples.get(key)
        fresh, detail = _fresh(
            sample,
            key=key,
            now_monotonic_ns=now_monotonic_ns,
            now_realtime_ns=now_realtime_ns,
            source_required=True,
        )
        valid = bool(isinstance(sample, Mapping) and sample.get("valid") is True)
        if key == "state":
            valid = bool(
                valid
                and sample.get("mode") == allowed_mode
                and sample.get("error_code") == allowed_state_marker
            )
        _check(
            checks,
            f"{key}_valid_and_fresh",
            valid and fresh,
            detail
            if valid
            else (
                "state mode/marker does not match permit"
                if key == "state"
                else f"{key} payload invalid"
            ),
            sample,
        )

    if phase == "post_guard":
        localization_for_start = samples.get("localization")
        try:
            start_distance = math.hypot(
                float(localization_for_start["x_m"])
                - float(expected_start_x),
                float(localization_for_start["y_m"])
                - float(expected_start_y),
            )
        except (KeyError, TypeError, ValueError):
            start_distance = math.inf
        _check(
            checks,
            "localization_matches_expected_start",
            math.isfinite(start_distance)
            and start_distance <= MAX_START_DISTANCE_M,
            (
                "live localization is within the path-start radius"
                if start_distance <= MAX_START_DISTANCE_M
                else "live localization is outside the path-start radius"
            ),
            {
                "expected_x_m": expected_start_x,
                "expected_y_m": expected_start_y,
                "distance_m": (
                    start_distance if math.isfinite(start_distance) else None
                ),
                "maximum_distance_m": MAX_START_DISTANCE_M,
            },
        )

    tf_edges_value = samples.get("tf_edges", {})
    tf_edges = (
        tf_edges_value if isinstance(tf_edges_value, Mapping) else {}
    )
    for edge in ("map->odom", "odom->base_link"):
        sample = tf_edges.get(edge)
        fresh, detail = _fresh(
            sample,
            key="tf",
            now_monotonic_ns=now_monotonic_ns,
            now_realtime_ns=now_realtime_ns,
            source_required=True,
        )
        valid = bool(isinstance(sample, Mapping) and sample.get("valid") is True)
        _check(
            checks,
            f"tf_{edge.replace('->', '_to_')}_valid_and_fresh",
            valid and fresh,
            detail if valid else f"{edge} transform invalid",
            sample,
        )

    goal_status = samples.get("goal_status")
    status_fresh, status_detail = _fresh(
        goal_status,
        key="goal_status",
        now_monotonic_ns=now_monotonic_ns,
        now_realtime_ns=now_realtime_ns,
        source_required=False,
    )
    statuses = (
        list(goal_status.get("statuses", ()))
        if isinstance(goal_status, Mapping)
        else []
    )
    no_active_goal = all(status not in {1, 2, 3} for status in statuses)
    # An idle Nav2 action server is allowed to have no status sample.  Requiring
    # a status message before the first goal is a local commissioning artifact,
    # not an official Nav2 readiness condition.
    if goal_status is None:
        status_fresh = True
        status_detail = "no goal status sample; action server is idle"
    _check(
        checks,
        "nav2_goal_status_fresh_and_idle",
        status_fresh and no_active_goal,
        status_detail if no_active_goal else "an action goal is active",
        statuses,
    )

    chassis = samples.get("chassis")
    chassis_fresh, chassis_detail = _fresh(
        chassis,
        key="chassis",
        now_monotonic_ns=now_monotonic_ns,
        now_realtime_ns=now_realtime_ns,
        source_required=False,
    )
    chassis_disarmed = bool(
        isinstance(chassis, Mapping)
        and chassis.get("state") == "DISARMED"
        and chassis.get("daemon_armed") == "false"
        and chassis.get("motion_configured") == "true"
        and chassis.get("motion_profile") == PROFILE
        and chassis.get("odom_source") == "external_verified"
    )
    _check(
        checks,
        "chassis_disarmed",
        chassis_fresh and chassis_disarmed,
        chassis_detail
        if chassis_disarmed
        else (
            "chassis must report DISARMED, daemon_armed=false, "
            "motion_configured=true, the staged profile, and "
            "odom_source=external_verified"
        ),
        chassis,
    )

    for check_id, topic in (
        ("map_unique_publisher", MAP_TOPIC),
        ("map_lifecycle_unique_publisher", LIFECYCLE_TOPIC),
    ):
        observed = _endpoint_names(graph, "publishers", topic)
        _check(
            checks,
            check_id,
            len(observed) == 1,
            "exactly one live publisher must retain the static witness",
            observed,
        )

    for node in REQUIRED_LIFECYCLE_NODES:
        services_by_node = graph.get("services_by_node", {})
        node_services = (
            services_by_node.get(node, {})
            if isinstance(services_by_node, Mapping)
            else {}
        )
        node_services = (
            node_services if isinstance(node_services, Mapping) else {}
        )
        expected = {
            f"{node}/get_state": "lifecycle_msgs/srv/GetState",
            f"{node}/change_state": "lifecycle_msgs/srv/ChangeState",
        }
        missing = {
            service: expected_type
            for service, expected_type in expected.items()
            if expected_type
            not in tuple(str(item) for item in node_services.get(service, ()))
        }
        _check(
            checks,
            f"lifecycle_graph_{node.strip('/')}",
            not missing,
            "lifecycle state/change services discovered"
            if not missing
            else "lifecycle graph endpoints missing or wrong type",
            missing or node_services,
        )

    action_services = {
        "/navigate_to_pose/_action/send_goal":
            "nav2_msgs/action/NavigateToPose_SendGoal",
        "/navigate_to_pose/_action/get_result":
            "nav2_msgs/action/NavigateToPose_GetResult",
        "/navigate_to_pose/_action/cancel_goal":
            "action_msgs/srv/CancelGoal",
    }
    action_topics = {
        GOAL_STATUS_TOPIC: "action_msgs/msg/GoalStatusArray",
        GOAL_FEEDBACK_TOPIC:
            "nav2_msgs/action/NavigateToPose_FeedbackMessage",
    }
    missing_action_services = {
        name: expected_type
        for name, expected_type in action_services.items()
        if expected_type not in _types(graph, "services", name)
    }
    missing_action_topics = {
        name: expected_type
        for name, expected_type in action_topics.items()
        if expected_type not in _types(graph, "topics", name)
    }
    _check(
        checks,
        "navigate_to_pose_action_graph",
        not missing_action_services and not missing_action_topics,
        "NavigateToPose server graph discovered"
        if not missing_action_services and not missing_action_topics
        else "NavigateToPose graph endpoints missing or wrong type",
        {
            "missing_services": missing_action_services,
            "missing_topics": missing_action_topics,
        },
    )

    exact_topology: list[
        tuple[str, str, str, tuple[str, ...]]
    ] = [
        (
            "guard_input_publisher",
            "publishers",
            GUARD_INPUT_TOPIC,
            (EXPECTED_CONTROLLER,),
        ),
        (
            "staged_output_subscriber",
            "subscribers",
            STAGED_OUTPUT_TOPIC,
            (EXPECTED_CHASSIS,),
        ),
        (
            "canonical_cmd_vel_publisher",
            "publishers",
            CANONICAL_OUTPUT_TOPIC,
            (),
        ),
        (
            "behavior_sink_subscriber",
            "subscribers",
            BEHAVIOR_SINK_TOPIC,
            (),
        ),
    ]
    if phase == "pre_guard":
        exact_topology.extend(
            (
                (
                    "guard_input_subscriber_pre_guard",
                    "subscribers",
                    GUARD_INPUT_TOPIC,
                    (),
                ),
                (
                    "staged_output_publisher_pre_guard",
                    "publishers",
                    STAGED_OUTPUT_TOPIC,
                    (),
                ),
            )
        )
    else:
        exact_topology.extend(
            (
                (
                    "guard_input_subscriber_post_guard",
                    "subscribers",
                    GUARD_INPUT_TOPIC,
                    (EXPECTED_GUARD,),
                ),
                (
                    "staged_output_publisher_post_guard",
                    "publishers",
                    STAGED_OUTPUT_TOPIC,
                    (EXPECTED_GUARD,),
                ),
            )
        )
    for check_id, direction, topic, expected in exact_topology:
        observed = _endpoint_names(graph, direction, topic)
        _check(
            checks,
            check_id,
            observed == expected,
            f"expected exact {direction} {expected!r}",
            observed,
        )

    passed = all(bool(check["passed"]) for check in checks)
    localization = samples.get("localization")
    localization_witness = (
        {
            "x_m": localization.get("x_m"),
            "y_m": localization.get("y_m"),
            "yaw_rad": localization.get("yaw_rad"),
            "source_stamp_ns": localization.get("source_stamp_ns"),
            "received_monotonic_ns": localization.get(
                "received_monotonic_ns"
            ),
        }
        if isinstance(localization, Mapping)
        else None
    )
    return {
        "schema": SCHEMA,
        "read_only": True,
        "profile": PROFILE,
        "phase": phase,
        "session_id": session_id,
        "network_interface": network_interface,
        "result": "pass" if passed else "fail",
        "passed": passed,
        "map": {
            "id": expected_map_id,
            "mode": "localization",
            "generation": expected_generation,
        },
        "allowed_state": {
            "mode": allowed_mode,
            "marker": allowed_state_marker,
        },
        "expected_start": (
            {
                "x_m": expected_start_x,
                "y_m": expected_start_y,
                "maximum_distance_m": MAX_START_DISTANCE_M,
            }
            if phase == "post_guard"
            else None
        ),
        "localization": localization_witness,
        "timestamps": {
            "evaluated_realtime_ns": int(now_realtime_ns),
            "evaluated_monotonic_ns": int(now_monotonic_ns),
        },
        "freshness_limits_ns": {
            "receipt": dict(RECEIPT_FRESHNESS_NS),
            "source": dict(SOURCE_FRESHNESS_NS),
            "maximum_source_future_ns": MAX_SOURCE_FUTURE_NS,
        },
        "checks": checks,
        "failures": [
            str(check["id"]) for check in checks if not check["passed"]
        ],
        "safety": {
            "mode": "subscription-and-graph-only",
            "ros_publishers_created": False,
            "ros_services_created": False,
            "ros_clients_created": False,
            "ros_actions_created": False,
            "unitree_clients_created": False,
            "motion_commands_sent": False,
            "network_configuration_changed": False,
            "motion_ready": False,
        },
    }


def expected_check_ids(phase: str) -> frozenset[str]:
    """Return the exact check contract for a readiness phase."""

    if phase not in PHASES:
        raise ReadinessError(f"unsupported phase: {phase}")
    identifiers = {
        "map_valid_and_fresh",
        "map_lifecycle_exact",
        "localization_valid_and_fresh",
        "odom_valid_and_fresh",
        "scan_valid_and_fresh",
        "state_valid_and_fresh",
        "tf_map_to_odom_valid_and_fresh",
        "tf_odom_to_base_link_valid_and_fresh",
        "nav2_goal_status_fresh_and_idle",
        "chassis_disarmed",
        "map_unique_publisher",
        "map_lifecycle_unique_publisher",
        "navigate_to_pose_action_graph",
        "guard_input_publisher",
        "staged_output_subscriber",
        "canonical_cmd_vel_publisher",
        "behavior_sink_subscriber",
    }
    identifiers.update(
        f"lifecycle_graph_{node.strip('/')}"
        for node in REQUIRED_LIFECYCLE_NODES
    )
    if phase == "pre_guard":
        identifiers.update(
            {
                "guard_input_subscriber_pre_guard",
                "staged_output_publisher_pre_guard",
            }
        )
    else:
        identifiers.update(
            {
                "guard_input_subscriber_post_guard",
                "staged_output_publisher_post_guard",
                "localization_matches_expected_start",
            }
        )
    return frozenset(identifiers)


def validate_readiness_receipt(
    payload: object,
    *,
    phase: str,
    session_id: str,
    network_interface: str,
    map_id: str,
    map_generation: int,
    allowed_mode: int,
    allowed_state_marker: int,
    expected_start_x: float | None,
    expected_start_y: float | None,
    now_realtime_ns: int,
    maximum_receipt_age_ns: int = 30_000_000_000,
) -> dict[str, Any]:
    """Strictly validate a readiness receipt for launcher/permit reuse.

    The function is intentionally ROS-free and deterministic.  Callers must
    supply their current realtime value instead of allowing this validator to
    read a clock implicitly.
    """

    if not isinstance(payload, dict):
        raise ReadinessError("readiness receipt must be a JSON object")
    if payload.get("schema") != SCHEMA:
        raise ReadinessError("readiness receipt schema mismatch")
    if payload.get("read_only") is not True:
        raise ReadinessError("readiness receipt must assert read_only=true")
    if payload.get("profile") != PROFILE:
        raise ReadinessError("readiness receipt profile mismatch")
    exact_scalars = {
        "phase": phase,
        "session_id": session_id,
        "network_interface": network_interface,
        "result": "pass",
    }
    for key, expected in exact_scalars.items():
        if payload.get(key) != expected:
            raise ReadinessError(f"readiness receipt {key} mismatch")
    if payload.get("passed") is not True:
        raise ReadinessError("readiness receipt did not pass")
    if payload.get("map") != {
        "id": map_id,
        "generation": map_generation,
        "mode": "localization",
    }:
        raise ReadinessError("readiness receipt map identity mismatch")
    if payload.get("allowed_state") != {
        "mode": allowed_mode,
        "marker": allowed_state_marker,
    }:
        raise ReadinessError("readiness receipt allowed state mismatch")
    expected_start = (
        {
            "x_m": expected_start_x,
            "y_m": expected_start_y,
            "maximum_distance_m": MAX_START_DISTANCE_M,
        }
        if phase == "post_guard"
        else None
    )
    if (
        phase == "post_guard"
        and (
            expected_start_x is None
            or expected_start_y is None
            or not _all_finite((expected_start_x, expected_start_y))
        )
    ):
        raise ReadinessError(
            "post_guard receipt validation requires expected path start"
        )
    if payload.get("expected_start") != expected_start:
        raise ReadinessError("readiness receipt expected path start mismatch")
    if payload.get("freshness_limits_ns") != {
        "receipt": dict(RECEIPT_FRESHNESS_NS),
        "source": dict(SOURCE_FRESHNESS_NS),
        "maximum_source_future_ns": MAX_SOURCE_FUTURE_NS,
    }:
        raise ReadinessError("readiness receipt freshness contract mismatch")

    timestamps = payload.get("timestamps")
    if not isinstance(timestamps, dict):
        raise ReadinessError("readiness receipt timestamps are missing")
    realtime = timestamps.get("evaluated_realtime_ns")
    monotonic = timestamps.get("evaluated_monotonic_ns")
    if (
        isinstance(realtime, bool)
        or not isinstance(realtime, int)
        or realtime <= 0
        or isinstance(monotonic, bool)
        or not isinstance(monotonic, int)
        or monotonic <= 0
    ):
        raise ReadinessError("readiness receipt timestamps are invalid")
    age_ns = now_realtime_ns - realtime
    if age_ns < -MAX_SOURCE_FUTURE_NS:
        raise ReadinessError("readiness receipt timestamp is in the future")
    if age_ns > maximum_receipt_age_ns:
        raise ReadinessError("readiness receipt is stale")

    localization = payload.get("localization")
    localization_keys = {
        "x_m",
        "y_m",
        "yaw_rad",
        "source_stamp_ns",
        "received_monotonic_ns",
    }
    if not isinstance(localization, dict) or set(localization) != localization_keys:
        raise ReadinessError("readiness receipt localization witness is malformed")
    coordinates = (
        localization.get("x_m"),
        localization.get("y_m"),
        localization.get("yaw_rad"),
    )
    if not _all_finite(coordinates):
        raise ReadinessError("readiness receipt localization pose is not finite")
    if not -math.pi <= float(localization["yaw_rad"]) <= math.pi:
        raise ReadinessError("readiness receipt localization yaw is invalid")
    source_stamp = localization.get("source_stamp_ns")
    received_monotonic = localization.get("received_monotonic_ns")
    if (
        isinstance(source_stamp, bool)
        or not isinstance(source_stamp, int)
        or source_stamp <= 0
        or isinstance(received_monotonic, bool)
        or not isinstance(received_monotonic, int)
        or received_monotonic <= 0
    ):
        raise ReadinessError(
            "readiness receipt localization timestamps are invalid"
        )
    source_age = realtime - source_stamp
    receipt_age = monotonic - received_monotonic
    if (
        source_age < -MAX_SOURCE_FUTURE_NS
        or source_age > SOURCE_FRESHNESS_NS["localization"]
        or receipt_age < 0
        or receipt_age > RECEIPT_FRESHNESS_NS["localization"]
    ):
        raise ReadinessError(
            "readiness receipt localization witness is stale"
        )
    if phase == "post_guard":
        start_distance = math.hypot(
            float(localization["x_m"]) - float(expected_start_x),
            float(localization["y_m"]) - float(expected_start_y),
        )
        if start_distance > MAX_START_DISTANCE_M:
            raise ReadinessError(
                "readiness receipt localization does not match path start"
            )

    checks = payload.get("checks")
    if not isinstance(checks, list) or not checks:
        raise ReadinessError("readiness receipt checks are missing")
    identifiers: list[str] = []
    for check in checks:
        if not isinstance(check, dict):
            raise ReadinessError("readiness receipt contains a malformed check")
        identifier = check.get("id")
        if not isinstance(identifier, str) or not identifier:
            raise ReadinessError("readiness receipt check id is invalid")
        if check.get("passed") is not True:
            raise ReadinessError(f"readiness check did not pass: {identifier}")
        identifiers.append(identifier)
    if len(identifiers) != len(set(identifiers)):
        raise ReadinessError("readiness receipt contains duplicate checks")
    expected = expected_check_ids(phase)
    if frozenset(identifiers) != expected:
        raise ReadinessError("readiness receipt check set mismatch")
    if payload.get("failures") != []:
        raise ReadinessError("passing readiness receipt must have no failures")

    safety = payload.get("safety")
    if not isinstance(safety, dict):
        raise ReadinessError("readiness receipt safety section is missing")
    required_safety = {
        "mode": "subscription-and-graph-only",
        "ros_publishers_created": False,
        "ros_services_created": False,
        "ros_clients_created": False,
        "ros_actions_created": False,
        "unitree_clients_created": False,
        "motion_commands_sent": False,
        "network_configuration_changed": False,
        "motion_ready": False,
    }
    if safety != required_safety:
        raise ReadinessError("readiness receipt safety assertions mismatch")
    return payload


def _graph_snapshot(
    node_handle: Any,
    implementation: Any,
    topic_endpoint_info: Any,
) -> dict[str, Any]:
    publishers: dict[str, list[str]] = {}
    subscribers: dict[str, list[str]] = {}
    endpoint_topics = (
        *TOPOLOGY_TOPICS,
        MAP_TOPIC,
        LIFECYCLE_TOPIC,
        GOAL_STATUS_TOPIC,
        GOAL_FEEDBACK_TOPIC,
    )
    for topic in endpoint_topics:
        publisher_info = implementation.rclpy_get_publishers_info_by_topic(
            node_handle, topic, False
        )
        subscriber_info = implementation.rclpy_get_subscriptions_info_by_topic(
            node_handle, topic, False
        )
        publishers[topic] = [
            _canonical_endpoint(endpoint.node_name, endpoint.node_namespace)
            for endpoint in (
                topic_endpoint_info(**item) for item in publisher_info
            )
        ]
        subscribers[topic] = [
            _canonical_endpoint(endpoint.node_name, endpoint.node_namespace)
            for endpoint in (
                topic_endpoint_info(**item) for item in subscriber_info
            )
        ]

    services = {
        str(name): [str(item) for item in types]
        for name, types in implementation.rclpy_get_service_names_and_types(
            node_handle
        )
    }
    topics = {
        str(name): [str(item) for item in types]
        for name, types in implementation.rclpy_get_topic_names_and_types(
            node_handle, False
        )
    }
    services_by_node: dict[str, dict[str, list[str]]] = {}
    for full_name in REQUIRED_LIFECYCLE_NODES:
        name, namespace = _node_parts(full_name)
        try:
            discovered = (
                implementation.rclpy_get_service_names_and_types_by_node(
                    node_handle, name, namespace
                )
            )
        except Exception:
            discovered = []
        services_by_node[full_name] = {
            str(service): [str(item) for item in types]
            for service, types in discovered
        }
    return {
        "publishers": publishers,
        "subscribers": subscribers,
        "topics": topics,
        "services": services,
        "services_by_node": services_by_node,
    }


def collect_live_snapshot(duration_s: float) -> dict[str, Any]:
    """Collect bounded ROS evidence using only low-level subscriptions."""

    # All ROS imports are lazy so importing this module is an offline operation.
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from map.msg import MapLifecycle
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.context import Context
    from rclpy.impl.implementation_singleton import (
        rclpy_implementation as _rclpy,
    )
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from rclpy.topic_endpoint_info import TopicEndpointInfo
    from rclpy.type_support import check_is_valid_msg_type
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from unitree_go.msg import SportModeState

    reliable = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    best_effort = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=32,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    latched = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    converters: dict[
        str, tuple[Any, Any, Callable[[Any, int], dict[str, Any]]]
    ] = {
        "map": (OccupancyGrid, latched, _map_sample),
        "map_lifecycle": (MapLifecycle, latched, _lifecycle_sample),
        "localization": (
            PoseWithCovarianceStamped,
            reliable,
            _pose_sample,
        ),
        "odom": (Odometry, best_effort, _odom_sample),
        "scan": (LaserScan, best_effort, _scan_sample),
        "state": (SportModeState, best_effort, _state_sample),
        "goal_status": (
            GoalStatusArray,
            reliable,
            _goal_status_sample,
        ),
        "chassis": (String, reliable, _chassis_sample),
        "tf": (TFMessage, best_effort, _tf_samples),
        "tf_static": (TFMessage, latched, _tf_samples),
    }
    topics = {
        "map": MAP_TOPIC,
        "map_lifecycle": LIFECYCLE_TOPIC,
        "localization": LOCALIZATION_TOPIC,
        "odom": ODOM_TOPIC,
        "scan": SCAN_TOPIC,
        "state": STATE_TOPIC,
        "goal_status": GOAL_STATUS_TOPIC,
        "chassis": CHASSIS_STATUS_TOPIC,
        "tf": TF_TOPIC,
        "tf_static": TF_STATIC_TOPIC,
    }

    context = None
    context_initialized = False
    node_handle = None
    wait_set = None
    subscriptions: list[Any] = []
    metadata: dict[int, tuple[str, Any, Callable[[Any, int], Any]]] = {}
    samples: dict[str, Any] = {"tf_edges": {}}
    started_realtime_ns = time.time_ns()
    started_monotonic_ns = time.monotonic_ns()
    graph: dict[str, Any] = {}
    try:
        context = Context()
        context.init(args=[], initialize_logging=False)
        context_initialized = True
        for message_type, _qos, _converter in converters.values():
            check_is_valid_msg_type(message_type)
        with context.handle:
            node_handle = _rclpy.Node(
                "go2_staged_nav2_readiness_readonly",
                "",
                context.handle,
                None,
                False,
                False,
            )
            wait_set = _rclpy.WaitSet(
                len(converters), 0, 0, 0, 0, 0, context.handle
            )
        with node_handle:
            for key, (message_type, qos, converter) in converters.items():
                subscription = _rclpy.Subscription(
                    node_handle,
                    message_type,
                    topics[key],
                    qos.get_c_qos_profile(),
                )
                subscriptions.append(subscription)
                metadata[subscription.pointer] = (
                    key,
                    message_type,
                    converter,
                )

        deadline_ns = (
            started_monotonic_ns + int(duration_s * 1_000_000_000)
        )
        while context.ok():
            remaining_ns = deadline_ns - time.monotonic_ns()
            if remaining_ns <= 0:
                break
            wait_set.clear_entities()
            for subscription in subscriptions:
                wait_set.add_subscription(subscription)
            wait_set.wait(min(100_000_000, remaining_ns))
            ready = set(wait_set.get_ready_entities("subscription"))
            for subscription in subscriptions:
                if subscription.pointer not in ready:
                    continue
                key, message_type, converter = metadata[subscription.pointer]
                message_info = subscription.take_message(message_type, False)
                if message_info is None:
                    continue
                received_ns = time.monotonic_ns()
                converted = converter(message_info[0], received_ns)
                if key in {"tf", "tf_static"}:
                    # Required navigation transforms must be live dynamic TF.
                    # Static samples are retained only as diagnostic evidence
                    # and never satisfy the required dynamic-edge checks.
                    if key == "tf":
                        samples["tf_edges"].update(converted)
                    elif converted:
                        samples["tf_static_edges"] = converted
                else:
                    samples[key] = converted
        with node_handle:
            graph = _graph_snapshot(node_handle, _rclpy, TopicEndpointInfo)
        # Map and lifecycle are transient-local state, not periodic sensor
        # streams.  A valid latched sample remains a current witness only while
        # its topic still has exactly one publisher.  Refreshing the receipt
        # timestamp here avoids treating the first latched delivery as stale
        # after a multi-second collection without weakening publisher identity.
        static_refresh_ns = time.monotonic_ns()
        for key, topic in (
            ("map", MAP_TOPIC),
            ("map_lifecycle", LIFECYCLE_TOPIC),
        ):
            if (
                isinstance(samples.get(key), dict)
                and len(_endpoint_names(graph, "publishers", topic)) == 1
            ):
                samples[key]["received_monotonic_ns"] = static_refresh_ns
                samples[key]["witness_refreshed_by_unique_publisher"] = True
    finally:
        if wait_set is not None:
            try:
                wait_set.clear_entities()
            except Exception:
                pass
            try:
                wait_set.destroy_when_not_in_use()
            except Exception:
                pass
        for subscription in reversed(subscriptions):
            try:
                subscription.destroy_when_not_in_use()
            except Exception:
                pass
        if node_handle is not None:
            try:
                node_handle.destroy_when_not_in_use()
            except Exception:
                pass
        if context is not None and context_initialized:
            try:
                context.try_shutdown()
            except Exception:
                pass
            try:
                context.destroy()
            except Exception:
                pass

    finished_monotonic_ns = time.monotonic_ns()
    return {
        "samples": samples,
        "graph": graph,
        "collection": {
            "started_realtime_ns": started_realtime_ns,
            "started_monotonic_ns": started_monotonic_ns,
            "finished_realtime_ns": time.time_ns(),
            "finished_monotonic_ns": finished_monotonic_ns,
            "elapsed_monotonic_ns": (
                finished_monotonic_ns - started_monotonic_ns
            ),
            "duration_limit_ns": int(duration_s * 1_000_000_000),
            "subscription_count": len(subscriptions),
        },
    }


def run(arguments: argparse.Namespace) -> tuple[dict[str, Any], Path]:
    if arguments.phase == "post_guard" and (
        arguments.expected_start_x is None
        or arguments.expected_start_y is None
    ):
        raise ReadinessError(
            "post_guard requires --expected-start-x and --expected-start-y"
        )
    if arguments.phase == "pre_guard" and (
        arguments.expected_start_x is not None
        or arguments.expected_start_y is not None
    ):
        raise ReadinessError(
            "pre_guard must not receive --expected-start-x/--expected-start-y"
        )
    output = _output_path(arguments.output)
    snapshot = collect_live_snapshot(arguments.duration)
    evaluated_monotonic_ns = time.monotonic_ns()
    evaluated_realtime_ns = time.time_ns()
    receipt = evaluate_snapshot(
        snapshot,
        phase=arguments.phase,
        session_id=arguments.session_id,
        network_interface=arguments.network_interface,
        expected_map_id=arguments.map_id,
        expected_generation=arguments.map_generation,
        allowed_mode=arguments.allowed_mode,
        allowed_state_marker=arguments.allowed_state_marker,
        expected_start_x=arguments.expected_start_x,
        expected_start_y=arguments.expected_start_y,
        now_monotonic_ns=evaluated_monotonic_ns,
        now_realtime_ns=evaluated_realtime_ns,
    )
    receipt["collection"] = snapshot.get("collection", {})
    if receipt["passed"]:
        validate_readiness_receipt(
            receipt,
            phase=arguments.phase,
            session_id=arguments.session_id,
            network_interface=arguments.network_interface,
            map_id=arguments.map_id,
            map_generation=arguments.map_generation,
            allowed_mode=arguments.allowed_mode,
            allowed_state_marker=arguments.allowed_state_marker,
            expected_start_x=arguments.expected_start_x,
            expected_start_y=arguments.expected_start_y,
            now_realtime_ns=evaluated_realtime_ns,
        )
    _write_exclusive_private(output, receipt)
    return receipt, output


def main() -> int:
    arguments = build_argument_parser().parse_args()
    try:
        receipt, output = run(arguments)
    except (OSError, ReadinessError) as error:
        print(f"staged Nav2 readiness failed: {error}", flush=True)
        return 2
    print(output)
    return 0 if receipt["passed"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
