#!/usr/bin/env python3
"""Create read-only, known-space evidence for one short staged Nav2 goal.

This tool calls only ``ComputePathToPose``.  It never sends a navigation goal,
creates a velocity publisher, calls the chassis arm service, or invokes a
Unitree API.  A successful artifact binds one localization map generation and
one operator-reviewed goal to a collision-free, known-space path no longer
than 0.40 m.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = "robonix-go2-staged-nav2-short-goal-evidence-v1"
ACTION_NAME = "/compute_path_to_pose"
MAP_TOPIC = "/map"
MAP_LIFECYCLE_TOPIC = "/robonix/map/lifecycle"
FRAME_ID = "map"
GOAL_SOURCE = "operator_reviewed_short_goal"
MAX_PATH_M = 0.40
LETHAL_OCCUPANCY = 65
INTERFACE_RE = re.compile(r"^[A-Za-z0-9_.:-]+$")
SESSION_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{7,63}$")
TARGET_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$")
CHECK_NAMES = frozenset(
    {
        "goal_bearing_within_stage1",
        "map_lifecycle_exact",
        "plan_only_action",
        "path_collision_free",
        "path_endpoint_matches_goal",
        "path_finite",
        "path_frame_map",
        "path_known_space",
        "path_length_le_0_40_m",
        "path_nonempty",
        "path_start_matches_localization",
    }
)


def finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("must be finite")
    return parsed


def positive_integer(value: str) -> int:
    parsed = int(value, 10)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def bounded_timeout(value: str) -> float:
    parsed = finite_float(value)
    if not 1.0 <= parsed <= 120.0:
        raise argparse.ArgumentTypeError("must be in the range 1..120 seconds")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--map-generation", type=positive_integer, required=True)
    parser.add_argument("--target-id", required=True)
    parser.add_argument("--x", type=finite_float)
    parser.add_argument("--y", type=finite_float)
    parser.add_argument("--yaw", type=finite_float)
    parser.add_argument(
        "--forward-distance",
        type=finite_float,
        help="derive a goal this far ahead of the current live localization",
    )
    parser.add_argument("--planner-id", default="GridBased")
    parser.add_argument("--timeout-s", default=20.0, type=bounded_timeout)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if SESSION_RE.fullmatch(args.map_id) is None:
        parser.error("--map-id must be 8-64 safe ASCII characters")
    if TARGET_RE.fullmatch(args.target_id) is None:
        parser.error("--target-id must be 1-64 safe ASCII characters")
    explicit = (args.x, args.y, args.yaw)
    if args.forward_distance is None and any(value is None for value in explicit):
        parser.error("supply x/y/yaw together, or use --forward-distance")
    if args.forward_distance is not None and any(
        value is not None for value in explicit
    ):
        parser.error("--forward-distance cannot be combined with x/y/yaw")
    if args.forward_distance is not None and not 0.05 <= args.forward_distance <= 0.30:
        parser.error("--forward-distance must be in 0.05..0.30 m")
    if args.yaw is not None and not -math.pi <= args.yaw <= math.pi:
        parser.error("--yaw must be within [-pi, pi]")
    return args


def resolve_output(raw: str) -> Path:
    candidate = Path(raw).expanduser()
    if not candidate.is_absolute():
        candidate = ROOT / candidate
    if candidate.exists() and candidate.is_symlink():
        raise ValueError("output must not be a symlink")
    output = candidate.resolve()
    if not output.is_relative_to(ROOT):
        raise ValueError(f"output must remain below {ROOT}")
    return output


def configure_process_local_dds(environ: dict[str, str]) -> str:
    interface = environ.get("GO2_NETWORK_INTERFACE", "").strip()
    if INTERFACE_RE.fullmatch(interface) is None:
        raise ValueError("GO2_NETWORK_INTERFACE is required and must be safe")
    if not (Path("/sys/class/net") / interface).is_dir():
        raise ValueError(f"GO2_NETWORK_INTERFACE does not exist: {interface}")
    owned_uri = (
        "<CycloneDDS><Domain><General><Interfaces>"
        f'<NetworkInterface name="{interface}" priority="default" multicast="default"/>'
        "</Interfaces></General></Domain></CycloneDDS>"
    )
    inherited = environ.get("CYCLONEDDS_URI", "")
    if inherited and inherited != owned_uri:
        raise ValueError(
            "inherited CYCLONEDDS_URI does not match GO2_NETWORK_INTERFACE"
        )
    environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    environ["CYCLONEDDS_URI"] = owned_uri
    environ["GO2_ALLOW_MOTION"] = "false"
    environ["GO2_OPERATOR_PRESENT"] = "false"
    environ["GO2_SAFETY_ACK"] = ""
    environ["GO2_ALLOWED_MODES"] = ""
    environ["GO2_ALLOWED_STATE_MARKERS"] = ""
    return interface


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} is not finite")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{label} is not finite")
    return parsed


def path_points(path: Any) -> list[tuple[float, float]]:
    if str(path.header.frame_id) != FRAME_ID:
        raise ValueError("path frame is not map")
    points: list[tuple[float, float]] = []
    for index, stamped in enumerate(path.poses):
        if str(stamped.header.frame_id) not in {"", FRAME_ID}:
            raise ValueError(f"path pose {index} frame is not map")
        pose = stamped.pose
        point = (
            _finite(pose.position.x, f"path pose {index} x"),
            _finite(pose.position.y, f"path pose {index} y"),
        )
        for name in ("z",):
            _finite(getattr(pose.position, name), f"path pose {index} {name}")
        for name in ("x", "y", "z", "w"):
            _finite(
                getattr(pose.orientation, name),
                f"path pose {index} q{name}",
            )
        points.append(point)
    if not points:
        raise ValueError("planner returned an empty path")
    return points


def path_length(points: Iterable[tuple[float, float]]) -> float:
    values = list(points)
    return sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(values, values[1:])
    )


def pose_yaw(pose: Any) -> float:
    quaternion = pose.orientation
    x = _finite(quaternion.x, "path endpoint qx")
    y = _finite(quaternion.y, "path endpoint qy")
    z = _finite(quaternion.z, "path endpoint qz")
    w = _finite(quaternion.w, "path endpoint qw")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("path endpoint quaternion is degenerate")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _origin_yaw(origin: Any) -> float:
    quaternion = origin.orientation
    x = _finite(quaternion.x, "map origin qx")
    y = _finite(quaternion.y, "map origin qy")
    z = _finite(quaternion.z, "map origin qz")
    w = _finite(quaternion.w, "map origin qw")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("map origin quaternion is degenerate")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def stamp_age_s(node: Any, stamp: Any) -> float:
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return (
        int(node.get_clock().now().nanoseconds) - stamp_ns
    ) / 1_000_000_000.0


def validate_path_against_map(
    points: list[tuple[float, float]],
    grid: Any,
) -> dict[str, object]:
    if str(grid.header.frame_id) != FRAME_ID:
        raise ValueError("occupancy grid frame is not map")
    width = int(grid.info.width)
    height = int(grid.info.height)
    resolution = _finite(grid.info.resolution, "map resolution")
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError("occupancy grid dimensions are invalid")
    data = list(grid.data)
    if len(data) != width * height:
        raise ValueError("occupancy grid data length is invalid")
    origin_x = _finite(grid.info.origin.position.x, "map origin x")
    origin_y = _finite(grid.info.origin.position.y, "map origin y")
    origin_yaw = _origin_yaw(grid.info.origin)
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)

    sampled: list[tuple[float, float]] = [points[0]]
    maximum_step = resolution * 0.5
    for previous, current in zip(points, points[1:]):
        distance = math.hypot(
            current[0] - previous[0], current[1] - previous[1]
        )
        subdivisions = max(1, math.ceil(distance / maximum_step))
        for index in range(1, subdivisions + 1):
            ratio = index / subdivisions
            sampled.append(
                (
                    previous[0] + (current[0] - previous[0]) * ratio,
                    previous[1] + (current[1] - previous[1]) * ratio,
                )
            )

    for index, (world_x, world_y) in enumerate(sampled):
        delta_x = world_x - origin_x
        delta_y = world_y - origin_y
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        cell_x = math.floor(local_x / resolution)
        cell_y = math.floor(local_y / resolution)
        if not (0 <= cell_x < width and 0 <= cell_y < height):
            raise ValueError(f"path sample {index} is outside the map")
        occupancy = int(data[cell_y * width + cell_x])
        if occupancy < 0:
            raise ValueError(f"path sample {index} crosses unknown space")
        if occupancy >= LETHAL_OCCUPANCY:
            raise ValueError(f"path sample {index} crosses occupied space")
    return {
        "sample_count": len(sampled),
        "map_width": width,
        "map_height": height,
        "map_resolution_m": resolution,
        "lethal_occupancy_threshold": LETHAL_OCCUPANCY,
    }


def write_evidence(output: Path, payload: dict[str, object]) -> None:
    output.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=output.parent,
        prefix=f".{output.name}.",
        delete=False,
    ) as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    temporary.replace(output)


def main() -> int:
    args = parse_args()
    try:
        output = resolve_output(args.output)
        interface = configure_process_local_dds(os.environ)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    evidence: dict[str, object] = {
        "schema": SCHEMA,
        "operation": "compute_path_to_pose_only",
        "action": ACTION_NAME,
        "frame_id": FRAME_ID,
        "motion_disabled": True,
        "network_interface": interface,
        "map": {
            "id": args.map_id,
            "generation": args.map_generation,
            "mode": "localization",
        },
        "goal": {
            "source": GOAL_SOURCE,
            "target_id": args.target_id,
            "pose": {"x": args.x, "y": args.y, "yaw": args.yaw},
        },
        "started_unix_ns": time.time_ns(),
        "status": "error",
        "checks": {name: "unknown" for name in sorted(CHECK_NAMES)},
    }
    node = None
    action_client = None
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from map.msg import MapLifecycle
        from nav2_msgs.action import ComputePathToPose
        from nav_msgs.msg import OccupancyGrid
        from rclpy.action import ActionClient
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        rclpy.init(args=None)
        node = rclpy.create_node("robonix_staged_nav2_plan_only")
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        observations: dict[str, object] = {}
        node.create_subscription(
            OccupancyGrid,
            MAP_TOPIC,
            lambda message: observations.__setitem__("map", message),
            map_qos,
        )
        node.create_subscription(
            MapLifecycle,
            MAP_LIFECYCLE_TOPIC,
            lambda message: observations.__setitem__("lifecycle", message),
            map_qos,
        )
        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        node.create_subscription(
            PoseWithCovarianceStamped,
            "/robonix/map/pose",
            lambda message: observations.__setitem__("localization", message),
            reliable,
        )
        observation_deadline = time.monotonic() + min(args.timeout_s, 10.0)
        while (
            not {"map", "lifecycle", "localization"}.issubset(observations)
            and time.monotonic() < observation_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        if not {"map", "lifecycle", "localization"}.issubset(observations):
            raise RuntimeError(
                "map, map lifecycle, or localization was not available"
            )
        lifecycle = observations["lifecycle"]
        if (
            str(lifecycle.map_id) != args.map_id
            or str(lifecycle.mode).lower() != "localization"
            or int(lifecycle.generation) != args.map_generation
        ):
            raise RuntimeError("map lifecycle does not match requested map")
        evidence["checks"]["map_lifecycle_exact"] = "pass"

        if args.forward_distance is not None:
            current_pose = observations["localization"].pose.pose
            current_x = _finite(
                current_pose.position.x, "localization x"
            )
            current_y = _finite(
                current_pose.position.y, "localization y"
            )
            current_yaw = pose_yaw(current_pose)
            args.x = current_x + args.forward_distance * math.cos(current_yaw)
            args.y = current_y + args.forward_distance * math.sin(current_yaw)
            args.yaw = current_yaw
            evidence["goal"]["pose"] = {
                "x": args.x,
                "y": args.y,
                "yaw": args.yaw,
            }

        action_client = ActionClient(node, ComputePathToPose, ACTION_NAME)
        if not action_client.wait_for_server(timeout_sec=args.timeout_s):
            raise RuntimeError(f"{ACTION_NAME} server was not ready")
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = FRAME_ID
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x = args.x
        goal.goal.pose.position.y = args.y
        goal.goal.pose.orientation.z = math.sin(args.yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(args.yaw / 2.0)
        goal.planner_id = args.planner_id
        goal.use_start = False

        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.timeout_s)
        if not send_future.done():
            raise RuntimeError("planner goal response timed out")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("planner rejected the goal")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            node, result_future, timeout_sec=args.timeout_s
        )
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("planner result timed out or was absent")
        wrapped = result_future.result()
        if int(wrapped.status) != int(GoalStatus.STATUS_SUCCEEDED):
            raise RuntimeError(
                f"planner action status was {wrapped.status}, not SUCCEEDED"
            )
        evidence["action_status"] = int(wrapped.status)
        evidence["action_status_name"] = "succeeded"
        evidence["checks"]["plan_only_action"] = "pass"

        points = path_points(wrapped.result.path)
        evidence["checks"]["path_frame_map"] = "pass"
        evidence["checks"]["path_finite"] = "pass"
        evidence["checks"]["path_nonempty"] = "pass"
        length_m = path_length(points)
        if not math.isfinite(length_m) or length_m > MAX_PATH_M:
            raise RuntimeError(
                f"planned path is {length_m:.6f} m, above {MAX_PATH_M:.2f} m"
            )
        evidence["checks"]["path_length_le_0_40_m"] = "pass"
        path_start_yaw = pose_yaw(wrapped.result.path.poses[0].pose)
        localization_message = observations["localization"]
        localization_age_s = stamp_age_s(
            node, localization_message.header.stamp
        )
        if not -0.05 <= localization_age_s <= 0.50:
            refresh_deadline = time.monotonic() + 0.75
            while (
                not -0.05 <= localization_age_s <= 0.50
                and time.monotonic() < refresh_deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.05)
                localization_message = observations["localization"]
                localization_age_s = stamp_age_s(
                    node, localization_message.header.stamp
                )
        if not -0.05 <= localization_age_s <= 0.50:
            raise RuntimeError("live localization is stale or future-dated")
        localization_pose = localization_message.pose.pose
        localization_x = _finite(
            localization_pose.position.x, "localization x"
        )
        localization_y = _finite(
            localization_pose.position.y, "localization y"
        )
        localization_yaw = pose_yaw(localization_pose)
        start_error_m = math.hypot(
            points[0][0] - localization_x, points[0][1] - localization_y
        )
        if start_error_m > 0.05:
            raise RuntimeError(
                "planned path start differs from live localization"
            )
        evidence["checks"]["path_start_matches_localization"] = "pass"
        evidence["localization"] = {
            "pose": {
                "x": localization_x,
                "y": localization_y,
                "yaw": localization_yaw,
            },
            "source_age_s": localization_age_s,
        }
        goal_delta_x = args.x - localization_x
        goal_delta_y = args.y - localization_y
        goal_distance_m = math.hypot(goal_delta_x, goal_delta_y)
        desired_heading = (
            math.atan2(goal_delta_y, goal_delta_x)
            if goal_distance_m > 1e-6
            else args.yaw
        )
        goal_bearing_error = abs(
            math.atan2(
                math.sin(desired_heading - localization_yaw),
                math.cos(desired_heading - localization_yaw),
            )
        )
        if goal_bearing_error > 0.50:
            raise RuntimeError(
                "reviewed goal is outside the stage1 forward bearing"
            )
        evidence["checks"]["goal_bearing_within_stage1"] = "pass"
        evidence["stage1_geometry"] = {
            "goal_distance_m": goal_distance_m,
            "goal_bearing_error_rad": goal_bearing_error,
        }
        endpoint_yaw = pose_yaw(wrapped.result.path.poses[-1].pose)
        endpoint_error_m = math.hypot(
            points[-1][0] - args.x, points[-1][1] - args.y
        )
        endpoint_yaw_error = abs(
            math.atan2(
                math.sin(endpoint_yaw - args.yaw),
                math.cos(endpoint_yaw - args.yaw),
            )
        )
        if endpoint_error_m > 0.02 or endpoint_yaw_error > 0.10:
            raise RuntimeError(
                "planned path endpoint differs from the reviewed goal"
            )
        evidence["checks"]["path_endpoint_matches_goal"] = "pass"
        map_summary = validate_path_against_map(
            points, observations["map"]
        )
        evidence["checks"]["path_known_space"] = "pass"
        evidence["checks"]["path_collision_free"] = "pass"
        evidence["path"] = {
            "pose_count": len(points),
            "length_m": round(length_m, 9),
            "start": {
                "x": points[0][0],
                "y": points[0][1],
                "yaw": path_start_yaw,
            },
            "end": {
                "x": points[-1][0],
                "y": points[-1][1],
                "yaw": endpoint_yaw,
            },
            "endpoint_position_error_m": endpoint_error_m,
            "endpoint_yaw_error_rad": endpoint_yaw_error,
            **map_summary,
        }
        planning_time = wrapped.result.planning_time
        evidence["planning_time_s"] = round(
            int(planning_time.sec)
            + int(planning_time.nanosec) / 1_000_000_000.0,
            9,
        )
        if (
            set(evidence["checks"]) != CHECK_NAMES
            or any(value != "pass" for value in evidence["checks"].values())
        ):
            raise RuntimeError("not every required staged path check passed")
        evidence["status"] = "pass"
        return_code = 0
    except Exception as error:
        evidence["error"] = f"{type(error).__name__}: {error}"
        return_code = 1
    finally:
        evidence["finished_unix_ns"] = time.time_ns()
        write_evidence(output, evidence)
        if action_client is not None:
            action_client.destroy()
        if node is not None:
            node.destroy_node()
        try:
            if "rclpy" in locals() and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    if return_code == 0:
        path_claim = evidence["path"]
        print(
            "PASS: staged plan-only path "
            f"{path_claim['pose_count']} poses, {path_claim['length_m']} m; "
            f"evidence={output}"
        )
    else:
        print(f"FAIL: {evidence['error']}; evidence={output}")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
