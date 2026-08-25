#!/usr/bin/env python3
"""Request a Nav2 path without starting path following or chassis control."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parent.parent
ACTION_NAME = "/compute_path_to_pose"
FRAME_ID = "map"
INTERFACE_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]+$")


def finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("must be finite")
    return parsed


def bounded_timeout(value: str) -> float:
    parsed = finite_float(value)
    if parsed < 0.5 or parsed > 120.0:
        raise argparse.ArgumentTypeError("must be in the range 0.5..120 seconds")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Call Nav2 ComputePathToPose only. This tool never starts path "
            "execution and never creates a velocity publisher."
        )
    )
    parser.add_argument("--x", required=True, type=finite_float)
    parser.add_argument("--y", required=True, type=finite_float)
    parser.add_argument("--yaw", required=True, type=finite_float)
    parser.add_argument("--planner-id", default="GridBased")
    parser.add_argument("--timeout-s", default=20.0, type=bounded_timeout)
    parser.add_argument(
        "--output",
        required=True,
        help="JSON evidence path below this repository",
    )
    return parser.parse_args()


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


def configure_process_local_dds() -> str:
    interface = os.environ.get("GO2_NETWORK_INTERFACE", "").strip()
    if not interface:
        raise ValueError("GO2_NETWORK_INTERFACE is required")
    if INTERFACE_PATTERN.fullmatch(interface) is None:
        raise ValueError("GO2_NETWORK_INTERFACE contains unsupported characters")
    if not (Path("/sys/class/net") / interface).is_dir():
        raise ValueError(f"GO2_NETWORK_INTERFACE does not exist: {interface}")
    owned_uri = (
        "<CycloneDDS><Domain><General><Interfaces>"
        f'<NetworkInterface name="{interface}" priority="default" multicast="default"/>'
        "</Interfaces></General></Domain></CycloneDDS>"
    )
    inherited = os.environ.get("CYCLONEDDS_URI", "")
    if inherited and inherited != owned_uri:
        raise ValueError(
            "inherited CYCLONEDDS_URI does not match GO2_NETWORK_INTERFACE"
        )
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["CYCLONEDDS_URI"] = owned_uri
    os.environ["GO2_ALLOW_MOTION"] = "false"
    os.environ["GO2_OPERATOR_PRESENT"] = "false"
    os.environ["GO2_SAFETY_ACK"] = ""
    os.environ["GO2_ALLOWED_MODES"] = ""
    return interface


def _number(value: Any, label: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{label} is not finite")
    return parsed


def summarize_path(path: Any) -> dict[str, Any]:
    if str(path.header.frame_id) != FRAME_ID:
        raise ValueError(
            f"path frame must be {FRAME_ID!r}, got {path.header.frame_id!r}"
        )
    poses = list(path.poses)
    if not poses:
        raise ValueError("planner returned an empty path")
    points: list[tuple[float, float]] = []
    for index, stamped in enumerate(poses):
        pose_frame = str(stamped.header.frame_id)
        if pose_frame and pose_frame != FRAME_ID:
            raise ValueError(
                f"path pose {index} frame must be empty or {FRAME_ID!r}, "
                f"got {pose_frame!r}"
            )
        pose = stamped.pose
        x = _number(pose.position.x, f"path pose {index} x")
        y = _number(pose.position.y, f"path pose {index} y")
        _number(pose.position.z, f"path pose {index} z")
        _number(pose.orientation.x, f"path pose {index} qx")
        _number(pose.orientation.y, f"path pose {index} qy")
        _number(pose.orientation.z, f"path pose {index} qz")
        _number(pose.orientation.w, f"path pose {index} qw")
        points.append((x, y))
    length = sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(points, points[1:])
    )
    return {
        "frame_id": FRAME_ID,
        "pose_count": len(points),
        "length_m": round(length, 6),
        "start": {"x": round(points[0][0], 6), "y": round(points[0][1], 6)},
        "end": {"x": round(points[-1][0], 6), "y": round(points[-1][1], 6)},
    }


def write_evidence(output: Path, payload: dict[str, Any]) -> None:
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
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    temporary.replace(output)


def main() -> int:
    args = parse_args()
    try:
        output = resolve_output(args.output)
        interface = configure_process_local_dds()
    except ValueError as error:
        raise SystemExit(str(error)) from error

    evidence: dict[str, Any] = {
        "schema_version": 1,
        "operation": "compute_path_to_pose_only",
        "action": ACTION_NAME,
        "frame_id": FRAME_ID,
        "planner_id": args.planner_id,
        "goal": {
            "x": args.x,
            "y": args.y,
            "yaw": args.yaw,
        },
        "network_interface": interface,
        "started_unix_ns": time.time_ns(),
        "motion_disabled": True,
        "status": "error",
    }

    node = None
    action_client = None
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient

        rclpy.init(args=None)
        node = rclpy.create_node("robonix_compute_path_nomotion")
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
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout_s)
        if not result_future.done():
            raise RuntimeError("planner result timed out")
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("planner returned no result")
        evidence["action_status"] = int(wrapped.status)
        if int(wrapped.status) != int(GoalStatus.STATUS_SUCCEEDED):
            raise RuntimeError(f"planner action status was {wrapped.status}, not SUCCEEDED")
        evidence["path"] = summarize_path(wrapped.result.path)
        planning_time = wrapped.result.planning_time
        evidence["planning_time_s"] = round(
            int(planning_time.sec) + int(planning_time.nanosec) / 1_000_000_000.0,
            9,
        )
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
        print(
            "PASS: plan-only path "
            f"{evidence['path']['pose_count']} poses, "
            f"{evidence['path']['length_m']} m; evidence={output}"
        )
    else:
        print(f"FAIL: {evidence['error']}; evidence={output}")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
