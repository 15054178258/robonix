#!/usr/bin/env python3
"""Measure Gate 2 replay and exercise Nav2 goal cancellation.

The node subscribes to replay/generated telemetry and uses only the isolated
Nav2 NavigateToPose action interface.  It has no Twist publisher and no
Unitree transport surface.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

import yaml


def _write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _load(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("scenario must be an object")
    return value


def _occupied_in_region(message: Any, region: dict[str, Any], threshold: int) -> int:
    if str(message.header.frame_id) != str(region["frame_id"]):
        return -1
    width = int(message.info.width)
    height = int(message.info.height)
    resolution = float(message.info.resolution)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return -1
    origin = message.info.origin
    q = origin.orientation
    yaw = math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    radius_squared = float(region["radius"]) ** 2
    target_x = float(region["x"])
    target_y = float(region["y"])
    occupied = 0
    data = message.data
    for index, value in enumerate(data):
        if int(value) < threshold:
            continue
        row, column = divmod(index, width)
        if row >= height:
            break
        local_x = (column + 0.5) * resolution
        local_y = (row + 0.5) * resolution
        world_x = float(origin.position.x) + cosine * local_x - sine * local_y
        world_y = float(origin.position.y) + sine * local_x + cosine * local_y
        if (world_x - target_x) ** 2 + (world_y - target_y) ** 2 <= radius_squared:
            occupied += 1
    return occupied


def main() -> int:
    parser = argparse.ArgumentParser(description="Gate 2 ROS replay observer")
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    scenario = _load(args.scenario)

    try:
        import rclpy
        from action_msgs.msg import GoalStatus, GoalStatusArray
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.action import ActionClient
        from rclpy.duration import Duration
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from rclpy.time import Time
        from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2
        from tf2_ros import Buffer, TransformListener
    except ImportError as exc:
        _write(args.output, {"schema_version": 1, "error": f"ROS import failed: {exc}"})
        return 2

    topics = scenario["topics"]
    frames = scenario["frames"]
    counts = {key: 0 for key in ("map", "odom", "cloud", "scan", "imu", "camera", "nav_status")}
    scan_times: list[float] = []
    costmaps: dict[str, list[dict[str, Any]]] = {"local": [], "global": []}
    tf_state = {"map_to_odom": False, "odom_to_base": False}
    navigation: dict[str, Any] = {
        "goal_sent": False,
        "goal_accepted": False,
        "cancel_requested": False,
        "cancel_accepted": False,
        "result": "",
    }
    errors: list[str] = []
    first_sim_ns: int | None = None
    goal_handle: Any = None
    send_future: Any = None
    cancel_future: Any = None
    result_future: Any = None

    try:
        rclpy.init(args=None)
        node = Node(
            "gate2_replay_observer",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
    except Exception as exc:
        _write(
            args.output,
            {"schema_version": 1, "error": f"ROS node creation failed: {exc}"},
        )
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        return 2
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
    action_client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    def relative_time() -> float:
        nonlocal first_sim_ns
        now_ns = int(node.get_clock().now().nanoseconds)
        if now_ns > 0 and first_sim_ns is None:
            first_sim_ns = now_ns
        return (now_ns - first_sim_ns) / 1_000_000_000.0 if first_sim_ns is not None else -1.0

    def count(name: str):
        def callback(_: Any) -> None:
            counts[name] += 1
        return callback

    def on_scan(_: LaserScan) -> None:
        counts["scan"] += 1
        current = relative_time()
        if current >= 0.0 and len(scan_times) < 20_000:
            scan_times.append(round(current, 6))

    def on_status(_: GoalStatusArray) -> None:
        counts["nav_status"] += 1

    def on_costmap(name: str):
        def callback(message: OccupancyGrid) -> None:
            current = relative_time()
            occupied = _occupied_in_region(
                message,
                scenario["costmaps"][name],
                int(scenario["costmaps"]["lethal_threshold"]),
            )
            if current >= 0.0 and len(costmaps[name]) < 10_000:
                costmaps[name].append(
                    {
                        "t": round(current, 6),
                        "occupied_cells": occupied,
                        "frame_id": str(message.header.frame_id),
                    }
                )
        return callback

    sensor_qos = qos_profile_sensor_data
    transient_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    status_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscriptions = [
        node.create_subscription(OccupancyGrid, topics["map"], count("map"), transient_qos),
        node.create_subscription(Odometry, topics["odom"], count("odom"), sensor_qos),
        node.create_subscription(PointCloud2, topics["cloud"], count("cloud"), sensor_qos),
        node.create_subscription(LaserScan, topics["scan"], on_scan, sensor_qos),
        node.create_subscription(Imu, topics["imu"], count("imu"), sensor_qos),
        node.create_subscription(Image, topics["camera"], count("camera"), sensor_qos),
        node.create_subscription(GoalStatusArray, topics["nav_status"], on_status, status_qos),
        node.create_subscription(OccupancyGrid, topics["local_costmap"], on_costmap("local"), transient_qos),
        node.create_subscription(OccupancyGrid, topics["global_costmap"], on_costmap("global"), transient_qos),
    ]

    status_labels = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }

    def on_result(future: Any) -> None:
        try:
            response = future.result()
            navigation["result"] = status_labels.get(int(response.status), str(response.status))
        except Exception as exc:
            errors.append(f"NavigateToPose result failed: {exc}")

    def on_goal(future: Any) -> None:
        nonlocal goal_handle, result_future
        try:
            goal_handle = future.result()
            navigation["goal_accepted"] = bool(goal_handle.accepted)
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(on_result)
            else:
                navigation["result"] = "REJECTED"
        except Exception as exc:
            errors.append(f"NavigateToPose goal response failed: {exc}")

    def on_cancel(future: Any) -> None:
        try:
            response = future.result()
            navigation["cancel_accepted"] = bool(response.goals_canceling)
        except Exception as exc:
            errors.append(f"NavigateToPose cancel response failed: {exc}")

    def timer_callback() -> None:
        nonlocal send_future, cancel_future
        current = relative_time()
        if current < 0.0:
            return
        try:
            if tf_buffer.can_transform(frames["map"], frames["odom"], Time(), timeout=Duration(seconds=0.02)):
                tf_state["map_to_odom"] = True
            if tf_buffer.can_transform(frames["odom"], frames["base"], Time(), timeout=Duration(seconds=0.02)):
                tf_state["odom_to_base"] = True
        except Exception:
            pass
        if not navigation["goal_sent"] and current >= float(scenario["goal"]["dispatch_at_s"]):
            if action_client.server_is_ready():
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = frames["map"]
                goal.pose.header.stamp = node.get_clock().now().to_msg()
                goal.pose.pose.position.x = float(scenario["goal"]["x"])
                goal.pose.pose.position.y = float(scenario["goal"]["y"])
                yaw = float(scenario["goal"]["yaw"])
                goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
                goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
                navigation["goal_sent"] = True
                navigation["goal_sent_at_s"] = round(current, 6)
                send_future = action_client.send_goal_async(goal)
                send_future.add_done_callback(on_goal)
        if (
            navigation["goal_accepted"]
            and not navigation["cancel_requested"]
            and current >= float(scenario["goal"]["cancel_at_s"])
            and goal_handle is not None
        ):
            navigation["cancel_requested"] = True
            navigation["cancel_requested_at_s"] = round(current, 6)
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(on_cancel)

    timer = node.create_timer(0.1, timer_callback)
    exit_code = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        exit_code = 2
        errors.append(f"observer runtime failed: {exc}")
    finally:
        evidence = {
            "schema_version": 1,
            "counts": counts,
            "scan_times": scan_times,
            "costmaps": costmaps,
            "tf": tf_state,
            "navigation": navigation,
            "errors": errors,
        }
        _write(args.output, evidence)
        try:
            action_client.destroy()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
