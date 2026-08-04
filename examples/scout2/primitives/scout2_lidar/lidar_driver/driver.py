#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Scout2 lidar primitive — Capability-based driver.

Owns `robonix/primitive/lidar/*`. Subscribes to 2D LaserScan and 3D
Point cloud topics, exposing:

  primitive/lidar/lidar     topic_in   ROS 2 LaserScan stream
  primitive/lidar/lidar3d   topic_in   ROS 2 PointCloud2 stream
  primitive/lidar/snapshot  rpc        MCP one-shot LaserScan capture
  primitive/lidar/driver    rpc        gRPC lifecycle (Init waits for first scan)
"""
from __future__ import annotations

import os
import threading
import time

from robonix_api import Primitive, Ok, Err, Deferred

scout2_lidar = Primitive(id="scout2_lidar", namespace="robonix/primitive/lidar")

# ── shared state ────────────────────────────────────────────────────────────
state_lock = threading.Lock()
latest_scan = None    # latest rclpy LaserScan
latest_pc2 = None     # latest rclpy PointCloud2


def on_scan(msg):
    global latest_scan
    with state_lock:
        latest_scan = msg


def on_pointcloud(msg):
    global latest_pc2
    with state_lock:
        latest_pc2 = msg


# ── MCP snapshot tool (typed against codegen MCP dataclasses) ──────────────
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import LaserScan  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def ros_to_mcp(msg) -> LaserScan:
    h = msg.header
    stamp = builtin_interfaces_mcp.Time(sec=int(h.stamp.sec), nanosec=int(h.stamp.nanosec))
    header = std_msgs_mcp.Header(stamp=stamp, frame_id=str(h.frame_id))
    intensities = [float(x) for x in msg.intensities] if len(msg.intensities) else []
    return LaserScan(
        header=header,
        angle_min=float(msg.angle_min),
        angle_max=float(msg.angle_max),
        angle_increment=float(msg.angle_increment),
        time_increment=float(msg.time_increment),
        scan_time=float(msg.scan_time),
        range_min=float(msg.range_min),
        range_max=float(msg.range_max),
        ranges=[float(r) for r in msg.ranges],
        intensities=intensities,
    )


@scout2_lidar.mcp("robonix/primitive/lidar/snapshot")
def snapshot(msg: Empty) -> LaserScan:
    """Get the latest planar lidar scan. Returns sensor_msgs/LaserScan;
    `ranges[i]` is the distance (m) at angle `angle_min + i*angle_increment`.
    Useful for "obstacle in front?" / "where's the nearest open space?"
    Contract: robonix/primitive/lidar/snapshot."""
    _ = msg
    with state_lock:
        ros_scan = latest_scan
    if ros_scan is None:
        raise RuntimeError("no LaserScan received yet")
    return ros_to_mcp(ros_scan)


# ── lifecycle ────────────────────────────────────────────────────────────────
@scout2_lidar.on_init
def init(cfg):
    scan_topic = cfg.get("scan_topic") or os.environ.get("SCOUT2_SCAN_TOPIC", "/scan")
    pc2_topic = cfg.get("pointcloud_topic") or os.environ.get("SCOUT2_POINTCLOUD_TOPIC", "/velodyne_points")

    # Subscribe 2D scan
    scout2_lidar.create_subscription(
        "robonix/primitive/lidar/lidar",
        topic=scan_topic, msg_type="LaserScan",
        callback=on_scan, qos="best_effort", declare=False,
    )

    # Subscribe 3D point cloud
    scout2_lidar.create_subscription(
        "robonix/primitive/lidar/lidar3d",
        topic=pc2_topic, msg_type="PointCloud2",
        callback=on_pointcloud, qos="best_effort", declare=False,
    )

    # Gate INIT on first LaserScan arriving
    if not scout2_lidar.wait_for_topic(scan_topic, "LaserScan", float(cfg.get("sentinel_timeout_s", 15.0))):
        return Err(f"no LaserScan received on {scan_topic} within timeout")

    scout2_lidar.declare_ros2_topic("robonix/primitive/lidar/lidar", scan_topic, qos="best_effort")
    scout2_lidar.declare_ros2_topic("robonix/primitive/lidar/lidar3d", pc2_topic, qos="best_effort")
    return Ok()


@scout2_lidar.on_shutdown
def shutdown():
    return Ok()


if __name__ == "__main__":
    scout2_lidar.run()
