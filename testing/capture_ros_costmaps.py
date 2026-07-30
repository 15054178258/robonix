#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Capture Nav2 occupancy-grid costmaps for the Webots review report.

Run this inside any ROS environment that can see the Nav2 graph.  The output
is a compact, self-contained JSON snapshot; signed OccupancyGrid cells are
stored as zlib-compressed bytes so a full global costmap does not inflate the
review artifact.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import time
import zlib
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def _yaw(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _transform_origin(
    message: OccupancyGrid,
    *,
    target_frame: str,
    tf_buffer: Buffer,
) -> tuple[float, float, float]:
    origin = message.info.origin
    source_frame = str(message.header.frame_id or target_frame)
    origin_yaw = _yaw(origin.orientation)
    if source_frame == target_frame:
        return (
            float(origin.position.x),
            float(origin.position.y),
            origin_yaw,
        )
    transform = tf_buffer.lookup_transform(
        target_frame,
        source_frame,
        Time.from_msg(message.header.stamp),
        timeout=Duration(seconds=1.0),
    ).transform
    transform_yaw = _yaw(transform.rotation)
    cosine = math.cos(transform_yaw)
    sine = math.sin(transform_yaw)
    return (
        float(transform.translation.x)
        + cosine * float(origin.position.x)
        - sine * float(origin.position.y),
        float(transform.translation.y)
        + sine * float(origin.position.x)
        + cosine * float(origin.position.y),
        transform_yaw + origin_yaw,
    )


def _snapshot(
    message: OccupancyGrid,
    *,
    target_frame: str,
    tf_buffer: Buffer,
) -> dict[str, Any]:
    width = int(message.info.width)
    height = int(message.info.height)
    if width <= 0 or height <= 0 or len(message.data) != width * height:
        raise ValueError(
            f"invalid OccupancyGrid dimensions {width}x{height} "
            f"for {len(message.data)} cells"
        )
    origin_x, origin_y, origin_yaw = _transform_origin(
        message,
        target_frame=target_frame,
        tf_buffer=tf_buffer,
    )
    raw = bytes(int(value) & 0xFF for value in message.data)
    return {
        "frame_id": target_frame,
        "source_frame_id": str(message.header.frame_id),
        "stamp": {
            "sec": int(message.header.stamp.sec),
            "nanosec": int(message.header.stamp.nanosec),
        },
        "width": width,
        "height": height,
        "resolution": float(message.info.resolution),
        "origin_x": origin_x,
        "origin_y": origin_y,
        "origin_yaw": origin_yaw,
        "encoding": "ros-occupancy-int8-zlib-base64",
        "data": base64.b64encode(zlib.compress(raw, level=9)).decode("ascii"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--target-frame", default="map")
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        metavar="NAME=TOPIC",
        help=(
            "costmap to capture; repeat as needed "
            "(default: global and local Nav2 costmaps)"
        ),
    )
    args = parser.parse_args()
    topics = args.topic or [
        "global=/global_costmap/costmap",
        "local=/local_costmap/costmap",
    ]
    resolved_topics: dict[str, str] = {}
    for raw in topics:
        name, separator, topic = raw.partition("=")
        if not separator or not name or not topic:
            parser.error(f"invalid --topic {raw!r}; expected NAME=TOPIC")
        resolved_topics[name] = topic

    rclpy.init()
    node = rclpy.create_node("robonix_costmap_snapshot")
    tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
    _listener = TransformListener(tf_buffer, node, spin_thread=False)
    messages: dict[str, OccupancyGrid] = {}
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    subscriptions = []
    for name, topic in resolved_topics.items():
        subscriptions.append(
            node.create_subscription(
                OccupancyGrid,
                topic,
                lambda message, key=name: messages.__setitem__(key, message),
                qos,
            )
        )

    deadline = time.monotonic() + args.timeout_s
    while len(messages) < len(resolved_topics) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = sorted(set(resolved_topics) - set(messages))
    if missing:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"timed out waiting for costmap(s): {', '.join(missing)}")

    payload = {
        "schema": "robonix.webots.costmaps.v1",
        "target_frame": args.target_frame,
        "layers": {
            name: _snapshot(
                message,
                target_frame=args.target_frame,
                tf_buffer=tf_buffer,
            )
            for name, message in messages.items()
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    node.destroy_node()
    rclpy.shutdown()
    print(
        json.dumps(
            {
                name: {
                    "topic": resolved_topics[name],
                    "width": layer["width"],
                    "height": layer["height"],
                    "resolution": layer["resolution"],
                    "frame_id": layer["frame_id"],
                }
                for name, layer in payload["layers"].items()
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
