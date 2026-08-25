#!/usr/bin/env python3
"""Subscription-only /cmd_vel evidence sink for isolated Gate 2 replay.

It deliberately exposes no publisher, client, service, socket, SDK import or
forwarding callback.  Receiving a Twist only appends a bounded measurement to
the JSON evidence file.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any


def _write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description="Gate 2 no-op Twist subscriber")
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.topic != "/cmd_vel":
        parser.error("the Gate 2 sink is fixed to /cmd_vel")

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        _write(
            args.output,
            {
                "schema_version": 1,
                "topic": args.topic,
                "subscribes_only": True,
                "forwarded": 0,
                "samples": [],
                "error": f"ROS import failed: {exc}",
            },
        )
        return 2

    try:
        rclpy.init(args=None)
        node = Node(
            "gate2_noop_cmd_vel_sink",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
    except Exception as exc:
        _write(
            args.output,
            {
                "schema_version": 1,
                "topic": args.topic,
                "subscribes_only": True,
                "forwarded": 0,
                "samples": [],
                "invalid_samples": 0,
                "error": f"ROS node creation failed: {exc}",
            },
        )
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        return 2
    samples: list[dict[str, float]] = []
    first_sim_ns: int | None = None
    invalid_samples = 0

    def on_twist(message: Twist) -> None:
        nonlocal first_sim_ns, invalid_samples
        now_ns = int(node.get_clock().now().nanoseconds)
        if now_ns > 0 and first_sim_ns is None:
            first_sim_ns = now_ns
        relative = (now_ns - first_sim_ns) / 1_000_000_000.0 if first_sim_ns is not None else -1.0
        values = (float(message.linear.x), float(message.linear.y), float(message.angular.z))
        if not all(math.isfinite(value) for value in values):
            invalid_samples += 1
            return
        if len(samples) < 20_000:
            samples.append(
                {
                    "t": round(relative, 6),
                    "x": round(values[0], 8),
                    "y": round(values[1], 8),
                    "z": round(values[2], 8),
                }
            )

    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    subscription = node.create_subscription(Twist, args.topic, on_twist, qos)
    del subscription  # the node retains ownership; no other ROS entity is created
    exit_code = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        exit_code = 2
        error = str(exc)
    else:
        error = ""
    finally:
        evidence = {
            "schema_version": 1,
            "topic": args.topic,
            "subscribes_only": True,
            "forwarded": 0,
            "received": len(samples),
            "invalid_samples": invalid_samples,
            "samples": samples,
            "error": error if "error" in locals() else "",
        }
        _write(args.output, evidence)
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
