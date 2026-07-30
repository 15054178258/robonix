#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Publish one bounded Webots evaluation motion and finish with a stop."""

from __future__ import annotations

import argparse
import json
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--linear-x-mps", type=float, default=0.0)
    parser.add_argument("--angular-z-rps", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float, required=True)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    args = parser.parse_args()
    if args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    if args.rate_hz <= 0.0:
        parser.error("--rate-hz must be positive")

    import rclpy
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("webots_evaluation_twist_motion")
    publisher = node.create_publisher(Twist, args.topic, 10)
    command = Twist()
    command.linear.x = float(args.linear_x_mps)
    command.angular.z = float(args.angular_z_rps)
    period_s = 1.0 / float(args.rate_hz)
    started = time.monotonic()
    samples = 0
    try:
        while time.monotonic() - started < args.duration_s:
            publisher.publish(command)
            samples += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_s)
    finally:
        stop = Twist()
        # Publish more than once so a transient DDS discovery delay cannot
        # leave the last non-zero velocity latched at the controller.
        for _ in range(5):
            publisher.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_s)
        node.destroy_node()
        rclpy.shutdown()
    print(
        json.dumps(
            {
                "ok": True,
                "topic": args.topic,
                "linear_x_mps": args.linear_x_mps,
                "angular_z_rps": args.angular_z_rps,
                "duration_s": args.duration_s,
                "published_samples": samples,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
