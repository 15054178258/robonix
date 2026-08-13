#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""tf2 → /robonix/map/pose adapter.

rtabmap in mapping mode does NOT publish a standalone
PoseWithCovarianceStamped on /localization_pose (that topic is for
localization mode only — once a pre-built db is loaded). The
SLAM-corrected pose is exposed exclusively via the tf2 chain
``map → odom → base_link``.

Robonix's `service/map/pose` contract promises a topic-out stream
of ``PoseWithCovarianceStamped``, so consumers (scene's self-tracker,
nav, …) shouldn't have to touch tf2. This adapter bridges the two:
periodically look up ``map → base_link`` and publish the result on
``/robonix/map/pose``.

Single-purpose by design — nothing here parses lidars, camera images,
or anything else; if the lookup fails it just skips the tick.
"""
from __future__ import annotations

import math
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class TfToPose(Node):
    def __init__(self) -> None:
        super().__init__("tf_to_pose")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("topic", "/robonix/map/pose")

        self.map_frame: str = self.get_parameter("map_frame").get_parameter_value().string_value
        self.base_frame: str = self.get_parameter("base_frame").get_parameter_value().string_value
        rate: float = float(
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value or 10.0
        )
        topic: str = self.get_parameter("topic").get_parameter_value().string_value

        # Publishers: keep last sample latched-ish (DEPTH=1 + RELIABLE)
        # so a late subscriber gets pose without waiting a tick. Not
        # TRANSIENT_LOCAL — pose is ephemeral, "last 100ms ago" is the
        # interesting state, not "last whenever-the-publisher-started".
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(PoseWithCovarianceStamped, topic, qos)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Tick timer; we *don't* drive off /tf callbacks because there's
        # no single "this transform updated" event for a chain — easier
        # to poll the buffer at a fixed rate.
        period_s = max(0.01, 1.0 / max(0.1, rate))
        self._timer = self.create_timer(period_s, self._tick)

        # Diagnostic noise control: log lookup failures only when
        # they FIRST start happening or transition back to working.
        self._last_ok: bool | None = None

        self.get_logger().info(
            f"tf_to_pose up: {self.map_frame} → {self.base_frame} "
            f"@ {rate:.1f} Hz on {topic}"
        )

    def _tick(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as e:  # noqa: BLE001
            if self._last_ok is not False:
                self.get_logger().warn(
                    f"tf2 lookup {self.map_frame}→{self.base_frame} failed: {e}"
                )
                self._last_ok = False
            return
        if self._last_ok is not True:
            self.get_logger().info(
                f"tf2 lookup recovered ({self.map_frame}→{self.base_frame})"
            )
            self._last_ok = True

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.map_frame
        t = tf.transform.translation
        q = tf.transform.rotation
        msg.pose.pose.position.x = float(t.x)
        msg.pose.pose.position.y = float(t.y)
        msg.pose.pose.position.z = float(t.z)
        msg.pose.pose.orientation.x = float(q.x)
        msg.pose.pose.orientation.y = float(q.y)
        msg.pose.pose.orientation.z = float(q.z)
        msg.pose.pose.orientation.w = float(q.w)
        # No real covariance source for tf-derived pose; leave zeros.
        # Consumers that need uncertainty should pull it from rtabmap's
        # info topics, not from this adapter.
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TfToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
