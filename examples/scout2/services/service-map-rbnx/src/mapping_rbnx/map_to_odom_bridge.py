#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Bridge RTAB-Map's private ICP trajectory to a navigation odom frame.

RTAB-Map owns ``map -> odom_icp`` and internal ICP publishes an Odometry
message describing ``odom_icp -> base_link`` without broadcasting that TF.
The chassis independently owns ``odom -> base_link``.  This node composes the
first two poses, removes the chassis pose at the same timestamp, and broadcasts
the sole navigation correction ``map -> odom``.

The node is opt-in.  Legacy mapping deployments never start it and retain the
existing RTAB-Map/odometry TF behaviour unchanged.
"""
from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from mapping_rbnx.odom_bridge_math import (
    Pose2,
    TimedPose2,
    compose,
    interpolate,
    inverse,
)


def pose_from_odometry(msg: Odometry) -> Pose2:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return Pose2(float(p.x), float(p.y), yaw)


def pose_from_transform(msg: TransformStamped) -> Pose2:
    p = msg.transform.translation
    q = msg.transform.rotation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return Pose2(float(p.x), float(p.y), yaw)


class MapToOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("map_to_odom_bridge")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("icp_odom_frame", "odom_icp")
        self.declare_parameter("nav_odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("icp_odom_topic", "/rtabmap/odom_icp")
        self.declare_parameter("nav_odom_topic", "/odom")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("odom_cache_s", 5.0)
        self.declare_parameter("max_sync_error_s", 0.10)
        self.declare_parameter("localization_timeout_s", 2.0)
        self.declare_parameter("reset_translation_m", 0.30)
        self.declare_parameter("reset_rotation_rad", math.radians(30.0))

        self.map_frame = self._string("map_frame")
        self.icp_odom_frame = self._string("icp_odom_frame")
        self.nav_odom_frame = self._string("nav_odom_frame")
        self.base_frame = self._string("base_frame")
        self.icp_odom_topic = self._string("icp_odom_topic")
        self.nav_odom_topic = self._string("nav_odom_topic")
        rate = self._double("publish_rate_hz")
        self.cache_ns = int(self._double("odom_cache_s") * 1e9)
        self.max_sync_ns = int(self._double("max_sync_error_s") * 1e9)
        self.localization_timeout_ns = int(
            self._double("localization_timeout_s") * 1e9
        )
        self.reset_translation_m = self._double("reset_translation_m")
        self.reset_rotation_rad = self._double("reset_rotation_rad")

        if self.icp_odom_frame == self.nav_odom_frame:
            raise RuntimeError(
                "icp_odom_frame and nav_odom_frame must differ; using the same "
                "frame would recreate the competing odom -> base_link TF"
            )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.create_subscription(
            Odometry, self.nav_odom_topic, self._on_nav_odom, qos
        )
        self.create_subscription(
            Odometry, self.icp_odom_topic, self._on_icp_odom, qos
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.nav_history: deque[TimedPose2] = deque()
        self.latest_icp: TimedPose2 | None = None
        self.last_correction: Pose2 | None = None
        self.last_processed_icp_ns = -1
        self.last_nav_pose: TimedPose2 | None = None
        self.wait_reason = ""
        self.timer = self.create_timer(max(0.01, 1.0 / max(0.1, rate)), self._tick)
        self.get_logger().info(
            "bridge active: %s->%s + %s => %s->%s (icp=%s chassis=%s)"
            % (
                self.map_frame,
                self.icp_odom_frame,
                self.icp_odom_frame,
                self.map_frame,
                self.nav_odom_frame,
                self.icp_odom_topic,
                self.nav_odom_topic,
            )
        )

    def _string(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    @staticmethod
    def _stamp_ns(msg: Odometry) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )

    def _valid_frames(self, msg: Odometry, parent: str, source: str) -> bool:
        if msg.header.frame_id != parent or msg.child_frame_id != self.base_frame:
            self._warn_once(
                f"{source} frames are {msg.header.frame_id!r}->{msg.child_frame_id!r}; "
                f"expected {parent!r}->{self.base_frame!r}"
            )
            return False
        return True

    def _on_nav_odom(self, msg: Odometry) -> None:
        if not self._valid_frames(msg, self.nav_odom_frame, "navigation odom"):
            return
        sample = TimedPose2(self._stamp_ns(msg), pose_from_odometry(msg))
        previous = self.last_nav_pose
        if previous is not None and sample.stamp_ns > previous.stamp_ns:
            delta = compose(inverse(previous.pose), sample.pose)
            if (
                math.hypot(delta.x, delta.y) > self.reset_translation_m
                or abs(delta.yaw) > self.reset_rotation_rad
            ):
                self.nav_history.clear()
                self.last_correction = None
                self.last_processed_icp_ns = -1
                self.get_logger().error(
                    "navigation odom reset/jump detected; waiting for a new "
                    "time-aligned RTAB-Map correction"
                )
        self.last_nav_pose = sample
        if self.nav_history and sample.stamp_ns <= self.nav_history[-1].stamp_ns:
            return
        self.nav_history.append(sample)
        cutoff = sample.stamp_ns - self.cache_ns
        while len(self.nav_history) > 2 and self.nav_history[1].stamp_ns < cutoff:
            self.nav_history.popleft()

    def _on_icp_odom(self, msg: Odometry) -> None:
        if not self._valid_frames(msg, self.icp_odom_frame, "ICP odom"):
            return
        self.latest_icp = TimedPose2(self._stamp_ns(msg), pose_from_odometry(msg))

    def _nav_pose_at(self, stamp_ns: int) -> Pose2 | None:
        if not self.nav_history:
            return None
        before: TimedPose2 | None = None
        after: TimedPose2 | None = None
        for sample in reversed(self.nav_history):
            if sample.stamp_ns <= stamp_ns:
                before = sample
                break
            after = sample
        if before is not None and after is not None:
            if max(stamp_ns - before.stamp_ns, after.stamp_ns - stamp_ns) <= self.max_sync_ns:
                return interpolate(before, after, stamp_ns)
        nearest = min(
            self.nav_history,
            key=lambda sample: abs(sample.stamp_ns - stamp_ns),
        )
        if abs(nearest.stamp_ns - stamp_ns) <= self.max_sync_ns:
            return nearest.pose
        return None

    def _update_correction(self) -> None:
        icp = self.latest_icp
        if icp is None or icp.stamp_ns == self.last_processed_icp_ns:
            return
        chassis = self._nav_pose_at(icp.stamp_ns)
        if chassis is None:
            self._warn_once("no chassis odom sample close enough to the ICP timestamp")
            return
        try:
            map_to_icp_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.icp_odom_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception as exc:  # noqa: BLE001
            self._warn_once(f"waiting for RTAB-Map map->odom_icp: {exc}")
            return
        map_to_base = compose(pose_from_transform(map_to_icp_msg), icp.pose)
        self.last_correction = compose(map_to_base, inverse(chassis))
        self.last_processed_icp_ns = icp.stamp_ns
        if self.wait_reason:
            self.get_logger().info("map-to-odom bridge inputs recovered")
            self.wait_reason = ""

    def _warn_once(self, reason: str) -> None:
        if reason != self.wait_reason:
            self.get_logger().warn(reason)
            self.wait_reason = reason

    def _tick(self) -> None:
        self._update_correction()
        correction = self.last_correction
        icp = self.latest_icp
        if correction is None or icp is None:
            return
        age_ns = self.get_clock().now().nanoseconds - icp.stamp_ns
        if age_ns > self.localization_timeout_ns:
            self._warn_once(
                f"ICP/RTAB-Map correction is stale ({age_ns / 1e9:.2f}s); "
                "holding the last map->odom correction"
            )
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.nav_odom_frame
        msg.transform.translation.x = correction.x
        msg.transform.translation.y = correction.y
        msg.transform.translation.z = 0.0
        half = 0.5 * correction.yaw
        msg.transform.rotation.z = math.sin(half)
        msg.transform.rotation.w = math.cos(half)
        self.tf_broadcaster.sendTransform(msg)


def main() -> None:
    rclpy.init()
    node = MapToOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
