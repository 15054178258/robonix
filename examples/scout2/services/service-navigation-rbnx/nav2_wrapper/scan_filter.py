"""Remove isolated PointCloud2-to-LaserScan speckles without temporal lag."""

from __future__ import annotations

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from nav2_wrapper.scan_filter_core import filter_ranges


class ScanSpeckleFilter(Node):
    def __init__(self):
        super().__init__("robonix_scan_speckle_filter")
        self.window_bins = int(os.getenv("ROBONIX_SCAN_FILTER_WINDOW_BINS", "2"))
        self.max_delta = float(os.getenv("ROBONIX_SCAN_FILTER_MAX_DELTA_M", "0.20"))
        self.min_neighbors = int(os.getenv("ROBONIX_SCAN_FILTER_MIN_NEIGHBORS", "2"))
        self._last_log = 0.0
        self._pub = self.create_publisher(LaserScan, "/scanner/scan", 10)
        self.create_subscription(
            LaserScan, "/scanner/scan_raw", self._on_scan, qos_profile_sensor_data
        )
        self.get_logger().info(
            "filter active: window=%d bins delta=%.2fm min_neighbors=%d"
            % (self.window_bins, self.max_delta, self.min_neighbors)
        )

    def _on_scan(self, msg: LaserScan) -> None:
        filtered, removed = filter_ranges(
            list(msg.ranges), msg.range_min, msg.range_max,
            self.window_bins, self.max_delta, self.min_neighbors,
        )
        output = LaserScan()
        output.header = msg.header
        output.angle_min = msg.angle_min
        output.angle_max = msg.angle_max
        output.angle_increment = msg.angle_increment
        output.time_increment = msg.time_increment
        output.scan_time = msg.scan_time
        output.range_min = msg.range_min
        output.range_max = msg.range_max
        output.ranges = filtered
        output.intensities = list(msg.intensities)
        self._pub.publish(output)
        now = time.monotonic()
        if now - self._last_log >= 1.0:
            closest = min((msg.ranges[i] for i in removed), default=math.inf)
            self.get_logger().info(
                "scan bins=%d removed=%d closest_removed=%s"
                % (len(msg.ranges), len(removed),
                   "none" if not math.isfinite(closest) else f"{closest:.3f}m")
            )
            self._last_log = now


def main() -> None:
    rclpy.init()
    node = ScanSpeckleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
