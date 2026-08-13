#!/usr/bin/env python3
"""Live occupancy-grid builder using ground-truth odom + lidar accumulation.

Replaces static_map_from_lidar when live mapping is desired but fighting a
vendor-specific SLAM backend (FASTLIO2 requires Livox per-point timestamps)
isn't worth the time for a simulator-based evaluation.

Since Isaac Sim publishes perfect odometry on `/chassis/odom`, we can:
  1. Buffer latest pose from odom.
  2. On each lidar frame, rotate+translate points to world frame using pose.
  3. Bresenham-ray each hit from sensor to the endpoint, marking free / occupied.
  4. Republish the accumulated grid at a fixed rate.

The result is SLAM-quality map without running a SLAM (because pose is ground
truth). In real deployment you'd swap `/chassis/odom` for a SLAM frontend.

Interface abstraction reasoning:
  Robonix primitive/sensor/lidar3d  = PointCloud2
  Robonix primitive/base/odom       = nav_msgs/Odometry
  Robonix srv/common/map/occupancy_grid = nav_msgs/OccupancyGrid
  → this node is a `lidar3d × odom → occupancy_grid` user-service implementation.
"""
import argparse
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry


def parse_xyz(msg: PointCloud2) -> np.ndarray:
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    n = msg.width * msg.height
    arr = data.reshape(n, msg.point_step)
    xs = np.frombuffer(arr[:, fields["x"].offset:fields["x"].offset + 4].tobytes(), dtype=np.float32)
    ys = np.frombuffer(arr[:, fields["y"].offset:fields["y"].offset + 4].tobytes(), dtype=np.float32)
    zs = np.frombuffer(arr[:, fields["z"].offset:fields["z"].offset + 4].tobytes(), dtype=np.float32)
    finite = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
    return np.column_stack([xs[finite], ys[finite], zs[finite]]).astype(np.float32)


def quat_to_mat(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
    ], dtype=np.float32)


class LiveMapBuilder(Node):
    LOG_ODDS_HIT = 0.8
    LOG_ODDS_MISS = -0.4
    LOG_ODDS_MIN = -4.0
    LOG_ODDS_MAX = 4.0
    OCC_THRESH = 0.7  # p(occ) above this → 100
    FREE_THRESH = 0.3  # below this → 0, between → -1 (unknown)

    def __init__(
        self,
        lidar_topic: str,
        odom_topic: str,
        out_topic: str,
        res: float,
        half_size: float,
        z_min: float,
        z_max: float,
        publish_hz: float,
        max_range: float,
    ) -> None:
        super().__init__("live_map_builder")
        self._res = res
        self._half = half_size
        self._z_min = z_min
        self._z_max = z_max
        self._max_range = max_range

        self._w = int(2 * half_size / res)
        self._h = self._w
        self._origin_x = -half_size
        self._origin_y = -half_size

        self._log_odds = np.zeros((self._h, self._w), dtype=np.float32)

        self._lock = threading.Lock()
        self._pose: tuple[float, float, float, np.ndarray] | None = None  # (x, y, z, R)

        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_odom = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_odom)
        self.create_subscription(PointCloud2, lidar_topic, self._on_scan, qos_sub)
        self._pub = self.create_publisher(OccupancyGrid, out_topic, qos_pub)

        self.create_timer(1.0 / publish_hz, self._publish)

        self._scans_processed = 0
        self.get_logger().info(
            f"live map builder: lidar={lidar_topic} odom={odom_topic} "
            f"out={out_topic} grid={self._w}×{self._h} @ {res}m "
            f"pub={publish_hz}Hz max_range={max_range}m"
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        q = p.orientation
        R = quat_to_mat(q.x, q.y, q.z, q.w)
        with self._lock:
            self._pose = (p.position.x, p.position.y, p.position.z, R)

    def _on_scan(self, msg: PointCloud2) -> None:
        with self._lock:
            pose = self._pose
        if pose is None:
            return
        px, py, pz, R = pose

        pts = parse_xyz(msg)
        if pts.size == 0:
            return

        # filter by z band relative to sensor BEFORE rotation (crude but fast)
        # keep points whose distance is within max_range
        rng = np.linalg.norm(pts, axis=1)
        keep = (rng < self._max_range) & (rng > 0.3)
        pts = pts[keep]
        if pts.size == 0:
            return

        # rotate to world, translate by robot pose
        world = (R @ pts.T).T + np.array([px, py, pz], dtype=np.float32)

        # z filter in world frame
        zmask = (world[:, 2] > self._z_min) & (world[:, 2] < self._z_max)
        world = world[zmask]
        if world.size == 0:
            return

        res = self._res
        cx = int((px - self._origin_x) / res)
        cy = int((py - self._origin_y) / res)
        if not (0 <= cx < self._w and 0 <= cy < self._h):
            return

        hit_x = np.floor((world[:, 0] - self._origin_x) / res).astype(np.int32)
        hit_y = np.floor((world[:, 1] - self._origin_y) / res).astype(np.int32)

        valid = (hit_x >= 0) & (hit_x < self._w) & (hit_y >= 0) & (hit_y < self._h)
        hit_x = hit_x[valid]
        hit_y = hit_y[valid]

        self._update_free_line_vectorized(cx, cy, hit_x, hit_y)
        self._log_odds[hit_y, hit_x] = np.clip(
            self._log_odds[hit_y, hit_x] + self.LOG_ODDS_HIT,
            self.LOG_ODDS_MIN, self.LOG_ODDS_MAX,
        )

        self._scans_processed += 1

    def _update_free_line_vectorized(
        self, cx: int, cy: int, hit_x: np.ndarray, hit_y: np.ndarray,
    ) -> None:
        """Mark cells along the ray from (cx,cy) to each (hit_x,hit_y) as free.

        Simple DDA subsample: sample along the line at resolution pacing, drop
        the final cell (that's the hit), apply LOG_ODDS_MISS. Not perfectly
        Bresenham but good enough for grid-resolution rays.
        """
        dx = hit_x - cx
        dy = hit_y - cy
        dist = np.hypot(dx, dy).astype(np.int32)
        # Aggregate into a single (n_samples,) array of cells to mark free,
        # one sample per cell along each ray.
        all_xs = []
        all_ys = []
        max_len = int(dist.max()) if dist.size > 0 else 0
        for step in range(1, max_len):
            active = dist > step
            if not active.any():
                break
            frac = step / dist[active]
            ix = cx + (dx[active] * frac).astype(np.int32)
            iy = cy + (dy[active] * frac).astype(np.int32)
            all_xs.append(ix)
            all_ys.append(iy)
        if all_xs:
            xs = np.concatenate(all_xs)
            ys = np.concatenate(all_ys)
            mask = (xs >= 0) & (xs < self._w) & (ys >= 0) & (ys < self._h)
            xs = xs[mask]
            ys = ys[mask]
            # free-space update is soft; accumulates only for cells not already occupied
            self._log_odds[ys, xs] = np.clip(
                self._log_odds[ys, xs] + self.LOG_ODDS_MISS,
                self.LOG_ODDS_MIN, self.LOG_ODDS_MAX,
            )

    def _publish(self) -> None:
        with self._lock:
            pose = self._pose
        prob = 1.0 - 1.0 / (1.0 + np.exp(self._log_odds))
        grid = np.full((self._h, self._w), -1, dtype=np.int8)  # unknown
        grid[prob < self.FREE_THRESH] = 0
        grid[prob > self.OCC_THRESH] = 100

        if pose is not None:
            px, py, _, _ = pose
            cx = int((px - self._origin_x) / self._res)
            cy = int((py - self._origin_y) / self._res)
            if 0 <= cx < self._w and 0 <= cy < self._h:
                r = int(0.35 / self._res)
                yy, xx = np.ogrid[:self._h, :self._w]
                disk = (xx - cx) ** 2 + (yy - cy) ** 2 <= r * r
                grid[disk] = 0

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self._res
        msg.info.width = self._w
        msg.info.height = self._h
        msg.info.origin.position.x = self._origin_x
        msg.info.origin.position.y = self._origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self._pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lidar-topic", default="/front_3d_lidar/lidar_points")
    ap.add_argument("--odom-topic", default="/chassis/odom")
    ap.add_argument("--out-topic", default="/robonix/map/occupancy_grid")
    ap.add_argument("--resolution", type=float, default=0.1)
    ap.add_argument("--half-size", type=float, default=30.0)
    ap.add_argument("--z-min", type=float, default=0.15)
    ap.add_argument("--z-max", type=float, default=2.0)
    ap.add_argument("--publish-hz", type=float, default=2.0)
    ap.add_argument("--max-range", type=float, default=25.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = LiveMapBuilder(
        lidar_topic=args.lidar_topic,
        odom_topic=args.odom_topic,
        out_topic=args.out_topic,
        res=args.resolution,
        half_size=args.half_size,
        z_min=args.z_min,
        z_max=args.z_max,
        publish_hz=args.publish_hz,
        max_range=args.max_range,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
