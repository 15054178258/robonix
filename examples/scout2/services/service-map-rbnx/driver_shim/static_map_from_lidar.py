#!/usr/bin/env python3
"""One-shot 2D OccupancyGrid from a single lidar frame.

Ground-truth odom + static map stand-in for live SLAM when the target sensor
doesn't meet the SLAM backend's assumptions (e.g. Isaac Sim RTX lidar has no
per-point timestamps, breaks FASTLIO2 undistortion).

Accumulates one or a few lidar frames while robot is static, projects to XY,
builds an inflated OccupancyGrid, then publishes it latched forever.
"""
import argparse

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


class MapBuilder(Node):
    def __init__(self, in_topic: str, out_topic: str, res: float, accumulate_frames: int,
                 half_size_m: float, z_min: float, z_max: float, inflate_cells: int):
        super().__init__("static_map_from_lidar")
        self._res = res
        self._half = half_size_m
        self._z_min = z_min
        self._z_max = z_max
        self._inflate = inflate_cells
        self._needed = accumulate_frames
        self._buf: list[np.ndarray] = []

        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_odom = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._have_odom = False

        self._sub = self.create_subscription(PointCloud2, in_topic, self._cb, qos_sub)
        self._odom_sub = self.create_subscription(Odometry, "/chassis/odom", self._on_odom, qos_odom)
        self._pub = self.create_publisher(OccupancyGrid, out_topic, qos_pub)
        self.get_logger().info(
            f"accumulating {self._needed} lidar frames from {in_topic}; "
            f"will publish latched grid on {out_topic}"
        )

    def _on_odom(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._have_odom = True

    def _cb(self, msg: PointCloud2):
        if len(self._buf) >= self._needed:
            return
        if not self._have_odom:
            return  # wait for robot pose before accumulating
        pts = parse_xyz(msg)
        self.get_logger().info(
            f"frame {len(self._buf) + 1}/{self._needed}: {len(pts)} pts, "
            f"robot=({self._robot_x:.2f}, {self._robot_y:.2f})"
        )
        self._buf.append(pts)
        if len(self._buf) >= self._needed:
            self._publish_grid(msg.header.frame_id)

    def _publish_grid(self, frame_id: str):
        pts = np.concatenate(self._buf, axis=0)
        z_mask = (pts[:, 2] > self._z_min) & (pts[:, 2] < self._z_max)
        pts = pts[z_mask]

        res = self._res
        w = int(2 * self._half / res)
        h = w

        origin_x = -self._half
        origin_y = -self._half
        ix = np.floor((pts[:, 0] - origin_x) / res).astype(np.int32)
        iy = np.floor((pts[:, 1] - origin_y) / res).astype(np.int32)
        valid = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        ix = ix[valid]; iy = iy[valid]

        grid = np.zeros((h, w), dtype=np.int8)  # 0 = free
        grid[iy, ix] = 100  # 100 = occupied

        if self._inflate > 0:
            from scipy.ndimage import binary_dilation
            occ = grid == 100
            struct = np.ones((3, 3), dtype=bool)
            for _ in range(self._inflate):
                occ = binary_dilation(occ, structure=struct)
            grid[occ] = 100

        # Clear a generous free circle around the robot's actual pose so the
        # planner's A* start cell is guaranteed free. Isaac lidar sees the
        # robot's own chassis as obstacles otherwise.
        cx_cell = int((self._robot_x - origin_x) / res)
        cy_cell = int((self._robot_y - origin_y) / res)
        clear_radius_cells = int(1.5 / res)  # 1.5m free around robot
        yy, xx = np.ogrid[:h, :w]
        mask = (xx - cx_cell) ** 2 + (yy - cy_cell) ** 2 <= clear_radius_cells ** 2
        grid[mask] = 0
        self.get_logger().info(
            f"cleared {int(mask.sum())} cells within {1.5}m of robot"
        )

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = res
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()

        self._pub.publish(msg)
        self.get_logger().info(
            f"published OccupancyGrid {w}x{h} @ {res}m, occupied cells={int((grid == 100).sum())}"
        )

        timer_ns = int(1e9)
        self.create_timer(timer_ns / 1e9, lambda: self._pub.publish(msg))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-topic", default="/front_3d_lidar/lidar_points")
    ap.add_argument("--out-topic", default="/robonix/map/occupancy_grid")
    ap.add_argument("--resolution", type=float, default=0.1)
    ap.add_argument("--half-size", type=float, default=30.0,
                    help="half-side of the square map in meters")
    ap.add_argument("--z-min", type=float, default=0.1)
    ap.add_argument("--z-max", type=float, default=2.0)
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--inflate", type=int, default=2,
                    help="dilation passes; each pass ≈ resolution meters of inflation")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = MapBuilder(args.in_topic, args.out_topic, args.resolution,
                      args.frames, args.half_size, args.z_min, args.z_max, args.inflate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
