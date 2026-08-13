#!/usr/bin/env python3
"""mapping_rbnx driver-shim: sensor_msgs/PointCloud2 → livox_ros_driver2/CustomMsg.

Exists because FASTLIO2 hard-depends on Livox CustomMsg (per-point offset_time,
line, reflectivity, tag). Any non-Livox 3D lidar (Isaac Sim, Velodyne, Ouster,
Hesai, etc.) producing standard PointCloud2 feeds FASTLIO2 through this adapter.

This keeps the Robonix `primitive/sensor/lidar3d` primitive at PointCloud2;
vendor-specific SLAM implementations carry their own shim internally.

Usage:
    python3 livox_adapter.py \
        --in-topic /front_3d_lidar/lidar_points \
        --out-topic /livox/lidar \
        --lidar-hz 10
"""
import argparse
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg, CustomPoint


_FIELD_SIZE = {
    PointField.INT8: 1, PointField.UINT8: 1,
    PointField.INT16: 2, PointField.UINT16: 2,
    PointField.INT32: 4, PointField.UINT32: 4,
    PointField.FLOAT32: 4, PointField.FLOAT64: 8,
}


def parse_cloud(msg: PointCloud2):
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return None

    point_step = msg.point_step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    count = msg.width * msg.height

    def extract(name, dtype=np.float32):
        f = fields.get(name)
        if f is None:
            return None
        size = _FIELD_SIZE[f.datatype]
        out = np.empty(count, dtype=dtype)
        for i in range(count):
            base = i * point_step + f.offset
            out[i] = np.frombuffer(data[base:base + size].tobytes(), dtype=dtype)[0]
        return out

    # fast path with structured dtype when layout is contiguous x,y,z float32
    try:
        x = data.view(dtype=np.uint8).reshape(count, point_step)
        xs = np.frombuffer(x[:, fields["x"].offset:fields["x"].offset + 4].tobytes(), dtype=np.float32)
        ys = np.frombuffer(x[:, fields["y"].offset:fields["y"].offset + 4].tobytes(), dtype=np.float32)
        zs = np.frombuffer(x[:, fields["z"].offset:fields["z"].offset + 4].tobytes(), dtype=np.float32)
        inten = None
        if "intensity" in fields:
            f = fields["intensity"]
            inten = np.frombuffer(x[:, f.offset:f.offset + 4].tobytes(), dtype=np.float32)
    except Exception:
        xs = extract("x")
        ys = extract("y")
        zs = extract("z")
        inten = extract("intensity") if "intensity" in fields else None

    return xs, ys, zs, inten


class LivoxAdapter(Node):
    def __init__(self, in_topic: str, out_topic: str, lidar_hz: float):
        super().__init__("livox_adapter")
        self._frame_ns = int(1e9 / lidar_hz)

        # Sub: match Isaac Sim ros2_bridge default (BEST_EFFORT for sensor streams)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        # Pub: RELIABLE so fastlio2 lio_node (default RELIABLE) can receive
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)

        self._sub = self.create_subscription(PointCloud2, in_topic, self._cb, sub_qos)
        self._pub = self.create_publisher(CustomMsg, out_topic, pub_qos)
        self._count = 0
        self.get_logger().info(f"adapter {in_topic} → {out_topic} (assume {lidar_hz}Hz lidar)")

    def _cb(self, msg: PointCloud2):
        parsed = parse_cloud(msg)
        if parsed is None:
            self.get_logger().warn("PointCloud2 missing x/y/z fields, dropping")
            return
        xs, ys, zs, inten = parsed

        n = xs.size
        finite = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        xs = xs[finite]; ys = ys[finite]; zs = zs[finite]
        if inten is not None:
            inten = inten[finite]
        n = xs.size
        if n == 0:
            return

        out = CustomMsg()
        out.header = msg.header
        out.timebase = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        out.point_num = n
        out.lidar_id = 0
        out.rsvd = [0, 0, 0]
        out.timestamp_type = 0

        # linear ramp offset_time across the frame duration
        offsets = np.linspace(0, self._frame_ns, n, endpoint=False, dtype=np.uint32)

        if inten is not None:
            refl = np.clip(inten, 0, 255).astype(np.uint8)
        else:
            refl = np.zeros(n, dtype=np.uint8)

        pts = []
        for i in range(n):
            cp = CustomPoint()
            cp.offset_time = int(offsets[i])
            cp.x = float(xs[i])
            cp.y = float(ys[i])
            cp.z = float(zs[i])
            cp.reflectivity = int(refl[i])
            cp.tag = 0
            cp.line = 0
            pts.append(cp)
        out.points = pts

        self._pub.publish(out)
        self._count += 1
        if self._count % 30 == 0:
            self.get_logger().info(f"published {self._count} frames, last n={n}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-topic", default="/front_3d_lidar/lidar_points")
    ap.add_argument("--out-topic", default="/livox/lidar")
    ap.add_argument("--lidar-hz", type=float, default=10.0,
                    help="assumed lidar publish rate; used only for per-point offset_time ramp")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = LivoxAdapter(args.in_topic, args.out_topic, args.lidar_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
