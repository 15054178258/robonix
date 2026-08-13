#!/usr/bin/env python3
# Live 2D LaserScan from Livox CustomMsg — for Nav2 obstacle_layer.
# Subscribes: /scanner/cloud (livox_ros_driver2/CustomMsg, live unregistered)
# Publishes:  /robonix/map/scan_2d (sensor_msgs/LaserScan) at LiDAR rate (~10Hz).
#
# Height-slice around LiDAR mounting (body frame) and polar-bin to min range per angle.
# Unlike pointcloud_to_grid (static map, log-odds accumulated), this is a per-frame
# live obstacle signal — a person walking in front appears immediately and disappears
# the moment they leave the LiDAR FOV.
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

try:
    from livox_ros_driver2.msg import CustomMsg
    _HAS_CUSTOM = True
except ImportError:
    _HAS_CUSTOM = False
    CustomMsg = None


class ScanPublisher(Node):
    def __init__(self):
        super().__init__('scan_publisher')
        self.frame_id = 'livox_frame'   # body frame of LiDAR sensor
        self.n_ang = 720                # 0.5-deg bins
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_incr = (self.angle_max - self.angle_min) / self.n_ang
        self.range_min = 0.3            # robot self-blind zone
        self.range_max = 30.0
        self.zmin = -0.2                # relative to lidar frame
        self.zmax = 0.8                 # mid-body slice, avoid ground/ceiling

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(LaserScan, '/robonix/map/scan_2d', qos)

        if not _HAS_CUSTOM:
            self.get_logger().error('livox_ros_driver2 CustomMsg import failed — '
                                    'source mid360_drv/install/setup.bash first')
            raise RuntimeError('missing CustomMsg')
        self.create_subscription(CustomMsg, '/scanner/cloud', self.cb, 10)
        self.get_logger().info(
            f'ScanPublisher: {self.n_ang} bins, z=[{self.zmin},{self.zmax}], '
            f'range=[{self.range_min},{self.range_max}], frame={self.frame_id}')

    def cb(self, msg):
        n = len(msg.points)
        if n == 0:
            return
        xs = np.array([p.x for p in msg.points], dtype=np.float32)
        ys = np.array([p.y for p in msg.points], dtype=np.float32)
        zs = np.array([p.z for p in msg.points], dtype=np.float32)

        mask = (zs >= self.zmin) & (zs <= self.zmax) & np.isfinite(xs) & np.isfinite(ys)
        xs = xs[mask]; ys = ys[mask]
        if xs.size == 0:
            return

        r = np.hypot(xs, ys)
        ang = np.arctan2(ys, xs)
        rmask = (r >= self.range_min) & (r <= self.range_max)
        r = r[rmask]; ang = ang[rmask]
        if r.size == 0:
            return

        ab = ((ang - self.angle_min) / self.angle_incr).astype(np.int32)
        ab = np.clip(ab, 0, self.n_ang - 1)
        ranges = np.full(self.n_ang, math.inf, dtype=np.float32)
        np.minimum.at(ranges, ab, r)
        ranges[~np.isfinite(ranges)] = 0.0     # LaserScan uses 0 = no return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_incr
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        scan.intensities = []
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = ScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
