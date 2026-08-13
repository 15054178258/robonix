#!/usr/bin/env python3
# Voxel-accumulate world_cloud with temporal consensus + decay.
#
# - A voxel must be hit by >= CLOUD_ACC_MIN_HITS distinct scan frames to
#   be published. Moving objects (people behind the cart) only get 1-2
#   hits and are filtered out.
# - Periodically, voxels within CLOUD_ACC_DECAY_RANGE_M of the robot that
#   have NOT been re-hit for CLOUD_ACC_DECAY_FRAMES scans lose 1 hit.
#   If hits drop below min_hits they fall out of the published cloud;
#   drop to 0 and they're deleted. This handles "person stood here for
#   a while then left" without raycasting.
# - Voxels outside the decay range are kept as-is (we can't tell if they
#   moved or are just out of FOV).
import numpy as np, os, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class CloudAcc(Node):
    def __init__(self):
        super().__init__("cloud_accumulator")
        self.voxel = float(os.environ.get("CLOUD_ACC_VOXEL", "0.1"))
        self.z_min = float(os.environ.get("CLOUD_ACC_Z_MIN", "-2.0"))
        self.z_max = float(os.environ.get("CLOUD_ACC_Z_MAX", "1.5"))
        self.min_hits = int(os.environ.get("CLOUD_ACC_MIN_HITS", "3"))
        self.lock_at = int(os.environ.get("CLOUD_ACC_LOCK_AT", "30"))
        self.decay_frames = int(os.environ.get("CLOUD_ACC_DECAY_FRAMES", "50"))
        self.decay_range_m = float(os.environ.get("CLOUD_ACC_DECAY_RANGE_M", "8.0"))
        self.frame = os.environ.get("MAPPING_OUTPUT_FRAME", "lidar")
        # voxel_key -> [hits, x_mean, y_mean, z_mean, last_seen_scan_idx]
        self.voxels = {}
        self.scan_idx = 0
        self.robot_xyz = None
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(
            PointCloud2, "/robonix/map/cloud_accumulated", latched
        )
        self.create_subscription(
            PointCloud2, os.environ.get("MAPPING_CLOUD_TOPIC", "/fastlio2/world_cloud"), self.cb, 10
        )
        self.create_subscription(
            Odometry, os.environ.get("MAPPING_ODOM_TOPIC", "/fastlio2/lio_odom"), self.odom_cb, 20
        )
        self.create_timer(1.0, self.publish)
        # decay runs every CLOUD_ACC_DECAY_FRAMES/10 seconds (since scans
        # are ~10 Hz); use a wall timer keyed off that.
        self.create_timer(max(1.0, self.decay_frames / 10.0), self.decay)
        self.get_logger().info(
            f"CloudAcc: voxel={self.voxel}m z=[{self.z_min},{self.z_max}] "
            f"min_hits={self.min_hits} decay_frames={self.decay_frames} "
            f"decay_range={self.decay_range_m}m publish 1Hz"
        )

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.robot_xyz = (float(p.x), float(p.y), float(p.z))

    def cb(self, msg):
        self.frame_idx += 1
        offs = {f.name: f.offset for f in msg.fields}
        if "x" not in offs:
            return
        ox, oy, oz = offs["x"], offs["y"], offs["z"]
        ps = msg.point_step
        n_pts = msg.width * msg.height
        if n_pts == 0:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)[: n_pts * ps].reshape(n_pts, ps)
        x = arr[:, ox : ox + 4].copy().view(np.float32).ravel()
        y = arr[:, oy : oy + 4].copy().view(np.float32).ravel()
        z = arr[:, oz : oz + 4].copy().view(np.float32).ravel()
        m = (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(z)
            & (z > self.z_min)
            & (z < self.z_max)
        )
        x = x[m]; y = y[m]; z = z[m]
        if x.size == 0:
            return
        ix = np.floor(x / self.voxel).astype(np.int64)
        iy = np.floor(y / self.voxel).astype(np.int64)
        iz = np.floor(z / self.voxel).astype(np.int64)
        seen = set()
        for i in range(x.size):
            k = (int(ix[i]), int(iy[i]), int(iz[i]))
            if k in seen:
                continue
            seen.add(k)
            rec = self.voxels.get(k)
            if rec is None:
                self.voxels[k] = [1, float(x[i]), float(y[i]), float(z[i]),
                                  self.scan_idx]
            else:
                if rec[0] < self.lock_at:
                    rec[0] += 1
                    a = 1.0 / rec[0]
                    rec[1] = rec[1] * (1 - a) + float(x[i]) * a
                    rec[2] = rec[2] * (1 - a) + float(y[i]) * a
                    rec[3] = rec[3] * (1 - a) + float(z[i]) * a
                rec[4] = self.scan_idx
        self.scan_idx += 1
        if len(self.voxels) > 500_000:
            # evict lowest-hit voxels
            items = sorted(self.voxels.items(), key=lambda kv: kv[1][0], reverse=True)
            self.voxels = dict(items[:400_000])

    def decay(self):
        """Decrement hits for voxels we expected to see but didn't."""
        if self.robot_xyz is None or not self.voxels:
            return
        rx, ry, rz = self.robot_xyz
        r2 = self.decay_range_m ** 2
        stale_thresh = self.scan_idx - self.decay_frames
        remove = []
        decayed = 0
        for k, rec in self.voxels.items():
            # cheap squared distance from voxel center to robot
            dx = rec[1] - rx; dy = rec[2] - ry; dz = rec[3] - rz
            if dx*dx + dy*dy + dz*dz > r2:
                continue  # outside decay zone, skip
            if rec[4] >= stale_thresh:
                continue  # recently seen, safe
            rec[0] -= 1
            decayed += 1
            if rec[0] <= 0:
                remove.append(k)
        for k in remove:
            del self.voxels[k]
        if decayed or remove:
            self.get_logger().info(
                f"decay: -{decayed} hits, removed {len(remove)} voxels, "
                f"total={len(self.voxels)}"
            )

    def publish(self):
        if not self.voxels:
            return
        # Range-aware min_hits: inside decay_range_m we enforce min_hits to
        # filter dynamic objects (people/carts); outside we cant get enough
        # hits on a fast pass so we accept single-hit voxels — theyre in
        # the no-decay zone anyway (trusted-once-seen).
        if self.robot_xyz is not None and self.min_hits > 1:
            rx, ry, rz = self.robot_xyz
            r2 = self.decay_range_m ** 2
            kept = []
            for v in self.voxels.values():
                if v[0] >= self.min_hits:
                    kept.append((v[1], v[2], v[3])); continue
                dx = v[1]-rx; dy = v[2]-ry; dz = v[3]-rz
                if dx*dx + dy*dy + dz*dz > r2:
                    kept.append((v[1], v[2], v[3]))
            pts = np.array(kept, dtype=np.float32) if kept else np.empty((0,3), np.float32)
        else:
            pts = np.array(
                [(v[1], v[2], v[3]) for v in self.voxels.values()
                 if v[0] >= self.min_hits],
                dtype=np.float32,
            )
        if pts.size == 0:
            return
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = pts.tobytes()
        msg.is_dense = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    n = CloudAcc()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
