#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Snapshot the current SLAM map into a set of human-previewable, offline,
# portable artifacts — so a saved map is usable WITHOUT rtabmap's database
# viewer and consumable by nav2 / scene directly.
#
# Two output modes:
#   --out-dir DIR     (preferred; per-map_id store, used by atlas_bridge)
#       DIR/occupancy.pgm   nav2 convention (0=occ, 254=free, 205=unknown)
#       DIR/occupancy.yaml  nav2 map_server yaml
#       DIR/occupancy.png   same image, png (double-click to preview)
#       DIR/cloud.pcd       binary PCD of the fused/accumulated cloud
#       DIR/meta.yaml       map_id, timestamp, frame, resolution, size
#       (DIR/rtabmap.db is written live by rtabmap; referenced in meta.)
#   <out_prefix>      (legacy; writes <out_prefix>.pcd/.pgm/.yaml/.png)
#
# Occupancy + cloud are read from whichever of the candidate latched topics
# is publishing, so this works for the rtabmap path (/map, /rtabmap/cloud_map)
# AND the dlio/adapter path (/robonix/map/occupancy_grid,
# /robonix/map/cloud_accumulated) without configuration. Override with
# --occ-topic / --cloud-topic.
#
# Poll the topics for up to --timeout seconds, then exit explicitly. Using
# spin_once + sys.exit instead of spin() because spin+shutdown-in-cb hangs
# rclpy in Humble.
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

# Candidate latched topics, tried in order. The robonix contract topics are
# what consumers see; the engine-native ones (/map, /rtabmap/cloud_map) are
# what rtabmap actually publishes inside the container.
OCC_TOPICS = ["/map", "/robonix/map/occupancy_grid"]
CLOUD_TOPICS = ["/rtabmap/cloud_map", "/robonix/map/cloud_accumulated"]


def write_pcd(path, xyz):
    n = xyz.shape[0]
    hdr = (
        b"# .PCD v0.7 - Point Cloud Data file format\n"
        b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        b"WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
        % (n, n)
    )
    with open(path, "wb") as f:
        f.write(hdr)
        f.write(xyz.astype("<f4").tobytes())


def grid_to_image(msg):
    """OccupancyGrid → (uint8 image, already row-flipped to image convention)."""
    w, h = msg.info.width, msg.info.height
    data = np.array(msg.data, dtype=np.int16).reshape(h, w)
    img = np.full_like(data, 205, dtype=np.uint8)  # unknown
    img[data == 0] = 254                            # free
    img[data >= 50] = 0                             # occupied
    return np.flipud(img), w, h


def write_grid(pgm_path, yaml_path, png_path, msg):
    img, w, h = grid_to_image(msg)
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.tobytes())
    with open(yaml_path, "w") as f:
        f.write(
            f"image: {os.path.basename(pgm_path)}\n"
            f"resolution: {msg.info.resolution}\n"
            f"origin: [{msg.info.origin.position.x}, "
            f"{msg.info.origin.position.y}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
    try:
        from PIL import Image
        Image.fromarray(img, mode="L").save(png_path)
    except ImportError:
        pass  # png is a convenience; pgm is the canonical image


def cloud_to_xyz(m):
    ps = m.point_step
    npts = m.width * m.height
    offs = {f.name: f.offset for f in m.fields}
    if npts <= 0 or "x" not in offs:
        return None
    arr = np.frombuffer(m.data, dtype=np.uint8)[: npts * ps].reshape(npts, ps)
    x = arr[:, offs["x"]:offs["x"] + 4].copy().view(np.float32).ravel()
    y = arr[:, offs["y"]:offs["y"] + 4].copy().view(np.float32).ravel()
    z = arr[:, offs["z"]:offs["z"] + 4].copy().view(np.float32).ravel()
    return np.stack([x, y, z], axis=1)


def collect(occ_topics, cloud_topics, timeout_s, settle_s):
    rclpy.init()
    node = rclpy.create_node("map_snapshotter")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    got = {"cloud": None, "grid": None}
    first_complete_at = None

    def on_grid(m):
        got["grid"] = m

    def on_cloud(m):
        got["cloud"] = m

    for t in occ_topics:
        node.create_subscription(OccupancyGrid, t, on_grid, latched)
    for t in cloud_topics:
        node.create_subscription(PointCloud2, t, on_cloud, latched)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.1)
        if got["cloud"] is not None and got["grid"] is not None:
            if first_complete_at is None:
                first_complete_at = time.time()
            if time.time() - first_complete_at >= settle_s:
                break
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass
    return got


def main():
    ap = argparse.ArgumentParser(description="Snapshot the SLAM map offline.")
    ap.add_argument("prefix", nargs="?",
                    help="legacy out-prefix (writes <prefix>.pcd/.pgm/.yaml/.png)")
    ap.add_argument("--out-dir",
                    help="per-map_id dir; writes occupancy.* + cloud.pcd + meta.yaml")
    ap.add_argument("--timeout", type=float, default=8.0,
                    help="seconds to wait for the latched topics")
    ap.add_argument("--occ-topic", action="append",
                    help="override occupancy topic(s) (repeatable)")
    ap.add_argument("--cloud-topic", action="append",
                    help="override cloud topic(s) (repeatable)")
    ap.add_argument("--settle", type=float, default=1.2,
                    help="after the first complete snapshot, keep spinning this long so a fresh /map publish can replace the latched old sample")
    args = ap.parse_args()

    if not args.out_dir and not args.prefix:
        ap.error("pass --out-dir DIR (preferred) or a legacy out-prefix")

    if args.out_dir:
        os.makedirs(args.out_dir, exist_ok=True)
        pcd_path = os.path.join(args.out_dir, "cloud.pcd")
        pgm_path = os.path.join(args.out_dir, "occupancy.pgm")
        yaml_path = os.path.join(args.out_dir, "occupancy.yaml")
        png_path = os.path.join(args.out_dir, "occupancy.png")
        meta_path = os.path.join(args.out_dir, "meta.yaml")
        map_id = os.path.basename(os.path.normpath(args.out_dir))
    else:
        pcd_path = args.prefix + ".pcd"
        pgm_path = args.prefix + ".pgm"
        yaml_path = args.prefix + ".yaml"
        png_path = args.prefix + ".png"
        meta_path = None
        map_id = os.path.basename(args.prefix)

    occ_topics = args.occ_topic or OCC_TOPICS
    cloud_topics = args.cloud_topic or CLOUD_TOPICS
    got = collect(occ_topics, cloud_topics, args.timeout, args.settle)

    rc = 0
    cloud_n, grid_info = 0, None

    if got["cloud"] is not None:
        xyz = cloud_to_xyz(got["cloud"])
        if xyz is not None and xyz.shape[0] > 0:
            write_pcd(pcd_path, xyz)
            cloud_n = xyz.shape[0]
            print(f"[save_map] wrote {pcd_path} ({cloud_n} pts, "
                  f"frame={got['cloud'].header.frame_id})")
        else:
            print("[save_map] cloud empty; continuing with occupancy preview only")
    else:
        print(f"[save_map] no cloud on {cloud_topics}; continuing with occupancy preview only")

    if got["grid"] is not None:
        write_grid(pgm_path, yaml_path, png_path, got["grid"])
        g = got["grid"].info
        grid_info = g
        print(f"[save_map] wrote {pgm_path}/.yaml/.png "
              f"({g.width}x{g.height} @ {g.resolution}m, "
              f"frame={got['grid'].header.frame_id})")
    else:
        print(f"[save_map] no occupancy grid on {occ_topics}"); rc = 1

    if meta_path is not None:
        db = os.path.join(args.out_dir, "rtabmap.db")
        with open(meta_path, "w") as f:
            f.write(f"map_id: {map_id}\n")
            f.write(f"saved_at: {time.strftime('%Y-%m-%dT%H:%M:%S%z')}\n")
            f.write(f"spatial_artifact: {'rtabmap.db' if os.path.isfile(db) else 'none'}\n")
            f.write(f"cloud_points: {cloud_n}\n")
            if grid_info is not None:
                f.write(f"frame_id: {got['grid'].header.frame_id}\n")
                f.write(f"resolution: {grid_info.resolution}\n")
                f.write(f"width: {grid_info.width}\n")
                f.write(f"height: {grid_info.height}\n")
                f.write(f"origin: [{grid_info.origin.position.x}, "
                        f"{grid_info.origin.position.y}, 0.0]\n")
        print(f"[save_map] wrote {meta_path}")

    sys.exit(rc)


if __name__ == "__main__":
    main()
