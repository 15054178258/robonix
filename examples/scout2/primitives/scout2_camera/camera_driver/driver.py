#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Scout2 camera primitive — Capability-based driver.

Owns `robonix/primitive/camera/*`. Subscribes to RGB + depth image topics,
CameraInfo, and extrinsics, exposing:

  primitive/camera/rgb            topic_in   (sensor_msgs/Image, /camera/camera/color/image_raw)
  primitive/camera/depth          topic_in   (sensor_msgs/Image, /camera/camera/depth/image_rect_raw)
  primitive/camera/extrinsics     topic_in   (TransformStamped, /camera/camera/extrinsics/depth_to_color)
  primitive/camera/intrinsics     topic_in   (CameraInfo, /camera/camera/color/camera_info)
  primitive/camera/snapshot       rpc        (MCP, RGB JPEG)
  primitive/camera/depth_snapshot rpc        (MCP, depth as 8-bit JPEG)
  primitive/camera/driver         rpc        (gRPC lifecycle)

Scout2 publishes real CameraInfo and extrinsics — no manual intrinsics config
needed unlike Webots.
"""
from __future__ import annotations

import os
import math
import threading
import time
from io import BytesIO

import numpy as np

from robonix_api import Primitive, Ok, Err, Deferred

scout2_camera = Primitive(id="scout2_camera", namespace="robonix/primitive/camera")

# ── shared state ─────────────────────────────────────────────────────────────
state_lock = threading.Lock()
intrinsics_lock = threading.Lock()
latest_rgb_jpeg: bytes | None = None
latest_depth_jpeg: bytes | None = None
extrinsics_pub = None  # rclpy publisher for the latched TF
intrinsics_pub = None  # rclpy publisher for the latched CameraInfo
intrinsics_published = False  # true after the first valid K sample is published
latest_intrinsics_msg = None
latest_intrinsics_k: list[float] = []
intrinsics_publish_interval_s = 0.5
last_intrinsics_publish = 0.0


# ── image conversion ─────────────────────────────────────────────────────────
def ros_image_to_jpeg(msg) -> bytes:
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    if enc == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif enc == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)[:, :, ::-1]
    elif enc == "rgba8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
    elif enc == "bgra8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3][:, :, ::-1]
    elif enc == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif enc == "16uc1":
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        arr = (raw / raw.max() * 255).astype(np.uint8) if raw.max() > 0 else np.zeros((h, w), np.uint8)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif enc == "32fc1":
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        valid = np.isfinite(raw)
        if valid.any():
            mn, mx = raw[valid].min(), raw[valid].max()
            norm = np.where(valid, (raw - mn) / max(mx - mn, 1e-6) * 255, 0).astype(np.uint8)
        else:
            norm = np.zeros((h, w), np.uint8)
        arr = np.stack([norm, norm, norm], axis=-1)
    else:
        raise ValueError(f"unsupported image encoding: {enc}")
    from PIL import Image as PILImage
    buf = BytesIO()
    PILImage.fromarray(np.ascontiguousarray(arr)).save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def on_rgb(msg):
    global latest_rgb_jpeg
    try:
        with state_lock:
            latest_rgb_jpeg = ros_image_to_jpeg(msg)
        publish_intrinsics_if_needed("rgb")
    except Exception as e:
        print(f"[scout2_camera] RGB conversion error: {e}", flush=True)


def on_depth(msg):
    global latest_depth_jpeg
    try:
        with state_lock:
            latest_depth_jpeg = ros_image_to_jpeg(msg)
    except Exception as e:
        print(f"[scout2_camera] Depth conversion error: {e}", flush=True)


def publish_intrinsics_if_needed(reason: str, *, force: bool = False) -> None:
    """Publish the best available CameraInfo on the intrinsics contract."""
    global last_intrinsics_publish
    pub = intrinsics_pub
    with intrinsics_lock:
        if pub is None:
            return
        if latest_intrinsics_msg is None:
            return
        msg = latest_intrinsics_msg
        k = list(latest_intrinsics_k)
    now = time.monotonic()
    elapsed = now - last_intrinsics_publish
    if not force and elapsed < intrinsics_publish_interval_s:
        return
    last_intrinsics_publish = now
    try:
        pub.publish(msg)
    except Exception as e:
        print(f"[scout2_camera] WARN: intrinsics publish failed: {e}", flush=True)
        return
    with intrinsics_lock:
        intrinsics_published = True
    print(
        f"[scout2_camera] publishing intrinsics via {reason}: "
        f"fx={k[0]:.1f} fy={k[1]:.1f} cx={k[2]:.1f} cy={k[3]:.1f} "
        f"{msg.width}x{msg.height}",
        flush=True,
    )


# ── MCP snapshot tools (typed against codegen MCP dataclasses) ──────────────
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import Image  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def now_header(frame_id: str) -> std_msgs_mcp.Header:
    now = time.time()
    sec = int(now)
    ns = int((now % 1) * 1e9) % 1_000_000_000
    return std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(sec=sec, nanosec=ns),
        frame_id=frame_id,
    )


def _save_snapshot(jpg: bytes, tag: str) -> None:
    """Save a snapshot JPEG to /tmp/robonix/ with a timestamp filename."""
    save_dir = "/tmp/robonix"
    os.makedirs(save_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(save_dir, f"{tag}_{ts}.jpg")
    try:
        with open(path, "wb") as f:
            f.write(jpg)
    except OSError as e:
        print(f"[scout2_camera] WARN: failed to save snapshot to {path}: {e}", flush=True)


def jpeg_to_image_mcp(jpg: bytes, frame_id: str) -> Image:
    from PIL import Image as PILImage
    im = PILImage.open(BytesIO(jpg))
    w, h = im.size
    return Image(
        header=now_header(frame_id),
        height=h, width=w,
        encoding="jpeg",
        is_bigendian=0,
        step=len(jpg),
        data=jpg,
    )


@scout2_camera.mcp("robonix/primitive/camera/snapshot")
def snapshot(msg: Empty) -> Image:
    """PRIMARY perception tool. Use freely — between every chassis/cmd
    burst — to see what's in front of the robot and decide what to do
    next. Returns the current RGB camera frame as a JPEG-encoded
    sensor_msgs/Image (`data` is base64).
    Contract: robonix/primitive/camera/snapshot."""
    _ = msg
    with state_lock:
        data = latest_rgb_jpeg
    if data is None:
        raise RuntimeError("no RGB image received yet")
    _save_snapshot(data, "rgb")
    return jpeg_to_image_mcp(
        data, os.environ.get("SCOUT2_RGB_FRAME_ID", "camera_color_optical_frame")
    )


@scout2_camera.mcp("robonix/primitive/camera/depth_snapshot")
def depth_snapshot(msg: Empty) -> Image:
    """Get the current depth camera frame as a JPEG-encoded
    sensor_msgs/Image (depth normalized to grayscale; binary `data` is
    base64). Use to gauge stand-off distance / find open space.
    Contract: robonix/primitive/camera/depth_snapshot."""
    _ = msg
    with state_lock:
        data = latest_depth_jpeg
    if data is None:
        raise RuntimeError("no depth image received yet")
    _save_snapshot(data, "depth")
    return jpeg_to_image_mcp(
        data, os.environ.get("SCOUT2_DEPTH_FRAME_ID", "camera_depth_optical_frame")
    )


# ── extrinsics: tf2 lookup once at startup, republish on a latched topic ────
def publish_extrinsics_when_ready(base_frame: str, cam_frame: str, topic: str) -> None:
    """Resolve `base_frame → cam_frame` from tf2, publish on latched extrinsics
    topic, exit."""
    from rclpy.duration import Duration  # type: ignore
    from rclpy.time import Time  # type: ignore
    from tf2_ros import Buffer, TransformListener  # type: ignore
    from robonix_api.ros import RosBackend
    node = RosBackend.get().node
    tf_buf = Buffer()
    TransformListener(tf_buf, node)
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        try:
            tf = tf_buf.lookup_transform(base_frame, cam_frame, Time(), Duration(seconds=0.5))
        except Exception:
            time.sleep(0.5)
            continue
        tf.header.frame_id = base_frame
        tf.child_frame_id = cam_frame
        if extrinsics_pub is not None:
            extrinsics_pub.publish(tf)
        t = tf.transform.translation
        print(f"[scout2_camera] published extrinsics {base_frame}→{cam_frame}: "
              f"({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) → {topic}")
        return
    print(f"[scout2_camera] WARN: extrinsics publish gave up — tf2 chain "
          f"{base_frame}→{cam_frame} not resolvable.")


def _cfg_float(cfg: dict, key: str, env_key: str, default: float = 0.0) -> float:
    value = cfg.get(key)
    if value is None:
        value = os.environ.get(env_key)
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _cfg_int(cfg: dict, key: str, env_key: str, default: int) -> int:
    value = cfg.get(key)
    if value is None:
        value = os.environ.get(env_key)
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


# ── lifecycle ────────────────────────────────────────────────────────────────
@scout2_camera.on_init
def init(cfg):
    global extrinsics_pub, intrinsics_pub, intrinsics_published
    global latest_intrinsics_msg, latest_intrinsics_k
    global intrinsics_publish_interval_s, last_intrinsics_publish
    cfg = cfg or {}
    rgb_topic = cfg.get("rgb_topic") or os.environ.get(
        "SCOUT2_RGB_TOPIC", "/camera/camera/color/image_raw")
    depth_topic = cfg.get("depth_topic") or os.environ.get(
        "SCOUT2_DEPTH_TOPIC", "/camera/camera/depth/image_rect_raw")
    extrinsics_topic = cfg.get("extrinsics_topic") or os.environ.get(
        "SCOUT2_CAMERA_EXTRINSICS_TOPIC", "/camera/camera/extrinsics/depth_to_color")
    camera_info_topic = cfg.get("camera_info_topic") or os.environ.get(
        "SCOUT2_CAMERA_INFO_TOPIC", "/camera/camera/color/camera_info")
    intrinsics_topic = cfg.get("intrinsics_topic") or os.environ.get(
        "SCOUT2_CAMERA_INTRINSICS_TOPIC", "/camera/camera/intrinsics")
    base_frame = cfg.get("base_frame") or os.environ.get("SCOUT2_BASE_FRAME", "base_link")
    cam_frame = cfg.get("cam_frame") or os.environ.get(
        "SCOUT2_RGB_FRAME_ID", "camera_color_optical_frame")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 60.0))
    intrinsics_publish_interval_s = _cfg_float(
        cfg, "intrinsics_publish_interval_s", "SCOUT2_CAMERA_INTRINSICS_PUBLISH_INTERVAL_S", 0.5
    )

    # subscribe RGB + depth (we own both contracts; declare manually below)
    scout2_camera.create_subscription("robonix/primitive/camera/rgb",
                             topic=rgb_topic, msg_type="Image",
                             callback=on_rgb, qos="best_effort", declare=False)
    scout2_camera.create_subscription("robonix/primitive/camera/depth",
                             topic=depth_topic, msg_type="Image",
                             callback=on_depth, qos="best_effort", declare=False)

    # latched extrinsics publisher
    from geometry_msgs.msg import TransformStamped  # type: ignore
    extrinsics_pub = scout2_camera.create_publisher(
        "robonix/primitive/camera/extrinsics",
        topic=extrinsics_topic, msg_type=TransformStamped, qos="latched",
    )
    threading.Thread(
        target=publish_extrinsics_when_ready,
        args=(base_frame, cam_frame, extrinsics_topic),
        daemon=True,
    ).start()

    # Intrinsics contract publisher. Scout2 publishes real CameraInfo over
    # /camera/camera/color/camera_info. Relay it directly.
    from sensor_msgs.msg import CameraInfo  # type: ignore
    intrinsics_pub = scout2_camera.create_publisher(
        "robonix/primitive/camera/intrinsics",
        topic=intrinsics_topic, msg_type=CameraInfo, qos="latched",
    )
    with intrinsics_lock:
        intrinsics_published = False
        latest_intrinsics_msg = None
        latest_intrinsics_k = []
        intrinsics_publish_interval_s = max(0.1, intrinsics_publish_interval_s)
        last_intrinsics_publish = 0.0

    def on_camera_info(msg, _topic=intrinsics_topic):
        global latest_intrinsics_msg, latest_intrinsics_k
        # Validate K before relaying: skip zero/partial CameraInfo.
        k = list(msg.k) if hasattr(msg, "k") else list(getattr(msg, "K", []))
        if len(k) < 6 or k[0] <= 0 or k[4] <= 0:
            return
        with intrinsics_lock:
            latest_intrinsics_msg = msg
            latest_intrinsics_k = [float(k[0]), float(k[4]), float(k[2]), float(k[5])]
        publish_intrinsics_if_needed(_topic, force=True)

    def publish_intrinsics_loop() -> None:
        try:
            while True:
                publish_intrinsics_if_needed("timer", force=True)
                with intrinsics_lock:
                    has_published = intrinsics_published
                if not has_published:
                    time.sleep(max(0.1, intrinsics_publish_interval_s))
                else:
                    time.sleep(max(0.1, intrinsics_publish_interval_s))
        except Exception as e:
            print(f"[scout2_camera] WARN: intrinsics publish thread exited: {e}", flush=True)

    scout2_camera.create_subscription(
        "robonix/primitive/camera/intrinsics",
        topic=camera_info_topic, msg_type="CameraInfo",
        callback=on_camera_info, qos="best_effort", declare=False,
    )
    threading.Thread(target=publish_intrinsics_loop, daemon=True).start()

    # Gate INIT on first RGB arriving.
    if not scout2_camera.wait_for_topic(rgb_topic, "Image", min(sentinel_timeout, 20.0)):
        fallback_wait_s = max(1.0, min(30.0, sentinel_timeout - 20.0))
        # Fallback: try ros2 topic echo
        import subprocess
        try:
            proc = subprocess.run(
                ["timeout", str(int(fallback_wait_s)), "ros2", "topic", "echo",
                 "--once", rgb_topic, "--field", "header"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                check=False,
            )
        except OSError:
            proc = None
        if proc is None or proc.returncode != 0:
            waited = 20.0 + fallback_wait_s
            return Err(f"no RGB on {rgb_topic} within {waited:.1f}s")
        print(f"[scout2_camera] RGB sample confirmed via ros2 CLI fallback on {rgb_topic}")

    scout2_camera.declare_ros2_topic("robonix/primitive/camera/rgb",   rgb_topic,   qos="best_effort")
    scout2_camera.declare_ros2_topic("robonix/primitive/camera/depth", depth_topic, qos="best_effort")
    return Ok()


@scout2_camera.on_shutdown
def shutdown():
    return Ok()


if __name__ == "__main__":
    scout2_camera.run()
