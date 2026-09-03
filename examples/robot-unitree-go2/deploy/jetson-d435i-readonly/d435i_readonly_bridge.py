#!/usr/bin/env python3
"""Ephemeral ROS 2 Foxy bridge for a USB-connected RealSense D435i.

The file is designed to be streamed to an Orin with ``ssh ... python3 -``.
Imports that require ROS or librealsense are deliberately deferred until after
the exact read-only acknowledgement and all stream limits have been checked.
"""

import argparse
import os
import re
import resource
import signal
import sys
import threading
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple


READONLY_STREAMING_ACK = "I_ACKNOWLEDGE_D435I_READONLY_SENSOR_STREAMING_ONLY"
DEFAULT_NAMESPACE = "/go2/d435i"
DEFAULT_FRAME_ID = "d435i_color_optical_frame"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
HARDWARE_FPS = 30
DEFAULT_PUBLISH_HZ = 10
MAX_WIDTH = 640
MAX_HEIGHT = 480
MAX_PUBLISH_HZ = 10
MAX_IMAGE_PAYLOAD_BYTES = MAX_WIDTH * MAX_HEIGHT * 3
MAX_BRIDGE_PEAK_RSS_MIB = 1024.0
DEPTH_METRES_PER_UNIT = 0.001

RGB_TOPIC = "color/image_raw"
ALIGNED_DEPTH_TOPIC = "aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "color/camera_info"


class ConfigurationError(ValueError):
    """Raised before ROS or camera hardware is opened."""


@dataclass(frozen=True)
class StreamPlan:
    namespace: str
    frame_id: str
    width: int
    height: int
    hardware_fps: int
    publish_hz: int
    serial: str = ""

    @property
    def decimation(self) -> int:
        return self.hardware_fps // self.publish_hz


def require_readonly_streaming_ack(value: str) -> None:
    """Require the exact operator acknowledgement; near matches fail closed."""

    if value != READONLY_STREAMING_ACK:
        raise ConfigurationError(
            "read-only D435i streaming acknowledgement is absent or not exact"
        )


def normalize_namespace(value: str) -> str:
    """Return one absolute ROS namespace while rejecting shell-like input."""

    candidate = value.strip()
    if not candidate:
        raise ConfigurationError("namespace must not be empty")
    parts = [part for part in candidate.split("/") if part]
    if not parts or any(
        re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", part) is None for part in parts
    ):
        raise ConfigurationError("namespace must contain only ROS name segments")
    return "/" + "/".join(parts)


def validate_frame_id(value: str) -> str:
    candidate = value.strip().lstrip("/")
    if not candidate or re.fullmatch(r"[A-Za-z_][A-Za-z0-9_/]*", candidate) is None:
        raise ConfigurationError("frame_id is not a valid relative ROS frame name")
    return candidate


def make_stream_plan(
    namespace: str = DEFAULT_NAMESPACE,
    frame_id: str = DEFAULT_FRAME_ID,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
    hardware_fps: int = HARDWARE_FPS,
    publish_hz: int = DEFAULT_PUBLISH_HZ,
    serial: str = "",
) -> StreamPlan:
    """Validate the deliberately narrow first-version streaming envelope."""

    if width <= 0 or height <= 0 or width > MAX_WIDTH or height > MAX_HEIGHT:
        raise ConfigurationError(
            "resolution exceeds the read-only 640x480 safety/performance envelope"
        )
    if hardware_fps != HARDWARE_FPS:
        raise ConfigurationError("D435i hardware capture must remain exactly 30 Hz")
    if (
        publish_hz <= 0
        or publish_hz > MAX_PUBLISH_HZ
        or hardware_fps % publish_hz != 0
    ):
        raise ConfigurationError(
            "publish_hz must divide 30 exactly and must not exceed 10 Hz"
        )
    clean_serial = serial.strip()
    if clean_serial and re.fullmatch(r"[A-Za-z0-9]+", clean_serial) is None:
        raise ConfigurationError("serial must be alphanumeric")
    return StreamPlan(
        namespace=normalize_namespace(namespace),
        frame_id=validate_frame_id(frame_id),
        width=width,
        height=height,
        hardware_fps=hardware_fps,
        publish_hz=publish_hz,
        serial=clean_serial,
    )


def should_publish_frame(frame_index: int, decimation: int) -> bool:
    """Select one frame from each fixed 30 Hz input group."""

    if frame_index < 0:
        raise ConfigurationError("frame_index must be non-negative")
    if decimation <= 0:
        raise ConfigurationError("decimation must be positive")
    return frame_index % decimation == 0


def validate_depth_scale(metres_per_unit: float) -> None:
    """The raw 16UC1 ROS encoding is valid only for millimetre depth units."""

    if abs(float(metres_per_unit) - DEPTH_METRES_PER_UNIT) > 1e-9:
        raise ConfigurationError(
            "D435i depth scale is not 0.001 m/unit; refusing ambiguous 16UC1 output"
        )


def validate_image_layout(
    width: int,
    height: int,
    step: int,
    encoding: str,
) -> Tuple[int, Tuple[int, ...]]:
    """Return the bounded payload size and exact NumPy shape for one frame."""

    bytes_per_pixel = {"bgr8": 3, "16UC1": 2}.get(encoding)
    if bytes_per_pixel is None:
        raise ConfigurationError("unsupported D435i image encoding")
    if (
        width <= 0
        or height <= 0
        or width > MAX_WIDTH
        or height > MAX_HEIGHT
    ):
        raise RuntimeError("camera frame dimensions exceed the approved envelope")
    expected_step = width * bytes_per_pixel
    if step != expected_step:
        raise RuntimeError(
            "camera frame step mismatch: {} != {}".format(step, expected_step)
        )
    expected_size = height * step
    if expected_size <= 0 or expected_size > MAX_IMAGE_PAYLOAD_BYTES:
        raise RuntimeError("camera frame payload exceeds the approved byte ceiling")
    shape = (height, width, 3) if encoding == "bgr8" else (height, width)
    return expected_size, shape


def validate_bridge_peak_rss(peak_rss_mib: float) -> None:
    """Abort a leaking bridge well before it can exhaust the Orin's memory."""

    if (
        peak_rss_mib < 0.0
        or peak_rss_mib > MAX_BRIDGE_PEAK_RSS_MIB
    ):
        raise RuntimeError(
            "D435i bridge peak RSS {:.1f} MiB exceeds {:.1f} MiB ceiling".format(
                peak_rss_mib,
                MAX_BRIDGE_PEAK_RSS_MIB,
            )
        )


def _peak_rss_mib() -> float:
    # Linux reports ru_maxrss in KiB. This bridge is intentionally Linux/Orin
    # specific, so no cross-platform unit heuristic is needed here.
    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def camera_info_fields(
    width: int,
    height: int,
    fx: float,
    fy: float,
    ppx: float,
    ppy: float,
    coefficients: Sequence[float],
) -> Dict[str, object]:
    """Build ROS CameraInfo calibration fields without importing ROS."""

    distortion = [float(value) for value in coefficients[:5]]
    distortion.extend([0.0] * (5 - len(distortion)))
    return {
        "width": int(width),
        "height": int(height),
        "distortion_model": "plumb_bob",
        "d": distortion,
        "k": [
            float(fx), 0.0, float(ppx),
            0.0, float(fy), float(ppy),
            0.0, 0.0, 1.0,
        ],
        "r": [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ],
        "p": [
            float(fx), 0.0, float(ppx), 0.0,
            0.0, float(fy), float(ppy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ],
    }


def _load_runtime_modules():
    try:
        import numpy as np
    except ImportError as error:
        raise RuntimeError(
            "NumPy is unavailable on the Orin; no camera was opened"
        ) from error
    try:
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError(
            "pyrealsense2 is unavailable on the Orin; no camera was opened"
        ) from error
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Foxy Python sensor message dependencies are unavailable"
        ) from error
    return np, rs, rclpy, qos_profile_sensor_data, CameraInfo, Image


def _make_image_message(
    np,
    image_type,
    frame,
    encoding: str,
    stamp,
    frame_id: str,
):
    width = int(frame.get_width())
    height = int(frame.get_height())
    step = int(frame.get_stride_in_bytes())
    expected_size, expected_shape = validate_image_layout(
        width,
        height,
        step,
        encoding,
    )
    # pyrealsense2's documented NumPy buffer view avoids BufData's iterable
    # conversion path. On JetPack that path was observed allocating until the
    # 15 GiB Orin was OOM-killed. Validate nbytes and shape before the one
    # bounded copy into a ROS message.
    array = np.asanyarray(frame.get_data())
    if tuple(int(value) for value in array.shape) != expected_shape:
        raise RuntimeError(
            "camera frame shape mismatch: {} != {}".format(
                tuple(array.shape),
                expected_shape,
            )
        )
    if int(array.nbytes) != expected_size:
        raise RuntimeError(
            "camera frame payload/stride mismatch: "
            "{} bytes != {}x{}".format(array.nbytes, height, step)
        )
    if not bool(array.flags.c_contiguous):
        raise RuntimeError("camera frame buffer is not C-contiguous")
    payload = array.tobytes(order="C")
    if len(payload) != expected_size:
        raise RuntimeError("bounded camera frame copy changed payload size")
    message = image_type()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = height
    message.width = width
    message.encoding = encoding
    message.is_bigendian = 1 if sys.byteorder == "big" else 0
    message.step = step
    message.data = payload
    return message


def _make_camera_info_message(
    camera_info_type,
    intrinsics,
    stamp,
    frame_id: str,
):
    fields = camera_info_fields(
        intrinsics.width,
        intrinsics.height,
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.ppx,
        intrinsics.ppy,
        intrinsics.coeffs,
    )
    message = camera_info_type()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.width = fields["width"]
    message.height = fields["height"]
    message.distortion_model = fields["distortion_model"]
    message.d = fields["d"]
    message.k = fields["k"]
    message.r = fields["r"]
    message.p = fields["p"]
    return message


def _install_stop_handlers(stop_event: threading.Event) -> Dict[int, object]:
    previous = {}

    def request_stop(_signum, _frame):
        stop_event.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[signum] = signal.getsignal(signum)
            signal.signal(signum, request_stop)
    except Exception:
        _restore_stop_handlers(previous)
        raise
    return previous


def _restore_stop_handlers(previous: Dict[int, object]) -> None:
    for signum, handler in previous.items():
        signal.signal(signum, handler)


def run_bridge(plan: StreamPlan) -> int:
    """Open only D435i color/depth streams and publish the bounded ROS view."""

    np, rs, rclpy, sensor_qos, CameraInfo, Image = _load_runtime_modules()
    stop_event = threading.Event()
    previous_handlers = {}
    pipeline = None
    node = None
    ros_started = False
    pipeline_started = False
    cleanup_errors = []
    try:
        previous_handlers = _install_stop_handlers(stop_event)
        pipeline = rs.pipeline()
        camera_config = rs.config()
        if plan.serial:
            camera_config.enable_device(plan.serial)
        camera_config.enable_stream(
            rs.stream.color,
            plan.width,
            plan.height,
            rs.format.bgr8,
            plan.hardware_fps,
        )
        camera_config.enable_stream(
            rs.stream.depth,
            plan.width,
            plan.height,
            rs.format.z16,
            plan.hardware_fps,
        )

        # The bridge has already parsed and validated its own arguments.
        # Supplying an empty ROS argument list prevents Foxy from interpreting
        # the acknowledgement or camera limits as middleware arguments.
        rclpy.init(args=[])
        ros_started = True
        node = rclpy.create_node(
            "d435i_readonly_bridge",
            namespace=plan.namespace,
        )
        rgb_publisher = node.create_publisher(Image, RGB_TOPIC, sensor_qos)
        depth_publisher = node.create_publisher(
            Image, ALIGNED_DEPTH_TOPIC, sensor_qos
        )
        info_publisher = node.create_publisher(
            CameraInfo, CAMERA_INFO_TOPIC, sensor_qos
        )

        pipeline_profile = pipeline.start(camera_config)
        pipeline_started = True
        device = pipeline_profile.get_device()
        product_name = device.get_info(rs.camera_info.name)
        serial_number = device.get_info(rs.camera_info.serial_number)
        if "D435I" not in product_name.upper():
            raise RuntimeError(
                "connected RealSense is {!r}, not a D435i".format(product_name)
            )
        validate_depth_scale(device.first_depth_sensor().get_depth_scale())
        align_to_rgb = rs.align(rs.stream.color)

        node.get_logger().info(
            "D435i read-only bridge active: device={} serial={} namespace={} "
            "capture={}x{}@{}Hz publish={}Hz".format(
                product_name,
                serial_number,
                plan.namespace,
                plan.width,
                plan.height,
                plan.hardware_fps,
                plan.publish_hz,
            )
        )
        frame_index = 0
        published_frame_sets = 0
        while rclpy.ok() and not stop_event.is_set():
            validate_bridge_peak_rss(_peak_rss_mib())
            try:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
            except RuntimeError:
                if stop_event.is_set():
                    break
                raise
            if not should_publish_frame(frame_index, plan.decimation):
                frame_index += 1
                continue
            frame_index += 1

            aligned_frames = align_to_rgb.process(frames)
            rgb_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not rgb_frame or not depth_frame:
                raise RuntimeError("aligned D435i frame set is incomplete")
            if (
                rgb_frame.get_width() != plan.width
                or rgb_frame.get_height() != plan.height
                or depth_frame.get_width() != plan.width
                or depth_frame.get_height() != plan.height
            ):
                raise RuntimeError("D435i delivered a frame outside the approved size")

            intrinsics = (
                rgb_frame.profile.as_video_stream_profile().get_intrinsics()
            )
            # One Orin ROS clock sample is deliberately shared by RGB, aligned
            # depth and CameraInfo for this published frame set.
            stamp = node.get_clock().now().to_msg()
            rgb_message = _make_image_message(
                np, Image, rgb_frame, "bgr8", stamp, plan.frame_id
            )
            depth_message = _make_image_message(
                np, Image, depth_frame, "16UC1", stamp, plan.frame_id
            )
            info_message = _make_camera_info_message(
                CameraInfo, intrinsics, stamp, plan.frame_id
            )
            info_publisher.publish(info_message)
            rgb_publisher.publish(rgb_message)
            depth_publisher.publish(depth_message)
            rclpy.spin_once(node, timeout_sec=0.0)
            validate_bridge_peak_rss(_peak_rss_mib())
            published_frame_sets += 1
            if published_frame_sets % (plan.publish_hz * 5) == 0:
                node.get_logger().info(
                    "D435i read-only health: published_frame_sets={} "
                    "peak_rss={:.1f} MiB".format(
                        published_frame_sets,
                        _peak_rss_mib(),
                    )
                )
        return 0
    finally:
        active_error = sys.exc_info()[0] is not None
        if pipeline_started and pipeline is not None:
            try:
                pipeline.stop()
            except Exception as error:  # cleanup must continue through all layers
                cleanup_errors.append("pipeline.stop: {}".format(error))
        if node is not None:
            try:
                node.destroy_node()
            except Exception as error:
                cleanup_errors.append("destroy_node: {}".format(error))
        if ros_started:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as error:
                cleanup_errors.append("rclpy.shutdown: {}".format(error))
        if previous_handlers:
            try:
                _restore_stop_handlers(previous_handlers)
            except Exception as error:
                cleanup_errors.append("restore signal handlers: {}".format(error))
        if cleanup_errors:
            detail = "strict D435i cleanup failure: " + "; ".join(cleanup_errors)
            if active_error:
                print(detail, file=sys.stderr)
            else:
                raise RuntimeError(detail)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Ephemeral, read-only D435i RGB/aligned-depth ROS 2 bridge"
    )
    parser.add_argument(
        "--readonly-streaming-ack",
        default=os.environ.get("D435I_READONLY_STREAMING_ACK", ""),
        help="must exactly acknowledge read-only D435i sensor streaming",
    )
    parser.add_argument("--namespace", default=DEFAULT_NAMESPACE)
    parser.add_argument("--frame-id", default=DEFAULT_FRAME_ID)
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--hardware-fps", type=int, default=HARDWARE_FPS)
    parser.add_argument("--publish-hz", type=int, default=DEFAULT_PUBLISH_HZ)
    parser.add_argument("--serial", default="")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        require_readonly_streaming_ack(arguments.readonly_streaming_ack)
        plan = make_stream_plan(
            namespace=arguments.namespace,
            frame_id=arguments.frame_id,
            width=arguments.width,
            height=arguments.height,
            hardware_fps=arguments.hardware_fps,
            publish_hz=arguments.publish_hz,
            serial=arguments.serial,
        )
    except ConfigurationError as error:
        print("D435i bridge refused before hardware access: {}".format(error), file=sys.stderr)
        return 2
    try:
        return run_bridge(plan)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print("D435i read-only bridge failed: {}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
