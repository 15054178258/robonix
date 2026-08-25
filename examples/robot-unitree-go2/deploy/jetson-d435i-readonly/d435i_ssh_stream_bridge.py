#!/usr/bin/env python3
"""Bounded D435i stream from an Orin to a workstation ROS 2 publisher.

The same source has two explicit roles:

* ``--remote-producer`` opens only the D435i, compresses RGB and aligned depth,
  and writes a checksummed binary stream to stdout.
* ``--workstation-publisher`` reads that stream from stdin and publishes three
  sensor-only ROS 2 Humble topics on the workstation.

The remote role never imports ROS. This avoids the Orin's incompatible
``rmw_cyclonedds_cpp`` node-creation path while keeping all ROS publishers on
the workstation.
"""

import argparse
import json
import math
import os
import re
import resource
import signal
import struct
import sys
import threading
import time
import zlib
from dataclasses import dataclass
from typing import BinaryIO, Dict, Optional, Sequence, Tuple


READONLY_STREAMING_ACK = "I_ACKNOWLEDGE_D435I_READONLY_SENSOR_STREAMING_ONLY"
DEFAULT_NAMESPACE = "/go2/d435i"
DEFAULT_FRAME_ID = "d435i_color_optical_frame"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
PREVIEW_WIDTH = 320
PREVIEW_HEIGHT = 240
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
HARDWARE_FPS = 30
DEFAULT_PUBLISH_HZ = 6
MAX_WIDTH = 640
MAX_HEIGHT = 480
MAX_PUBLISH_HZ = 10
MAX_PROCESS_PEAK_RSS_MIB = 1024.0
DEPTH_METRES_PER_UNIT = 0.001

RGB_TOPIC = "color/image_raw"
ALIGNED_DEPTH_TOPIC = "aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "color/camera_info"

PROTOCOL_MAGIC = b"RBNXD435"
PROTOCOL_VERSION = 1
PACKET_PREFIX = struct.Struct("!8s7I")
MAX_HEADER_BYTES = 4096
MAX_JPEG_BYTES = 1024 * 1024
MAX_DEPTH_ZLIB_BYTES = 700 * 1024
JPEG_QUALITY = 80


class ConfigurationError(ValueError):
    """Raised before camera or ROS access."""


class StreamProtocolError(RuntimeError):
    """Raised for malformed, oversized, truncated or corrupt frame data."""


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
    if value != READONLY_STREAMING_ACK:
        raise ConfigurationError(
            "read-only D435i streaming acknowledgement is absent or not exact"
        )


def normalize_namespace(value: str) -> str:
    candidate = value.strip()
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
    if width <= 0 or height <= 0 or width > MAX_WIDTH or height > MAX_HEIGHT:
        raise ConfigurationError("resolution exceeds the approved 640x480 envelope")
    if hardware_fps != HARDWARE_FPS:
        raise ConfigurationError("hardware capture must remain exactly 30 Hz")
    if (
        publish_hz <= 0
        or publish_hz > MAX_PUBLISH_HZ
        or hardware_fps % publish_hz != 0
    ):
        raise ConfigurationError("publish_hz must divide 30 and not exceed 10 Hz")
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
    if frame_index < 0 or decimation <= 0:
        raise ConfigurationError("invalid frame decimation input")
    return frame_index % decimation == 0


def validate_depth_scale(metres_per_unit: float) -> None:
    if abs(float(metres_per_unit) - DEPTH_METRES_PER_UNIT) > 1e-9:
        raise ConfigurationError(
            "D435i depth scale is not 0.001 m/unit; refusing 16UC1 output"
        )


def _peak_rss_mib() -> float:
    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def validate_peak_rss(peak_rss_mib: float) -> None:
    if peak_rss_mib < 0.0 or peak_rss_mib > MAX_PROCESS_PEAK_RSS_MIB:
        raise RuntimeError(
            "D435i stream process peak RSS {:.1f} MiB exceeds {:.1f} MiB".format(
                peak_rss_mib,
                MAX_PROCESS_PEAK_RSS_MIB,
            )
        )


def _crc32(payload: bytes) -> int:
    return zlib.crc32(payload) & 0xFFFFFFFF


def _canonical_header_bytes(header: Dict[str, object]) -> bytes:
    return json.dumps(
        header,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("ascii")


def encode_packet(
    header: Dict[str, object],
    jpeg_payload: bytes,
    depth_zlib_payload: bytes,
) -> bytes:
    header_payload = _canonical_header_bytes(header)
    if not 0 < len(header_payload) <= MAX_HEADER_BYTES:
        raise StreamProtocolError("frame header exceeds the approved byte ceiling")
    if not 0 < len(jpeg_payload) <= MAX_JPEG_BYTES:
        raise StreamProtocolError("JPEG payload exceeds the approved byte ceiling")
    if not 0 < len(depth_zlib_payload) <= MAX_DEPTH_ZLIB_BYTES:
        raise StreamProtocolError(
            "compressed depth payload exceeds the approved byte ceiling"
        )
    prefix = PACKET_PREFIX.pack(
        PROTOCOL_MAGIC,
        PROTOCOL_VERSION,
        len(header_payload),
        len(jpeg_payload),
        len(depth_zlib_payload),
        _crc32(header_payload),
        _crc32(jpeg_payload),
        _crc32(depth_zlib_payload),
    )
    return prefix + header_payload + jpeg_payload + depth_zlib_payload


def _read_exact(stream: BinaryIO, size: int, allow_clean_eof: bool = False) -> bytes:
    if size < 0:
        raise StreamProtocolError("negative stream read size")
    chunks = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            if allow_clean_eof and remaining == size:
                return b""
            raise StreamProtocolError("D435i stream ended mid-packet")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_packet(stream: BinaryIO) -> Optional[Tuple[Dict[str, object], bytes, bytes]]:
    prefix = _read_exact(stream, PACKET_PREFIX.size, allow_clean_eof=True)
    if not prefix:
        return None
    (
        magic,
        version,
        header_length,
        jpeg_length,
        depth_length,
        header_crc,
        jpeg_crc,
        depth_crc,
    ) = PACKET_PREFIX.unpack(prefix)
    if magic != PROTOCOL_MAGIC or version != PROTOCOL_VERSION:
        raise StreamProtocolError("D435i stream magic/version mismatch")
    if not 0 < header_length <= MAX_HEADER_BYTES:
        raise StreamProtocolError("invalid D435i header length")
    if not 0 < jpeg_length <= MAX_JPEG_BYTES:
        raise StreamProtocolError("invalid D435i JPEG length")
    if not 0 < depth_length <= MAX_DEPTH_ZLIB_BYTES:
        raise StreamProtocolError("invalid D435i depth length")
    header_payload = _read_exact(stream, header_length)
    jpeg_payload = _read_exact(stream, jpeg_length)
    depth_payload = _read_exact(stream, depth_length)
    for label, payload, expected_crc in (
        ("header", header_payload, header_crc),
        ("JPEG", jpeg_payload, jpeg_crc),
        ("depth", depth_payload, depth_crc),
    ):
        if _crc32(payload) != expected_crc:
            raise StreamProtocolError("{} CRC mismatch".format(label))
    try:
        header = json.loads(header_payload.decode("ascii"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise StreamProtocolError("invalid D435i JSON header") from error
    if not isinstance(header, dict):
        raise StreamProtocolError("D435i header must be a JSON object")
    return header, jpeg_payload, depth_payload


def validate_header(header: Dict[str, object], plan: StreamPlan) -> Dict[str, object]:
    required = {
        "sequence",
        "capture_unix_ns",
        "capture_monotonic_ns",
        "width",
        "height",
        "frame_id",
        "color_codec",
        "depth_codec",
        "depth_scale",
        "fx",
        "fy",
        "ppx",
        "ppy",
        "coefficients",
    }
    if set(header) != required:
        raise StreamProtocolError("D435i header fields are not exact")
    if (
        not isinstance(header["sequence"], int)
        or header["sequence"] < 0
        or not isinstance(header["capture_unix_ns"], int)
        or header["capture_unix_ns"] <= 0
        or not isinstance(header["capture_monotonic_ns"], int)
        or header["capture_monotonic_ns"] <= 0
    ):
        raise StreamProtocolError("invalid D435i sequence or capture time")
    if (header["width"], header["height"]) != (plan.width, plan.height):
        raise StreamProtocolError("D435i frame dimensions changed")
    if header["frame_id"] != plan.frame_id:
        raise StreamProtocolError("D435i frame ID changed")
    if header["color_codec"] != "jpeg-bgr8":
        raise StreamProtocolError("unexpected D435i color codec")
    if header["depth_codec"] != "zlib-u16le":
        raise StreamProtocolError("unexpected D435i depth codec")
    validate_depth_scale(float(header["depth_scale"]))
    for field in ("fx", "fy", "ppx", "ppy"):
        value = float(header[field])
        if not math.isfinite(value):
            raise StreamProtocolError("non-finite D435i intrinsic")
        if field in ("fx", "fy") and value <= 0.0:
            raise StreamProtocolError("non-positive D435i focal length")
    coefficients = header["coefficients"]
    if not isinstance(coefficients, list) or len(coefficients) > 8:
        raise StreamProtocolError("invalid D435i distortion coefficients")
    if any(not math.isfinite(float(value)) for value in coefficients):
        raise StreamProtocolError("non-finite D435i distortion coefficient")
    return header


def _install_stop_handlers(stop_event: threading.Event) -> Dict[int, object]:
    previous = {}

    def request_stop(_signum, _frame):
        stop_event.set()

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    return previous


def _restore_stop_handlers(previous: Dict[int, object]) -> None:
    for signum, handler in previous.items():
        signal.signal(signum, handler)


def _load_remote_modules():
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError(
            "Orin requires existing OpenCV, NumPy and pyrealsense2"
        ) from error
    return cv2, np, rs


def _isolate_binary_stdout() -> BinaryIO:
    """Reserve a duplicate of stdout for protocol bytes and redirect fd 1 logs."""

    protocol_fd = os.dup(sys.stdout.fileno())
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    return os.fdopen(protocol_fd, "wb", buffering=0)


def _remote_header(
    sequence: int,
    capture_unix_ns: int,
    capture_monotonic_ns: int,
    plan: StreamPlan,
    intrinsics,
) -> Dict[str, object]:
    scale_x = float(plan.width) / float(CAPTURE_WIDTH)
    scale_y = float(plan.height) / float(CAPTURE_HEIGHT)
    return {
        "sequence": sequence,
        "capture_unix_ns": capture_unix_ns,
        "capture_monotonic_ns": capture_monotonic_ns,
        "width": plan.width,
        "height": plan.height,
        "frame_id": plan.frame_id,
        "color_codec": "jpeg-bgr8",
        "depth_codec": "zlib-u16le",
        "depth_scale": DEPTH_METRES_PER_UNIT,
        "fx": float(intrinsics.fx) * scale_x,
        "fy": float(intrinsics.fy) * scale_y,
        "ppx": float(intrinsics.ppx) * scale_x,
        "ppy": float(intrinsics.ppy) * scale_y,
        "coefficients": [float(value) for value in intrinsics.coeffs],
    }


def run_remote_producer(plan: StreamPlan) -> int:
    """Capture only D435i RGB/depth and write bounded binary packets."""

    output = _isolate_binary_stdout()
    cv2, np, rs = _load_remote_modules()
    stop_event = threading.Event()
    previous_handlers = _install_stop_handlers(stop_event)
    pipeline = rs.pipeline()
    pipeline_started = False
    try:
        config = rs.config()
        if plan.serial:
            config.enable_device(plan.serial)
        config.enable_stream(
            rs.stream.color,
            CAPTURE_WIDTH,
            CAPTURE_HEIGHT,
            rs.format.bgr8,
            plan.hardware_fps,
        )
        config.enable_stream(
            rs.stream.depth,
            CAPTURE_WIDTH,
            CAPTURE_HEIGHT,
            rs.format.z16,
            plan.hardware_fps,
        )
        profile = pipeline.start(config)
        pipeline_started = True
        device = profile.get_device()
        product_name = device.get_info(rs.camera_info.name)
        serial_number = device.get_info(rs.camera_info.serial_number)
        if "D435I" not in product_name.upper():
            raise RuntimeError(
                "connected RealSense is {!r}, not a D435i".format(product_name)
            )
        validate_depth_scale(device.first_depth_sensor().get_depth_scale())
        align_to_color = rs.align(rs.stream.color)
        print(
            "D435i remote producer active: device={} serial={} "
            "capture={}x{}@{}Hz transport={}x{}@{}Hz".format(
                product_name,
                serial_number,
                CAPTURE_WIDTH,
                CAPTURE_HEIGHT,
                plan.hardware_fps,
                plan.width,
                plan.height,
                plan.publish_hz,
            ),
            file=sys.stderr,
            flush=True,
        )
        frame_index = 0
        sequence = 0
        while not stop_event.is_set():
            validate_peak_rss(_peak_rss_mib())
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            selected = should_publish_frame(frame_index, plan.decimation)
            frame_index += 1
            if not selected:
                continue
            aligned = align_to_color.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                raise RuntimeError("aligned D435i frame set is incomplete")
            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            if color.shape != (CAPTURE_HEIGHT, CAPTURE_WIDTH, 3):
                raise RuntimeError("D435i color shape changed")
            if depth.shape != (CAPTURE_HEIGHT, CAPTURE_WIDTH):
                raise RuntimeError("D435i depth shape changed")
            if color.dtype != np.uint8 or depth.dtype != np.uint16:
                raise RuntimeError("D435i frame dtype changed")
            if not color.flags.c_contiguous or not depth.flags.c_contiguous:
                raise RuntimeError("D435i frame buffer is not contiguous")
            intrinsics = (
                color_frame.profile.as_video_stream_profile().get_intrinsics()
            )
            if (plan.width, plan.height) != (CAPTURE_WIDTH, CAPTURE_HEIGHT):
                color = cv2.resize(
                    color,
                    (plan.width, plan.height),
                    interpolation=cv2.INTER_AREA,
                )
                depth = cv2.resize(
                    depth,
                    (plan.width, plan.height),
                    interpolation=cv2.INTER_NEAREST,
                )
            if color.shape != (plan.height, plan.width, 3):
                raise RuntimeError("D435i transport color shape is invalid")
            if depth.shape != (plan.height, plan.width):
                raise RuntimeError("D435i transport depth shape is invalid")
            if not color.flags.c_contiguous or not depth.flags.c_contiguous:
                raise RuntimeError("D435i transport frame buffer is not contiguous")
            jpeg_ok, jpeg_array = cv2.imencode(
                ".jpg",
                color,
                [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
            )
            if not jpeg_ok:
                raise RuntimeError("OpenCV failed to encode D435i JPEG")
            jpeg_payload = jpeg_array.tobytes(order="C")
            depth_le = depth.astype("<u2", copy=False).tobytes(order="C")
            depth_payload = zlib.compress(depth_le, level=1)
            header = _remote_header(
                sequence,
                time.time_ns(),
                time.monotonic_ns(),
                plan,
                intrinsics,
            )
            packet = encode_packet(header, jpeg_payload, depth_payload)
            try:
                output.write(packet)
            except BrokenPipeError:
                stop_event.set()
                break
            sequence += 1
            validate_peak_rss(_peak_rss_mib())
            if sequence % (plan.publish_hz * 5) == 0:
                print(
                    "D435i remote health: frames={} jpeg={} depth_zlib={} "
                    "peak_rss={:.1f} MiB".format(
                        sequence,
                        len(jpeg_payload),
                        len(depth_payload),
                        _peak_rss_mib(),
                    ),
                    file=sys.stderr,
                    flush=True,
                )
        return 0
    finally:
        if pipeline_started:
            pipeline.stop()
        output.close()
        _restore_stop_handlers(previous_handlers)


def _load_workstation_modules():
    try:
        import cv2
        import numpy as np
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as error:
        raise RuntimeError(
            "workstation requires ROS 2 Humble, sensor_msgs, OpenCV and NumPy"
        ) from error
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    return (
        cv2,
        np,
        rclpy,
        SignalHandlerOptions,
        sensor_qos,
        CameraInfo,
        Image,
    )


def _decompress_depth(payload: bytes, expected_size: int) -> bytes:
    decompressor = zlib.decompressobj()
    raw = decompressor.decompress(payload, expected_size + 1)
    if len(raw) > expected_size or decompressor.unconsumed_tail:
        raise StreamProtocolError("decompressed depth exceeds its exact size")
    tail = decompressor.flush()
    if len(raw) + len(tail) > expected_size:
        raise StreamProtocolError("depth flush exceeds its exact size")
    raw += tail
    if (
        len(raw) != expected_size
        or not decompressor.eof
        or decompressor.unused_data
    ):
        raise StreamProtocolError("compressed depth is incomplete or has trailing data")
    return raw


def _message_stamp_from_now(node):
    # Until the D435i mounting transform and cross-machine acquisition latency
    # are measured, Scene preview uses one shared workstation receipt stamp.
    # The remote capture time remains in the checked protocol header for logs.
    return node.get_clock().now().to_msg()


def _make_camera_info(
    CameraInfo,
    header,
    stamp,
    frame_id: str,
    output_width: int,
    output_height: int,
):
    message = CameraInfo()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.width = output_width
    message.height = output_height
    message.distortion_model = "plumb_bob"
    distortion = [float(value) for value in header["coefficients"][:5]]
    distortion.extend([0.0] * (5 - len(distortion)))
    message.d = distortion
    scale_x = float(output_width) / float(header["width"])
    scale_y = float(output_height) / float(header["height"])
    fx = float(header["fx"]) * scale_x
    fy = float(header["fy"]) * scale_y
    ppx = float(header["ppx"]) * scale_x
    ppy = float(header["ppy"]) * scale_y
    message.k = [fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [
        fx, 0.0, ppx, 0.0,
        0.0, fy, ppy, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return message


def _make_image(Image, array, encoding: str, stamp, frame_id: str):
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = int(array.shape[0])
    message.width = int(array.shape[1])
    message.encoding = encoding
    message.is_bigendian = 0
    bytes_per_pixel = 3 if encoding == "bgr8" else 2
    message.step = message.width * bytes_per_pixel
    message.data = array.tobytes(order="C")
    return message


def run_workstation_publisher(plan: StreamPlan) -> int:
    """Validate SSH packets and publish only the three approved sensor topics."""

    (
        cv2,
        np,
        rclpy,
        SignalHandlerOptions,
        sensor_qos,
        CameraInfo,
        Image,
    ) = _load_workstation_modules()
    stop_event = threading.Event()
    previous_handlers = _install_stop_handlers(stop_event)
    ros_started = False
    node = None
    try:
        rclpy.init(
            args=[],
            signal_handler_options=SignalHandlerOptions.NO,
        )
        ros_started = True
        node = rclpy.create_node(
            "d435i_workstation_stream_publisher",
            namespace=plan.namespace,
            enable_rosout=False,
            start_parameter_services=False,
        )
        rgb_publisher = node.create_publisher(Image, RGB_TOPIC, sensor_qos)
        depth_publisher = node.create_publisher(
            Image,
            ALIGNED_DEPTH_TOPIC,
            sensor_qos,
        )
        info_publisher = node.create_publisher(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            sensor_qos,
        )
        input_stream = sys.stdin.buffer
        expected_sequence = 0
        previous_capture_monotonic_ns = 0
        started_monotonic = time.monotonic()
        while rclpy.ok() and not stop_event.is_set():
            validate_peak_rss(_peak_rss_mib())
            packet = read_packet(input_stream)
            if packet is None:
                if stop_event.is_set():
                    break
                raise StreamProtocolError("SSH D435i stream ended unexpectedly")
            header, jpeg_payload, depth_payload = packet
            validate_header(header, plan)
            if header["sequence"] != expected_sequence:
                raise StreamProtocolError(
                    "D435i sequence discontinuity: {} != {}".format(
                        header["sequence"],
                        expected_sequence,
                    )
                )
            capture_monotonic_ns = int(header["capture_monotonic_ns"])
            if capture_monotonic_ns <= previous_capture_monotonic_ns:
                raise StreamProtocolError(
                    "D435i source monotonic time did not increase"
                )
            color = cv2.imdecode(
                np.frombuffer(jpeg_payload, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if (
                color is None
                or color.shape != (plan.height, plan.width, 3)
                or color.dtype != np.uint8
                or not color.flags.c_contiguous
            ):
                raise StreamProtocolError("decoded D435i color layout is invalid")
            raw_depth = _decompress_depth(
                depth_payload,
                plan.width * plan.height * 2,
            )
            depth = np.frombuffer(raw_depth, dtype="<u2").reshape(
                (plan.height, plan.width)
            )
            if not depth.flags.c_contiguous:
                raise StreamProtocolError("decoded D435i depth is not contiguous")
            preview_color = cv2.resize(
                color,
                (PREVIEW_WIDTH, PREVIEW_HEIGHT),
                interpolation=cv2.INTER_AREA,
            )
            preview_depth = cv2.resize(
                depth,
                (PREVIEW_WIDTH, PREVIEW_HEIGHT),
                interpolation=cv2.INTER_NEAREST,
            )
            if (
                preview_color.shape != (PREVIEW_HEIGHT, PREVIEW_WIDTH, 3)
                or preview_color.dtype != np.uint8
                or not preview_color.flags.c_contiguous
            ):
                raise StreamProtocolError("D435i preview color layout is invalid")
            if (
                preview_depth.shape != (PREVIEW_HEIGHT, PREVIEW_WIDTH)
                or preview_depth.dtype != np.uint16
                or not preview_depth.flags.c_contiguous
            ):
                raise StreamProtocolError("D435i preview depth layout is invalid")
            if stop_event.is_set() or not rclpy.ok():
                break
            stamp = _message_stamp_from_now(node)
            info_message = _make_camera_info(
                CameraInfo,
                header,
                stamp,
                plan.frame_id,
                PREVIEW_WIDTH,
                PREVIEW_HEIGHT,
            )
            rgb_message = _make_image(
                Image,
                preview_color,
                "bgr8",
                stamp,
                plan.frame_id,
            )
            depth_message = _make_image(
                Image,
                preview_depth,
                "16UC1",
                stamp,
                plan.frame_id,
            )
            info_publisher.publish(info_message)
            rgb_publisher.publish(rgb_message)
            depth_publisher.publish(depth_message)
            rclpy.spin_once(node, timeout_sec=0.0)
            expected_sequence += 1
            previous_capture_monotonic_ns = capture_monotonic_ns
            validate_peak_rss(_peak_rss_mib())
            if expected_sequence % (plan.publish_hz * 5) == 0:
                elapsed = max(time.monotonic() - started_monotonic, 1e-6)
                remote_age_ms = (
                    time.time_ns() - int(header["capture_unix_ns"])
                ) / 1_000_000.0
                node.get_logger().info(
                    "D435i workstation health: frames={} average_hz={:.2f} "
                    "remote_clock_age_ms={:.1f} peak_rss={:.1f} MiB".format(
                        expected_sequence,
                        expected_sequence / elapsed,
                        remote_age_ms,
                        _peak_rss_mib(),
                    )
                )
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if ros_started and rclpy.ok():
            rclpy.shutdown()
        _restore_stop_handlers(previous_handlers)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="D435i bounded SSH stream and workstation ROS publisher"
    )
    role = parser.add_mutually_exclusive_group(required=True)
    role.add_argument("--remote-producer", action="store_true")
    role.add_argument("--workstation-publisher", action="store_true")
    parser.add_argument(
        "--readonly-streaming-ack",
        default=os.environ.get("D435I_READONLY_STREAMING_ACK", ""),
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
        if arguments.remote_producer:
            return run_remote_producer(plan)
        return run_workstation_publisher(plan)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        role = "remote producer" if arguments.remote_producer else "workstation publisher"
        print("D435i {} failed: {}".format(role, error), file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
