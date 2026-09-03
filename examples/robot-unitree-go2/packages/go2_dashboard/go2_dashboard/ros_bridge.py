"""ROS 2 subscriptions feeding the read-only dashboard state.

ROS imports are intentionally delayed until the observer thread starts.  The
web UI can therefore report a useful dependency error instead of crashing when
it is opened on a host where the ROS environment was not sourced.
"""

from __future__ import annotations

import copy
import io
import math
import os
from pathlib import Path
import queue
import threading
import time
from dataclasses import dataclass, replace
from typing import Any

from .parsers import (
    cloud_to_points,
    goal_status_label,
    image_to_rgb_bytes,
    occupancy_to_luma,
    quaternion_to_yaw,
    scan_to_points,
)
from .initial_pose_store import InitialPoseError, InitialPoseStore
from .state import DEFAULT_TOPIC_SPECS, DashboardState, TopicSpec


def _environment_name(name: str, default: str, *, ros_topic: bool) -> str:
    value = os.environ.get(name, default).strip()
    if not value or len(value) > 200 or any(character.isspace() for character in value):
        raise ValueError(f"invalid {name}")
    if ros_topic and not value.startswith("/"):
        raise ValueError(f"{name} must be an absolute ROS topic")
    return value


def _environment_path(name: str, default: str = "") -> str:
    value = os.environ.get(name, default).strip()
    if not value:
        return ""
    if len(value) > 4096 or "\x00" in value or not Path(value).is_absolute():
        raise ValueError(f"{name} must be an absolute path")
    return value


def _environment_switch(name: str, default: bool) -> bool:
    value = os.environ.get(name, "1" if default else "0")
    if value == "1":
        return True
    if value == "0":
        return False
    raise ValueError(f"{name} must be exactly 0 or 1")


@dataclass(frozen=True)
class RosConfig:
    camera_topic: str = "/camera/color/image_raw"
    camera_status_topic: str = "/go2/sensors/status"
    d435i_color_topic: str = "/go2/d435i/color/image_raw"
    d435i_depth_topic: str = "/go2/d435i/aligned_depth_to_color/image_raw"
    d435i_camera_info_topic: str = "/go2/d435i/color/camera_info"
    scan_topic: str = "/scanner/scan"
    cloud_topic: str = "/scanner/cloud"
    map_topic: str = "/map"
    pose_topic: str = "/robonix/map/pose"
    odom_topic: str = "/odom"
    nav_status_topic: str = "/navigate_to_pose/_action/status"
    initial_pose_topic: str = "/initialpose"
    map_lifecycle_topic: str = "/robonix/map/lifecycle"
    initial_pose_maps_dir: str = ""
    initial_pose_auto_restore: bool = False
    map_frame: str = "map"
    base_frame: str = "base_link"
    camera_period_s: float = 0.2
    lidar_period_s: float = 0.2

    @classmethod
    def from_environment(cls) -> "RosConfig":
        return cls(
            camera_topic=_environment_name(
                "GO2_DASHBOARD_CAMERA_TOPIC", cls.camera_topic, ros_topic=True
            ),
            camera_status_topic=_environment_name(
                "GO2_DASHBOARD_CAMERA_STATUS_TOPIC",
                cls.camera_status_topic,
                ros_topic=True,
            ),
            d435i_color_topic=_environment_name(
                "GO2_DASHBOARD_D435I_COLOR_TOPIC",
                cls.d435i_color_topic,
                ros_topic=True,
            ),
            d435i_depth_topic=_environment_name(
                "GO2_DASHBOARD_D435I_DEPTH_TOPIC",
                cls.d435i_depth_topic,
                ros_topic=True,
            ),
            d435i_camera_info_topic=_environment_name(
                "GO2_DASHBOARD_D435I_CAMERA_INFO_TOPIC",
                cls.d435i_camera_info_topic,
                ros_topic=True,
            ),
            scan_topic=_environment_name(
                "GO2_DASHBOARD_SCAN_TOPIC", cls.scan_topic, ros_topic=True
            ),
            cloud_topic=_environment_name(
                "GO2_DASHBOARD_CLOUD_TOPIC", cls.cloud_topic, ros_topic=True
            ),
            map_topic=_environment_name(
                "GO2_DASHBOARD_MAP_TOPIC", cls.map_topic, ros_topic=True
            ),
            pose_topic=_environment_name(
                "GO2_DASHBOARD_POSE_TOPIC", cls.pose_topic, ros_topic=True
            ),
            odom_topic=_environment_name(
                "GO2_DASHBOARD_ODOM_TOPIC", cls.odom_topic, ros_topic=True
            ),
            nav_status_topic=_environment_name(
                "GO2_DASHBOARD_NAV_STATUS_TOPIC",
                cls.nav_status_topic,
                ros_topic=True,
            ),
            initial_pose_topic=_environment_name(
                "GO2_DASHBOARD_INITIAL_POSE_TOPIC",
                cls.initial_pose_topic,
                ros_topic=True,
            ),
            map_lifecycle_topic=_environment_name(
                "GO2_DASHBOARD_MAP_LIFECYCLE_TOPIC",
                cls.map_lifecycle_topic,
                ros_topic=True,
            ),
            initial_pose_maps_dir=_environment_path(
                "GO2_DASHBOARD_INITIAL_POSE_MAPS_DIR",
                cls.initial_pose_maps_dir,
            ),
            initial_pose_auto_restore=_environment_switch(
                "GO2_DASHBOARD_INITIAL_POSE_AUTO_RESTORE",
                cls.initial_pose_auto_restore,
            ),
            map_frame=_environment_name(
                "GO2_DASHBOARD_MAP_FRAME", cls.map_frame, ros_topic=False
            ),
            base_frame=_environment_name(
                "GO2_DASHBOARD_BASE_FRAME", cls.base_frame, ros_topic=False
            ),
        )

    def topic_specs(self) -> dict[str, TopicSpec]:
        specs = dict(DEFAULT_TOPIC_SPECS)
        specs["camera"] = replace(specs["camera"], topic=self.camera_topic)
        specs["d435i_color"] = replace(
            specs["d435i_color"], topic=self.d435i_color_topic
        )
        specs["d435i_depth"] = replace(
            specs["d435i_depth"], topic=self.d435i_depth_topic
        )
        specs["d435i_info"] = replace(
            specs["d435i_info"], topic=self.d435i_camera_info_topic
        )
        specs["laser_scan"] = replace(specs["laser_scan"], topic=self.scan_topic)
        specs["point_cloud"] = replace(specs["point_cloud"], topic=self.cloud_topic)
        specs["map"] = replace(specs["map"], topic=self.map_topic)
        specs["odom"] = replace(specs["odom"], topic=self.odom_topic)
        specs["pose_map"] = replace(specs["pose_map"], topic=self.pose_topic)
        specs["nav_status"] = replace(
            specs["nav_status"], topic=self.nav_status_topic
        )
        specs["map_lifecycle"] = replace(
            specs["map_lifecycle"], topic=self.map_lifecycle_topic
        )
        specs["initial_pose_input"] = replace(
            specs["initial_pose_input"], topic=self.initial_pose_topic
        )
        return specs


def _stamp_payload(stamp: Any) -> dict[str, int | float]:
    seconds = int(stamp.sec)
    nanoseconds = int(stamp.nanosec)
    return {
        "sec": seconds,
        "nanosec": nanoseconds,
        "unix_s": seconds + nanoseconds / 1_000_000_000.0,
    }


def _finite_float(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} is not finite")
    return result


def _source_age_seconds(receipt_time_ns: int, stamp: Any) -> float:
    seconds = int(stamp.sec)
    nanoseconds = int(stamp.nanosec)
    if not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("pose stamp.nanosec is outside ROS time range")
    age = (int(receipt_time_ns) - (seconds * 1_000_000_000 + nanoseconds)) / 1e9
    if not math.isfinite(age):
        raise ValueError("pose source age is not finite")
    return age


def _require_exact_frame_id(value: Any, expected: str, label: str) -> str:
    frame_id = str(value)
    if frame_id != expected:
        raise ValueError(
            f"{label} frame_id must exactly match {expected!r}, got {frame_id!r}"
        )
    return frame_id


def _uint8(value: Any, label: str) -> int:
    """Decode ROS uint8 representations without permissive coercion.

    Some generated Python message bindings expose ``uint8`` as a one-byte
    bytes-like value while others expose an integer. Strings, booleans,
    multi-byte buffers, and out-of-range integers are rejected instead of
    being silently reinterpreted.
    """

    if isinstance(value, bool):
        raise TypeError(f"{label} must be uint8, not bool")
    if isinstance(value, int):
        if 0 <= value <= 255:
            return value
        raise ValueError(f"{label} is outside uint8 range")
    if isinstance(value, (bytes, bytearray, memoryview)):
        raw = bytes(value)
        if len(raw) == 1:
            return raw[0]
        raise ValueError(f"{label} bytes-like value must contain exactly one byte")
    raise TypeError(f"{label} must be an integer or one-byte bytes-like value")


def _retain_telemetry_subscriptions(
    node: Any, subscriptions: Any
) -> tuple[Any, ...]:
    """Keep telemetry handles alive without replacing rclpy's registry.

    ``rclpy.node.Node`` owns a private ``_subscriptions`` list which its
    executor enumerates. Dashboard-specific handles therefore need a distinct
    attribute; assigning the private rclpy name would hide subscriptions from
    the executor.
    """

    retained = tuple(subscriptions)
    node._telemetry_subscriptions = retained
    return retained


def _position_payload(position: Any) -> dict[str, float]:
    return {
        "x": round(_finite_float(position.x, "position.x"), 5),
        "y": round(_finite_float(position.y, "position.y"), 5),
        "z": round(_finite_float(position.z, "position.z"), 5),
    }


def _quaternion_payload(rotation: Any) -> dict[str, float]:
    return {
        "x": round(_finite_float(rotation.x, "orientation.x"), 7),
        "y": round(_finite_float(rotation.y, "orientation.y"), 7),
        "z": round(_finite_float(rotation.z, "orientation.z"), 7),
        "w": round(_finite_float(rotation.w, "orientation.w"), 7),
    }


def _pose_map_observation(
    message: Any,
    *,
    expected_map_frame: str,
    base_frame: str,
    receipt_time_ns: int,
) -> tuple[dict[str, Any], float]:
    frame_id = _require_exact_frame_id(
        message.header.frame_id, expected_map_frame, "pose"
    )
    pose = message.pose.pose
    rotation = pose.orientation
    source_age_s = _source_age_seconds(receipt_time_ns, message.header.stamp)
    return (
        {
            "parent_frame": expected_map_frame,
            "child_frame": base_frame,
            "stamp": _stamp_payload(message.header.stamp),
            "position": _position_payload(pose.position),
            "orientation": _quaternion_payload(rotation),
            "yaw": round(
                quaternion_to_yaw(
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                ),
                7,
            ),
        },
        source_age_s,
    )


def depth_to_rgb_bytes(
    data: bytes | bytearray | memoryview,
    width: int,
    height: int,
    step: int,
    encoding: str,
    is_bigendian: bool,
) -> bytes:
    """Colorize one bounded aligned 16-bit depth image for operator preview."""

    if encoding.strip().lower() not in {"16uc1", "mono16"}:
        raise ValueError(f"unsupported depth encoding: {encoding}")
    if width <= 0 or height <= 0 or width * height > 4_194_304:
        raise ValueError("depth dimensions are outside the preview limit")
    row_bytes = width * 2
    if step < row_bytes:
        raise ValueError("depth row step is smaller than the image width")
    raw = memoryview(data).cast("B")
    if len(raw) < step * height:
        raise ValueError("depth buffer is shorter than its declared layout")
    byteorder = "big" if is_bigendian else "little"
    preview = bytearray(width * height * 3)
    target = 0
    for row in range(height):
        offset = row * step
        for column in range(width):
            pixel = offset + column * 2
            depth_mm = int.from_bytes(raw[pixel : pixel + 2], byteorder)
            if depth_mm == 0:
                red = green = blue = 0
            else:
                normalized = min(1.0, max(0.0, (depth_mm - 250.0) / 4750.0))
                red = int(round(255.0 * normalized))
                green = int(round(255.0 * (1.0 - abs(2.0 * normalized - 1.0))))
                blue = int(round(255.0 * (1.0 - normalized)))
            preview[target : target + 3] = bytes((red, green, blue))
            target += 3
    return bytes(preview)


class RosBridge:
    """Own a single ROS observer thread and no command-side interfaces."""

    def __init__(
        self, state: DashboardState, config: RosConfig | None = None
    ) -> None:
        self._state = state
        self._config = config or RosConfig.from_environment()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._initial_pose_store = (
            InitialPoseStore(
                self._config.initial_pose_maps_dir,
                frame_id=self._config.map_frame,
            )
            if self._config.initial_pose_maps_dir
            else None
        )
        self._initial_pose_requests: queue.Queue[dict[str, Any]] = queue.Queue(
            maxsize=4
        )
        if self._initial_pose_store is not None:
            self._state.set_initial_pose_status(self._initial_pose_store.status())

    def initial_pose_status(self) -> dict[str, Any]:
        return self._state.initial_pose_status()

    def request_initial_pose_restore(
        self, *, map_id: str, generation: int
    ) -> dict[str, Any]:
        store = self._initial_pose_store
        if store is None:
            raise InitialPoseError("initial pose persistence is not configured")
        pose = store.restore_pose(map_id=map_id, generation=generation)
        try:
            self._initial_pose_requests.put_nowait(pose)
        except queue.Full as error:
            raise InitialPoseError("initial pose restore queue is busy") from error
        status = store.status()
        self._state.set_initial_pose_status(status)
        return status

    def reset_initial_pose(
        self, *, confirm_map_id: str, generation: int
    ) -> dict[str, Any]:
        store = self._initial_pose_store
        if store is None:
            raise InitialPoseError("initial pose persistence is not configured")
        status = store.reset(
            confirm_map_id=confirm_map_id, generation=generation
        )
        self._state.set_initial_pose_status(status)
        return status

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._state.set_bridge(
            running=True, connected=False, error="正在初始化 ROS 只读观察线程"
        )
        self._thread = threading.Thread(
            target=self._run,
            name="go2-dashboard-ros-observer",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout_s: float = 5.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, float(timeout_s)))

    def _run(self) -> None:
        context = None
        node = None
        executor = None
        try:
            import rclpy
            from action_msgs.msg import GoalStatusArray
            from diagnostic_msgs.msg import DiagnosticArray
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav_msgs.msg import OccupancyGrid, Odometry
            from PIL import Image
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from sensor_msgs.msg import CameraInfo
            from sensor_msgs.msg import Image as RosImage
            from sensor_msgs.msg import LaserScan, PointCloud2

            MapLifecycle = None
            if self._initial_pose_store is not None:
                from map.msg import MapLifecycle as MapLifecycleMessage

                MapLifecycle = MapLifecycleMessage

            state = self._state
            config = self._config
            stop_event = self._stop_event
            initial_pose_store = self._initial_pose_store
            initial_pose_requests = self._initial_pose_requests

            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            pose_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            action_status_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

            class _TelemetryNode(Node):
                def __init__(self, ros_context: Context) -> None:
                    super().__init__("go2_readonly_dashboard", context=ros_context)
                    self._last_camera = 0.0
                    self._last_d435i_color = 0.0
                    self._last_d435i_depth = 0.0
                    self._last_scan = 0.0
                    self._last_cloud = 0.0
                    self._stream_receipts: dict[str, float] = {}
                    self._last_initial_pose_publish: tuple[Any, ...] | None = None
                    self._last_initial_pose_publish_at = 0.0
                    self._last_auto_restore_identity: tuple[str, int] | None = None
                    subscriptions = list(_retain_telemetry_subscriptions(
                        self,
                        (
                            self.create_subscription(
                                RosImage,
                                config.camera_topic,
                                self._on_camera,
                                qos_profile_sensor_data,
                            ),
                            self.create_subscription(
                                DiagnosticArray,
                                config.camera_status_topic,
                                self._on_camera_status,
                                10,
                            ),
                            self.create_subscription(
                                RosImage,
                                config.d435i_color_topic,
                                self._on_d435i_color,
                                qos_profile_sensor_data,
                            ),
                            self.create_subscription(
                                RosImage,
                                config.d435i_depth_topic,
                                self._on_d435i_depth,
                                qos_profile_sensor_data,
                            ),
                            self.create_subscription(
                                CameraInfo,
                                config.d435i_camera_info_topic,
                                self._on_d435i_camera_info,
                                qos_profile_sensor_data,
                            ),
                            self.create_subscription(
                                LaserScan,
                                config.scan_topic,
                                self._on_scan,
                                qos_profile_sensor_data,
                            ),
                            self.create_subscription(
                                PointCloud2,
                                config.cloud_topic,
                                self._on_cloud,
                                qos_profile_sensor_data,
                            ),
                            self.create_subscription(
                                OccupancyGrid,
                                config.map_topic,
                                self._on_map,
                                map_qos,
                            ),
                            self.create_subscription(
                                PoseWithCovarianceStamped,
                                config.pose_topic,
                                self._on_map_pose,
                                pose_qos,
                            ),
                            self.create_subscription(
                                Odometry,
                                config.odom_topic,
                                self._on_odom,
                                qos_profile_sensor_data,
                            ),
                            self.create_subscription(
                                GoalStatusArray,
                                config.nav_status_topic,
                                self._on_nav_status,
                                action_status_qos,
                            ),
                        ),
                    ))
                    self._initial_pose_publisher = None
                    self._initial_pose_timer = None
                    if initial_pose_store is not None and MapLifecycle is not None:
                        subscriptions.extend(
                            (
                                self.create_subscription(
                                    MapLifecycle,
                                    config.map_lifecycle_topic,
                                    self._on_map_lifecycle,
                                    map_qos,
                                ),
                                self.create_subscription(
                                    PoseWithCovarianceStamped,
                                    config.initial_pose_topic,
                                    self._on_initial_pose,
                                    pose_qos,
                                ),
                            )
                        )
                        self._initial_pose_publisher = self.create_publisher(
                            PoseWithCovarianceStamped,
                            config.initial_pose_topic,
                            pose_qos,
                        )
                        self._initial_pose_timer = self.create_timer(
                            0.1, self._drain_initial_pose_requests
                        )
                        _retain_telemetry_subscriptions(self, subscriptions)

                def _observed_rate(self, key: str, now: float) -> float | None:
                    previous = self._stream_receipts.get(key)
                    self._stream_receipts[key] = now
                    if previous is None or now <= previous:
                        return None
                    return round(1.0 / (now - previous), 2)

                @staticmethod
                def _jpeg(image: Any) -> bytes:
                    output = io.BytesIO()
                    image.save(output, format="JPEG", quality=72)
                    return output.getvalue()

                @staticmethod
                def _initial_pose_payload(
                    message: PoseWithCovarianceStamped,
                ) -> dict[str, Any]:
                    pose = message.pose.pose
                    return {
                        "frame_id": _require_exact_frame_id(
                            message.header.frame_id,
                            config.map_frame,
                            "initial pose",
                        ),
                        "position": {
                            key: _finite_float(
                                getattr(pose.position, key), f"position.{key}"
                            )
                            for key in ("x", "y", "z")
                        },
                        "orientation": {
                            key: _finite_float(
                                getattr(pose.orientation, key),
                                f"orientation.{key}",
                            )
                            for key in ("x", "y", "z", "w")
                        },
                        "covariance": [
                            _finite_float(value, f"covariance[{index}]")
                            for index, value in enumerate(message.pose.covariance)
                        ],
                    }

                @staticmethod
                def _initial_pose_signature(pose: dict[str, Any]) -> tuple[Any, ...]:
                    return (
                        pose["frame_id"],
                        *(round(float(pose["position"][key]), 9) for key in ("x", "y", "z")),
                        *(round(float(pose["orientation"][key]), 9) for key in ("x", "y", "z", "w")),
                        *(round(float(value), 9) for value in pose["covariance"]),
                    )

                def _publish_initial_pose(self, pose: dict[str, Any]) -> None:
                    publisher = self._initial_pose_publisher
                    if publisher is None:
                        raise InitialPoseError(
                            "initial pose publisher is not available"
                        )
                    message = PoseWithCovarianceStamped()
                    message.header.frame_id = config.map_frame
                    message.header.stamp = self.get_clock().now().to_msg()
                    for key in ("x", "y", "z"):
                        setattr(
                            message.pose.pose.position,
                            key,
                            float(pose["position"][key]),
                        )
                    for key in ("x", "y", "z", "w"):
                        setattr(
                            message.pose.pose.orientation,
                            key,
                            float(pose["orientation"][key]),
                        )
                    message.pose.covariance = [
                        float(value) for value in pose["covariance"]
                    ]
                    self._last_initial_pose_publish = self._initial_pose_signature(pose)
                    self._last_initial_pose_publish_at = time.monotonic()
                    publisher.publish(message)

                def _drain_initial_pose_requests(self) -> None:
                    try:
                        pose = initial_pose_requests.get_nowait()
                    except queue.Empty:
                        return
                    try:
                        self._publish_initial_pose(pose)
                        status = initial_pose_store.status()
                        status["last_action"] = "restore_published"
                        state.set_initial_pose_status(status)
                    except Exception as error:
                        status = initial_pose_store.status()
                        status["error"] = str(error)
                        status["last_action"] = "restore_publish_failed"
                        state.set_initial_pose_status(status)

                def _on_map_lifecycle(self, message: Any) -> None:
                    try:
                        map_id = str(message.map_id)
                        mode = str(message.mode)
                        generation = int(message.generation)
                        state.observe(
                            "map_lifecycle",
                            {
                                "map_id": map_id,
                                "mode": mode,
                                "generation": generation,
                            },
                        )
                        status = initial_pose_store.observe_lifecycle(
                            map_id, mode, generation
                        )
                        state.set_initial_pose_status(status)
                        identity = (map_id, generation)
                        if (
                            config.initial_pose_auto_restore
                            and mode.strip().casefold() == "localization"
                            and status["saved"]
                            and identity != self._last_auto_restore_identity
                        ):
                            pose = initial_pose_store.restore_pose(
                                map_id=map_id, generation=generation
                            )
                            self._publish_initial_pose(pose)
                            self._last_auto_restore_identity = identity
                            status = initial_pose_store.status()
                            status["last_action"] = "auto_restore_published"
                            state.set_initial_pose_status(status)
                    except Exception as error:
                        state.note_error("map_lifecycle", error)
                        status = initial_pose_store.status()
                        status["error"] = str(error)
                        state.set_initial_pose_status(status)

                def _on_initial_pose(
                    self, message: PoseWithCovarianceStamped
                ) -> None:
                    try:
                        pose = self._initial_pose_payload(message)
                        signature = self._initial_pose_signature(pose)
                        state.observe(
                            "initial_pose_input",
                            {
                                "position": copy.deepcopy(pose["position"]),
                                "orientation": copy.deepcopy(pose["orientation"]),
                                "stamp": _stamp_payload(message.header.stamp),
                            },
                            frame_id=pose["frame_id"],
                        )
                        if (
                            signature == self._last_initial_pose_publish
                            and time.monotonic() - self._last_initial_pose_publish_at
                            <= 2.0
                        ):
                            return
                        status = initial_pose_store.save_operator_pose(pose)
                        state.set_initial_pose_status(status)
                    except Exception as error:
                        state.note_error("initial_pose_input", error)
                        status = initial_pose_store.status()
                        status["error"] = str(error)
                        state.set_initial_pose_status(status)

                def _on_camera(self, message: RosImage) -> None:
                    now = time.monotonic()
                    if now - self._last_camera < config.camera_period_s:
                        return
                    self._last_camera = now
                    try:
                        rgb = image_to_rgb_bytes(
                            message.data,
                            int(message.width),
                            int(message.height),
                            int(message.step),
                            str(message.encoding),
                        )
                        image = Image.frombytes(
                            "RGB", (int(message.width), int(message.height)), rgb
                        )
                        rate_hz = self._observed_rate("camera", now)
                        state.set_camera(
                            self._jpeg(image),
                            {
                                "width": int(message.width),
                                "height": int(message.height),
                                "encoding": str(message.encoding),
                                "fps": rate_hz,
                                "stamp": _stamp_payload(message.header.stamp),
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("camera", error)

                def _on_d435i_color(self, message: RosImage) -> None:
                    now = time.monotonic()
                    if now - self._last_d435i_color < config.camera_period_s:
                        return
                    self._last_d435i_color = now
                    try:
                        rgb = image_to_rgb_bytes(
                            message.data,
                            int(message.width),
                            int(message.height),
                            int(message.step),
                            str(message.encoding),
                        )
                        image = Image.frombytes(
                            "RGB", (int(message.width), int(message.height)), rgb
                        )
                        state.set_camera_stream(
                            "d435i_color",
                            self._jpeg(image),
                            {
                                "width": int(message.width),
                                "height": int(message.height),
                                "encoding": str(message.encoding),
                                "fps": self._observed_rate("d435i_color", now),
                                "stamp": _stamp_payload(message.header.stamp),
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("d435i_color", error)

                def _on_d435i_depth(self, message: RosImage) -> None:
                    now = time.monotonic()
                    if now - self._last_d435i_depth < config.camera_period_s:
                        return
                    self._last_d435i_depth = now
                    try:
                        rgb = depth_to_rgb_bytes(
                            message.data,
                            int(message.width),
                            int(message.height),
                            int(message.step),
                            str(message.encoding),
                            bool(message.is_bigendian),
                        )
                        image = Image.frombytes(
                            "RGB", (int(message.width), int(message.height)), rgb
                        )
                        state.set_camera_stream(
                            "d435i_depth",
                            self._jpeg(image),
                            {
                                "width": int(message.width),
                                "height": int(message.height),
                                "encoding": str(message.encoding),
                                "preview": "250-5000 mm colorized",
                                "fps": self._observed_rate("d435i_depth", now),
                                "stamp": _stamp_payload(message.header.stamp),
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("d435i_depth", error)

                def _on_d435i_camera_info(self, message: CameraInfo) -> None:
                    try:
                        state.observe(
                            "d435i_info",
                            {
                                "width": int(message.width),
                                "height": int(message.height),
                                "distortion_model": str(message.distortion_model),
                                "fx": round(_finite_float(message.k[0], "fx"), 4),
                                "fy": round(_finite_float(message.k[4], "fy"), 4),
                                "cx": round(_finite_float(message.k[2], "cx"), 4),
                                "cy": round(_finite_float(message.k[5], "cy"), 4),
                                "stamp": _stamp_payload(message.header.stamp),
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("d435i_info", error)

                def _on_camera_status(self, message: DiagnosticArray) -> None:
                    try:
                        selected = next(
                            entry
                            for entry in message.status
                            if str(entry.name) == "go2_sensors/camera"
                        )
                    except StopIteration:
                        return
                    try:
                        values = {
                            str(entry.key): str(entry.value)
                            for entry in selected.values
                        }
                        numeric_keys = (
                            "rate_hz",
                            "quality_error_ratio",
                            "quality_window_attempts",
                            "quality_window_failures",
                            "reconnect_count",
                            "rejected_count",
                            "daemon_source_rejected_count",
                            "daemon_api_error_count",
                            "daemon_last_api_code",
                        )
                        numeric = {
                            key: _finite_float(values[key], key)
                            for key in numeric_keys
                            if key in values
                        }
                        state.set_camera_quality(
                            {
                                **numeric,
                                "level": _uint8(selected.level, "diagnostic level"),
                                "message": str(selected.message),
                                "healthy": values.get("healthy") == "true",
                                "ready": values.get("quality_ready") == "true",
                                "api_code_semantics": (
                                    "opaque vendor return code; not interpreted"
                                ),
                            }
                        )
                    except Exception as error:
                        state.note_error("camera", error)

                def _on_scan(self, message: LaserScan) -> None:
                    now = time.monotonic()
                    if now - self._last_scan < config.lidar_period_s:
                        return
                    self._last_scan = now
                    try:
                        points = scan_to_points(
                            message.ranges,
                            float(message.angle_min),
                            float(message.angle_increment),
                            float(message.range_min),
                            float(message.range_max),
                        )
                        state.observe(
                            "laser_scan",
                            {
                                "kind": "LaserScan",
                                "points": points,
                                "point_count": len(points),
                                "range_max": round(float(message.range_max), 3),
                                "stamp": _stamp_payload(message.header.stamp),
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("laser_scan", error)

                def _on_cloud(self, message: PointCloud2) -> None:
                    now = time.monotonic()
                    if now - self._last_cloud < config.lidar_period_s:
                        return
                    self._last_cloud = now
                    try:
                        points = cloud_to_points(
                            message.data,
                            message.fields,
                            int(message.point_step),
                            int(message.width),
                            int(message.height),
                            bool(message.is_bigendian),
                            row_step=int(message.row_step),
                        )
                        state.observe(
                            "point_cloud",
                            {
                                "kind": "PointCloud2",
                                "points": points,
                                "point_count": len(points),
                                "stamp": _stamp_payload(message.header.stamp),
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("point_cloud", error)

                def _on_map(self, message: OccupancyGrid) -> None:
                    try:
                        frame_id = _require_exact_frame_id(
                            message.header.frame_id, config.map_frame, "map"
                        )
                        width = int(message.info.width)
                        height = int(message.info.height)
                        luma = occupancy_to_luma(message.data, width, height)
                        image = Image.frombytes("L", (width, height), luma)
                        output = io.BytesIO()
                        image.save(output, format="PNG", compress_level=3)
                        origin = message.info.origin
                        origin_yaw = quaternion_to_yaw(
                            origin.orientation.x,
                            origin.orientation.y,
                            origin.orientation.z,
                            origin.orientation.w,
                        )
                        resolution = _finite_float(
                            message.info.resolution, "map resolution"
                        )
                        if resolution <= 0.0:
                            raise ValueError("map resolution must be positive")
                        state.set_map(
                            output.getvalue(),
                            {
                                "width": width,
                                "height": height,
                                "resolution": resolution,
                                "origin": {
                                    **_position_payload(origin.position),
                                    "yaw": round(origin_yaw, 7),
                                },
                                "stamp": _stamp_payload(message.header.stamp),
                                "load_time": _stamp_payload(message.info.map_load_time),
                            },
                            frame_id=frame_id,
                        )
                    except Exception as error:
                        state.note_error("map", error)

                def _on_odom(self, message: Odometry) -> None:
                    try:
                        pose = message.pose.pose
                        twist = message.twist.twist
                        yaw = quaternion_to_yaw(
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        )
                        state.observe(
                            "odom",
                            {
                                "frame_id": str(message.header.frame_id),
                                "child_frame_id": str(message.child_frame_id),
                                "stamp": _stamp_payload(message.header.stamp),
                                "position": _position_payload(pose.position),
                                "orientation": _quaternion_payload(pose.orientation),
                                "yaw": round(yaw, 7),
                                "linear": {
                                    "x": round(
                                        _finite_float(twist.linear.x, "linear.x"), 5
                                    ),
                                    "y": round(
                                        _finite_float(twist.linear.y, "linear.y"), 5
                                    ),
                                    "z": round(
                                        _finite_float(twist.linear.z, "linear.z"), 5
                                    ),
                                },
                                "angular": {
                                    "x": round(
                                        _finite_float(twist.angular.x, "angular.x"), 5
                                    ),
                                    "y": round(
                                        _finite_float(twist.angular.y, "angular.y"), 5
                                    ),
                                    "z": round(
                                        _finite_float(twist.angular.z, "angular.z"), 5
                                    ),
                                },
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("odom", error)

                def _on_nav_status(self, message: GoalStatusArray) -> None:
                    try:
                        if not message.status_list:
                            payload = {
                                "status": "idle",
                                "status_code": 0,
                                "goal_id": "",
                                "stamp": None,
                            }
                        else:
                            selected = max(
                                message.status_list,
                                key=lambda entry: (
                                    int(entry.goal_info.stamp.sec),
                                    int(entry.goal_info.stamp.nanosec),
                                ),
                            )
                            status_code = int(selected.status)
                            payload = {
                                "status": goal_status_label(status_code),
                                "status_code": status_code,
                                "goal_id": bytes(
                                    selected.goal_info.goal_id.uuid
                                ).hex(),
                                "stamp": _stamp_payload(
                                    selected.goal_info.stamp
                                ),
                                "tracked_goals": len(message.status_list),
                            }
                        state.observe("nav_status", payload)
                    except Exception as error:
                        state.note_error("nav_status", error)

                def _on_map_pose(
                    self, message: PoseWithCovarianceStamped
                ) -> None:
                    try:
                        payload, source_age_s = _pose_map_observation(
                            message,
                            expected_map_frame=config.map_frame,
                            base_frame=config.base_frame,
                            receipt_time_ns=self.get_clock().now().nanoseconds,
                        )
                        state.observe(
                            "pose_map",
                            payload,
                            frame_id=config.map_frame,
                            source_age_s=source_age_s,
                        )
                    except Exception as error:
                        state.note_error("pose_map", error)

            context = Context()
            rclpy.init(args=None, context=context)
            node = _TelemetryNode(context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            state.set_bridge(running=True, connected=True, error="")
            while not stop_event.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception as error:
            self._state.set_bridge(running=False, connected=False, error=str(error))
        finally:
            if executor is not None:
                try:
                    executor.shutdown(timeout_sec=1.0)
                except Exception:
                    pass
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if context is not None:
                try:
                    context.shutdown()
                except Exception:
                    pass
            if self._stop_event.is_set():
                self._state.set_bridge(
                    running=False, connected=False, error="ROS 观察线程已停止"
                )
            elif self._state.snapshot()["bridge"]["connected"]:
                self._state.set_bridge(
                    running=False, connected=False, error="ROS 上下文意外结束"
                )
