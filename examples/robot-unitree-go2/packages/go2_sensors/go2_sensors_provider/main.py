#!/usr/bin/env python3
"""Register the standard Go2 sensor streams with Robonix Atlas.

This module owns process lifecycle and endpoint metadata only. Sensor bytes are
handled by the C++ ROS nodes and the isolated, read-only camera daemon.
"""

from __future__ import annotations

import os
from pathlib import Path
import signal
import socket
import threading
import time
from typing import Any

from robonix_api import Deferred, Err, Ok, Primitive


ROOT = Path(__file__).resolve().parents[1]
ROS_BUILD = Path(os.environ.get("GO2_SENSORS_BUILD_ROOT", ROOT / ".build" / "ros"))
ROS_INSTALL = ROS_BUILD / "install" / "go2_sensors"
CAMERA_BUILD = Path(
    os.environ.get("GO2_CAMERA_DAEMON_BUILD_ROOT", ROOT / ".build" / "camera-daemon")
)
CAMERA_INSTALL = CAMERA_BUILD / "install"
CAMERA_LIBRARY_DIR = CAMERA_INSTALL / "lib"
SENSOR_RELAY = ROS_INSTALL / "lib" / "go2_sensors" / "go2_sensor_relay"
CAMERA_BRIDGE = ROS_INSTALL / "lib" / "go2_sensors" / "go2_camera_bridge"
CAMERA_DAEMON = CAMERA_INSTALL / "bin" / "go2_camera_daemon"
CAMERA_DDS_LIBRARIES = tuple(
    CAMERA_LIBRARY_DIR / name
    for name in ("libddsc.so", "libddsc.so.0", "libddscxx.so", "libddscxx.so.0")
)
PARAMETERS = ROOT / "config" / "go2_sensors.yaml"

provider = Primitive(id="go2_sensors", namespace="robonix/primitive/lidar")

# Keep this table literal and unconditional: package_manifest.yaml is a promise
# that every listed data-plane capability is declared after successful startup.
# CameraInfo remains a ROS diagnostic stream until a per-deployment calibration
# exists; an uncalibrated K[0] == 0 must not be advertised as intrinsics.
DATA_CAPABILITIES = (
    ("robonix/primitive/lidar/lidar3d", "lidar_output", "best_effort"),
    ("robonix/primitive/imu/imu", "imu_output", "best_effort"),
    ("robonix/primitive/camera/rgb", "camera_image", "best_effort"),
)

_lock = threading.Lock()
_config: dict[str, Any] = {}
_children: list[Any] = []
_declared = False
_active = False


def _wait_for_healthy_camera_diagnostic(topic: str, timeout_s: float) -> tuple[bool, str]:
    """Wait for an explicit, current camera quality pass without spawning CLI tools."""

    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as error:
        return False, f"camera quality observer dependency unavailable: {error}"

    context = Context()
    node = None
    passed = threading.Event()
    last_detail = "no go2_sensors/camera diagnostic received"

    def observe(message: Any) -> None:
        nonlocal last_detail
        for status in message.status:
            if status.name != "go2_sensors/camera":
                continue
            values: dict[str, str] = {}
            duplicate = False
            for entry in status.values:
                if entry.key in values:
                    duplicate = True
                    break
                values[entry.key] = entry.value
            if duplicate:
                last_detail = "camera diagnostic contains duplicate quality keys"
                return
            ready = values.get("quality_ready") == "true"
            healthy = values.get("healthy") == "true"
            if (
                ready
                and healthy
                and status.level == DiagnosticStatus.OK
                and status.message == "camera quality gate passed"
            ):
                last_detail = "camera quality gate passed"
                passed.set()
                return
            last_detail = (
                f"camera quality not ready/healthy: level={status.level}, "
                f"ready={ready}, healthy={healthy}, detail={status.message}"
            )

    try:
        rclpy.init(args=[], context=context)
        node = Node(f"go2_sensor_quality_gate_{os.getpid()}", context=context)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        subscription = node.create_subscription(DiagnosticArray, topic, observe, qos)
        deadline = time.monotonic() + timeout_s
        while not passed.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.25, remaining))
        # Keep the subscription alive until after the final spin.
        del subscription
        return passed.is_set(), last_detail
    except Exception as error:
        return False, f"camera quality observer failed: {type(error).__name__}: {error}"
    finally:
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()


_camera_quality_waiter = _wait_for_healthy_camera_diagnostic


def _absolute_topic(value: Any, name: str, default: str) -> str:
    topic = str(value or default)
    if not topic.startswith("/") or topic == "/":
        raise ValueError(f"{name} must be an absolute ROS topic")
    return topic


def _positive_float(value: Any, name: str, default: float, maximum: float) -> float:
    result = float(default if value is None else value)
    if not 0.0 < result <= maximum:
        raise ValueError(f"{name} must be in (0, {maximum}]")
    return result


def _boolean(value: Any, name: str, default: bool) -> bool:
    result = default if value is None else value
    if not isinstance(result, bool):
        raise ValueError(f"{name} must be a boolean")
    return result


def _normalize_config(cfg: dict[str, Any]) -> dict[str, Any]:
    source_mode = str(cfg.get("source_mode") or "local")
    if source_mode not in {"local", "external"}:
        raise ValueError("source_mode must be exactly 'local' or 'external'")

    interface = str(cfg.get("network_interface") or "")
    if not interface or "/" in interface or interface in {".", ".."} or len(interface) >= 16:
        raise ValueError("network_interface must be an explicit Linux interface name")
    if socket.if_nametoindex(interface) == 0:
        raise ValueError(f"network interface does not exist: {interface}")

    socket_path = Path(
        str(cfg.get("camera_ipc_socket") or f"/run/user/{os.getuid()}/robonix-go2/camera.sock")
    )
    if not socket_path.is_absolute() or ".." in socket_path.parts or len(str(socket_path)) >= 100:
        raise ValueError("camera_ipc_socket must be a short absolute path")

    domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if not 0 <= domain_id <= 232:
        raise ValueError("ROS_DOMAIN_ID must be in 0..232")

    normalized = {
        "source_mode": source_mode,
        "network_interface": interface,
        "domain_id": domain_id,
        "lidar_input": _absolute_topic(
            cfg.get("lidar_input_topic"), "lidar_input_topic", "/utlidar/cloud"
        ),
        "lidar_output": _absolute_topic(
            cfg.get("lidar_output_topic"), "lidar_output_topic", "/scanner/cloud"
        ),
        "imu_input": _absolute_topic(cfg.get("imu_input_topic"), "imu_input_topic", "/imu/data"),
        "imu_output": _absolute_topic(
            cfg.get("imu_output_topic"), "imu_output_topic", "/sensors/imu/data"
        ),
        "camera_image": _absolute_topic(
            cfg.get("camera_topic"), "camera_topic", "/camera/color/image_raw"
        ),
        "camera_info": _absolute_topic(
            cfg.get("camera_info_topic"),
            "camera_info_topic",
            "/camera/color/camera_info",
        ),
        "camera_status": _absolute_topic(
            cfg.get("camera_status_topic"),
            "camera_status_topic",
            "/go2/sensors/status",
        ),
        "camera_frame": str(cfg.get("camera_frame") or "front_camera"),
        "camera_fps": _positive_float(cfg.get("camera_fps"), "camera_fps", 10.0, 30.0),
        "camera_quality_required": _boolean(
            cfg.get("camera_quality_required"),
            "camera_quality_required",
            True,
        ),
        "camera_required": _boolean(
            cfg.get("camera_required"),
            "camera_required",
            True,
        ),
        "camera_socket": str(socket_path),
        "sentinel_timeout": _positive_float(
            cfg.get("sentinel_timeout_s"), "sentinel_timeout_s", 30.0, 60.0
        ),
    }
    if normalized["lidar_input"] == normalized["lidar_output"]:
        raise ValueError("lidar input and output topics must differ")
    if normalized["imu_input"] == normalized["imu_output"]:
        raise ValueError("IMU input and output topics must differ")
    if not normalized["camera_frame"] or normalized["camera_frame"].startswith("/"):
        raise ValueError("camera_frame must be a valid TF frame without a leading slash")
    return normalized


def _stop_children() -> None:
    global _children
    with _lock:
        children = _children
        _children = []
    for process in children:
        if process.poll() is not None:
            continue
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except OSError:
            continue
    deadline = time.monotonic() + 3.0
    for process in children:
        if process.poll() is not None:
            continue
        try:
            process.wait(timeout=max(0.0, deadline - time.monotonic()))
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=1.0)
            except Exception:
                pass


def _spawn(
    argv: list[str], log_name: str, *, child_env: dict[str, str] | None = None
) -> Any:
    process = provider.spawn(argv, env=child_env, log=log_name, cwd=ROOT)
    with _lock:
        _children.append(process)
    return process


@provider.on_init
def initialize(cfg: dict[str, Any]):
    global _config
    if not isinstance(cfg, dict):
        return Err("sensor config must be a JSON object")
    with _lock:
        if _active or _children:
            return Err("cannot reinitialize while the sensor provider is active")
    try:
        normalized = _normalize_config(cfg)
    except (TypeError, ValueError, OSError) as error:
        return Err(str(error))
    if normalized["source_mode"] == "local":
        executable_artifacts = [SENSOR_RELAY, CAMERA_BRIDGE, CAMERA_DAEMON]
        required_artifacts = [*executable_artifacts, *CAMERA_DDS_LIBRARIES, PARAMETERS]
        missing = [str(path) for path in required_artifacts if not path.is_file()]
        if missing:
            return Err("missing built sensor artifact(s): " + ", ".join(missing))
        not_executable = [
            str(path) for path in executable_artifacts if not os.access(path, os.X_OK)
        ]
        if not_executable:
            return Err("sensor artifact(s) are not executable: " + ", ".join(not_executable))
    if _declared and _config and normalized != _config:
        return Err("sensor endpoints were already declared; restart the provider to change config")
    _config = normalized
    return Ok()


@provider.on_activate
def activate():
    global _active, _declared
    if not _config:
        return Err("sensor provider was not initialized")
    with _lock:
        if _active or _children:
            return Deferred("sensor provider is already active")

    cfg = _config
    ros_common = ["--ros-args", "--params-file", str(PARAMETERS)]
    relay_arguments = [
        str(SENSOR_RELAY),
        *ros_common,
        "-p",
        f"lidar.input_topic:={cfg['lidar_input']}",
        "-p",
        f"lidar.output_topic:={cfg['lidar_output']}",
        "-p",
        f"imu.input_topic:={cfg['imu_input']}",
        "-p",
        f"imu.output_topic:={cfg['imu_output']}",
    ]
    camera_arguments = [
        str(CAMERA_BRIDGE),
        *ros_common,
        "-p",
        f"camera.socket_path:={cfg['camera_socket']}",
        "-p",
        f"camera.frame_id:={cfg['camera_frame']}",
        "-p",
        f"camera.image_topic:={cfg['camera_image']}",
        "-p",
        f"camera.camera_info_topic:={cfg['camera_info']}",
    ]
    daemon_arguments = [
        str(CAMERA_DAEMON),
        "--interface",
        cfg["network_interface"],
        "--domain-id",
        str(cfg["domain_id"]),
        "--fps",
        str(cfg["camera_fps"]),
        "--socket",
        cfg["camera_socket"],
    ]

    try:
        if cfg["source_mode"] == "local":
            _spawn(relay_arguments, "sensor-relay.log")
            # Override the inherited ROS LD_LIBRARY_PATH for this child only.  The
            # installed daemon also has $ORIGIN/../lib RPATH; this explicit child
            # environment prevents ROS's libddsc from winning loader precedence.
            _spawn(
                daemon_arguments,
                "camera-daemon.log",
                child_env={"LD_LIBRARY_PATH": str(CAMERA_LIBRARY_DIR)},
            )
            _spawn(camera_arguments, "camera-bridge.log")

        timeout = cfg["sentinel_timeout"]
        required = [
            (cfg["lidar_output"], "PointCloud2"),
            (cfg["imu_output"], "Imu"),
        ]
        if cfg["camera_required"]:
            required.extend(
                [
                    (cfg["camera_image"], "Image"),
                    # CameraInfo is a truthful ROS companion stream, even while
                    # K[0] == 0 means uncalibrated.
                    (cfg["camera_info"], "CameraInfo"),
                ]
            )
        deadline = time.monotonic() + timeout
        for topic, message_type in required:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not provider.wait_for_topic(topic, message_type, remaining):
                _stop_children()
                return Err(
                    f"no {message_type} sample on {topic} within the "
                    f"{timeout:.1f}s activation deadline"
                )

        if cfg["camera_required"] and cfg["camera_quality_required"]:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                _stop_children()
                return Err(
                    "camera quality gate had no time remaining in the "
                    "activation deadline"
                )
            # Do not route DiagnosticArray through Primitive.wait_for_topic():
            # the generic Robonix ROS helper intentionally resolves only the
            # common sensor/std/geometry/nav message packages.
            quality_ok, quality_detail = _camera_quality_waiter(
                cfg["camera_status"], remaining
            )
            if not quality_ok:
                _stop_children()
                return Err(
                    "camera stream exists but did not pass "
                    "quality_ready=true and healthy=true within the "
                    f"activation deadline: {quality_detail}"
                )

        if not _declared:
            for contract, topic_key, qos in DATA_CAPABILITIES:
                provider.declare_ros2_topic(contract, cfg[topic_key], qos=qos)
            _declared = True
        with _lock:
            _active = True
    except Exception as error:
        _stop_children()
        return Err(f"sensor activation failed: {type(error).__name__}: {error}")
    return Ok()


@provider.on_deactivate
def deactivate():
    global _active
    _stop_children()
    with _lock:
        _active = False
    return Ok()


@provider.on_shutdown
def shutdown():
    global _active
    _stop_children()
    with _lock:
        _active = False
    return Ok()


if __name__ == "__main__":
    provider.run()
