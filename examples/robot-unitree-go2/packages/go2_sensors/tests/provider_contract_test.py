#!/usr/bin/env python3
"""Offline check that the provider fulfils every manifest data capability."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import types


ROOT = Path(__file__).resolve().parents[1]
PROVIDER_PATH = ROOT / "go2_sensors_provider" / "main.py"


class FakeProcess:
    _next_pid = 100000

    def __init__(self) -> None:
        self.pid = FakeProcess._next_pid
        FakeProcess._next_pid += 1

    def poll(self):
        return None


class FakePrimitive:
    def __init__(self, *, id: str, namespace: str) -> None:
        self.id = id
        self.namespace = namespace
        self.spawned: list[tuple[list[str], dict[str, str] | None, str, Path]] = []
        self.waited: list[tuple[str, str, float]] = []
        self.declared: list[tuple[str, str, str]] = []

    def on_init(self, callback):
        self.init_callback = callback
        return callback

    def on_activate(self, callback):
        self.activate_callback = callback
        return callback

    def on_deactivate(self, callback):
        self.deactivate_callback = callback
        return callback

    def on_shutdown(self, callback):
        self.shutdown_callback = callback
        return callback

    def spawn(self, argv, *, env=None, log: str, cwd: Path):
        self.spawned.append((list(argv), env, log, cwd))
        return FakeProcess()

    def wait_for_topic(self, topic: str, message_type: str, timeout: float) -> bool:
        self.waited.append((topic, message_type, timeout))
        return True

    def declare_ros2_topic(self, contract: str, topic: str, *, qos: str) -> None:
        self.declared.append((contract, topic, qos))

    def run(self) -> None:
        raise AssertionError("offline contract test must not run the provider")


fake_api = types.ModuleType("robonix_api")
fake_api.Primitive = FakePrimitive
fake_api.Ok = lambda: ("ok", None)
fake_api.Err = lambda message: ("err", message)
fake_api.Deferred = lambda message: ("deferred", message)
sys.modules["robonix_api"] = fake_api

spec = importlib.util.spec_from_file_location("go2_sensors_provider_contract", PROVIDER_PATH)
if spec is None or spec.loader is None:
    raise SystemExit("could not load provider module")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
quality_waits: list[tuple[str, float]] = []
module._camera_quality_waiter = lambda topic, timeout: (
    quality_waits.append((topic, timeout)) or (True, "offline quality pass")
)

expected_ros_install = module.ROS_BUILD / "install" / "go2_sensors" / "lib" / "go2_sensors"
if module.SENSOR_RELAY != expected_ros_install / "go2_sensor_relay":
    raise SystemExit(f"provider sensor relay path does not match colcon isolated install: {module.SENSOR_RELAY}")
if module.CAMERA_BRIDGE != expected_ros_install / "go2_camera_bridge":
    raise SystemExit(f"provider camera bridge path does not match colcon isolated install: {module.CAMERA_BRIDGE}")

artifacts = tempfile.TemporaryDirectory(prefix="go2-sensors-contract-")
artifact_root = Path(artifacts.name)
module.SENSOR_RELAY = artifact_root / "ros" / "go2_sensor_relay"
module.CAMERA_BRIDGE = artifact_root / "ros" / "go2_camera_bridge"
module.CAMERA_INSTALL = artifact_root / "camera"
module.CAMERA_LIBRARY_DIR = module.CAMERA_INSTALL / "lib"
module.CAMERA_DAEMON = module.CAMERA_INSTALL / "bin" / "go2_camera_daemon"
module.CAMERA_DDS_LIBRARIES = tuple(
    module.CAMERA_LIBRARY_DIR / name
    for name in ("libddsc.so", "libddsc.so.0", "libddscxx.so", "libddscxx.so.0")
)
module.PARAMETERS = artifact_root / "go2_sensors.yaml"
for executable in (module.SENSOR_RELAY, module.CAMERA_BRIDGE, module.CAMERA_DAEMON):
    executable.parent.mkdir(parents=True, exist_ok=True)
    executable.write_text("offline placeholder\n", encoding="utf-8")
    executable.chmod(0o700)
for library in module.CAMERA_DDS_LIBRARIES:
    library.parent.mkdir(parents=True, exist_ok=True)
    library.write_bytes(b"offline library placeholder\n")
    library.chmod(0o600)
module.PARAMETERS.write_text("offline: true\n", encoding="utf-8")

config = {
    "source_mode": "local",
    "network_interface": "offline0",
    "lidar_input_topic": "/vendor/cloud",
    "lidar_output_topic": "/scanner/cloud",
    "imu_input_topic": "/vendor/imu",
    "imu_output_topic": "/scanner/imu",
    "camera_topic": "/camera/color/image_raw",
    "camera_info_topic": "/camera/color/camera_info",
    "camera_status_topic": "/go2/sensors/status",
    "camera_frame": "front_camera",
    "camera_fps": 10.0,
    "camera_ipc_socket": "/tmp/offline-go2-camera.sock",
    "sentinel_timeout_s": 30.0,
}
original_if_nametoindex = module.socket.if_nametoindex
module.socket.if_nametoindex = lambda interface: 1 if interface == "offline0" else 0
try:
    init_result = module.initialize(config)
finally:
    module.socket.if_nametoindex = original_if_nametoindex
if init_result != ("ok", None):
    raise SystemExit(f"provider initialization unexpectedly failed: {init_result!r}")
if any(os.access(path, os.X_OK) for path in module.CAMERA_DDS_LIBRARIES):
    raise SystemExit("offline test fixture unexpectedly made a DDS library executable")

result = module.activate()
if result != ("ok", None):
    raise SystemExit(f"provider activation unexpectedly failed: {result!r}")

provider = module.provider
if [entry[2] for entry in provider.spawned] != [
    "sensor-relay.log",
    "camera-daemon.log",
    "camera-bridge.log",
]:
    raise SystemExit("provider did not start all three mandatory read-only processes")

relay_env, daemon_env, bridge_env = [entry[1] for entry in provider.spawned]
if relay_env is not None or bridge_env is not None:
    raise SystemExit("vendor DDS environment leaked into a ROS child process")
expected_daemon_env = {"LD_LIBRARY_PATH": str(module.CAMERA_LIBRARY_DIR)}
if daemon_env != expected_daemon_env:
    raise SystemExit(f"camera daemon did not receive its private DDS path: {daemon_env!r}")

waited_streams = {(topic, message_type) for topic, message_type, _ in provider.waited}
expected_streams = {
    ("/scanner/cloud", "PointCloud2"),
    ("/scanner/imu", "Imu"),
    ("/camera/color/image_raw", "Image"),
    ("/camera/color/camera_info", "CameraInfo"),
}
if waited_streams != expected_streams:
    raise SystemExit(f"unexpected activation sentinels: {sorted(waited_streams)}")

expected_declarations = {
    ("robonix/primitive/lidar/lidar3d", "/scanner/cloud", "best_effort"),
    ("robonix/primitive/imu/imu", "/scanner/imu", "best_effort"),
    ("robonix/primitive/camera/rgb", "/camera/color/image_raw", "best_effort"),
}
if set(provider.declared) != expected_declarations:
    raise SystemExit(f"unexpected runtime capability declarations: {provider.declared!r}")
if any("intrinsics" in contract for contract, _, _ in provider.declared):
    raise SystemExit("uncalibrated CameraInfo was exposed as an intrinsics capability")
if len(quality_waits) != 1 or quality_waits[0][0] != "/go2/sensors/status":
    raise SystemExit(f"provider did not gate activation on camera quality: {quality_waits!r}")

# Reset only the in-memory fake lifecycle. Do not call the real process-group
# cleanup path for the deliberately synthetic FakeProcess PIDs above.
module._config = {}
module._children = []
module._declared = False
module._active = False
provider.spawned.clear()
provider.waited.clear()
provider.declared.clear()
quality_waits.clear()

# Localization/Nav2 may keep the RGB processes and capability available while
# treating a temporarily absent camera stream as non-blocking.  Lidar and IMU
# remain activation sentinels and the camera capability is still declared.
module._camera_quality_waiter = lambda topic, timeout: (_ for _ in ()).throw(
    AssertionError("optional camera quality waiter must not run")
)
optional_quality_config = dict(
    config,
    camera_required=False,
    camera_quality_required=False,
)
module.socket.if_nametoindex = lambda interface: 1 if interface == "offline0" else 0
try:
    optional_quality_init = module.initialize(optional_quality_config)
finally:
    module.socket.if_nametoindex = original_if_nametoindex
if optional_quality_init != ("ok", None):
    raise SystemExit(
        f"optional camera quality initialization failed: {optional_quality_init!r}"
    )
if module.activate() != ("ok", None):
    raise SystemExit("optional camera quality profile did not activate")
optional_waited_streams = {
    (topic, message_type) for topic, message_type, _ in provider.waited
}
if optional_waited_streams != {
    ("/scanner/cloud", "PointCloud2"),
    ("/scanner/imu", "Imu"),
}:
    raise SystemExit(
        f"optional camera profile retained camera sentinels: "
        f"{sorted(optional_waited_streams)}"
    )
if set(provider.declared) != expected_declarations:
    raise SystemExit("optional camera profile did not declare all capabilities")
module._config = {}
module._children = []
module._declared = False
module._active = False
provider.spawned.clear()
provider.waited.clear()
provider.declared.clear()
module._camera_quality_waiter = lambda topic, timeout: (
    quality_waits.append((topic, timeout)) or (True, "offline quality pass")
)

invalid_config = dict(config, source_mode="automatic")
module.socket.if_nametoindex = lambda interface: 1 if interface == "offline0" else 0
try:
    invalid_result = module.initialize(invalid_config)
finally:
    module.socket.if_nametoindex = original_if_nametoindex
if invalid_result != ("err", "source_mode must be exactly 'local' or 'external'"):
    raise SystemExit(f"invalid source mode did not fail closed: {invalid_result!r}")

# Prove that external/NX mode is only a Robonix topic registrar/sentinel. Point
# every local runtime artifact at an absent path: initialization and activation
# must still succeed without spawning a relay, camera daemon, or camera bridge.
missing_root = artifact_root / "intentionally-absent-external-runtime"
module.SENSOR_RELAY = missing_root / "go2_sensor_relay"
module.CAMERA_BRIDGE = missing_root / "go2_camera_bridge"
module.CAMERA_DAEMON = missing_root / "go2_camera_daemon"
module.CAMERA_DDS_LIBRARIES = (missing_root / "libddsc.so",)
module.PARAMETERS = missing_root / "go2_sensors.yaml"
external_config = dict(config, source_mode="external")
module.socket.if_nametoindex = lambda interface: 1 if interface == "offline0" else 0
try:
    external_init_result = module.initialize(external_config)
finally:
    module.socket.if_nametoindex = original_if_nametoindex
if external_init_result != ("ok", None):
    raise SystemExit(f"external provider initialization failed: {external_init_result!r}")

external_result = module.activate()
if external_result != ("ok", None):
    raise SystemExit(f"external provider activation failed: {external_result!r}")
if provider.spawned:
    raise SystemExit(
        "external source mode spawned a local publisher process: "
        f"{provider.spawned!r}"
    )
external_waited_streams = {
    (topic, message_type) for topic, message_type, _ in provider.waited
}
if external_waited_streams != expected_streams:
    raise SystemExit(
        "external mode did not sentinel every required standardized stream: "
        f"{sorted(external_waited_streams)}"
    )
if set(provider.declared) != expected_declarations:
    raise SystemExit(
        "external mode did not register the Robonix data capabilities: "
        f"{provider.declared!r}"
    )
if module.activate() != ("deferred", "sensor provider is already active"):
    raise SystemExit("external mode did not reject a duplicate activation")
if len(quality_waits) != 1:
    raise SystemExit("external mode did not require an explicit camera quality pass")

# A first Image sample is insufficient: quality failure must fail activation
# and must not expose any Robonix data-plane capability.
module._config = {}
module._children = []
module._declared = False
module._active = False
provider.waited.clear()
provider.declared.clear()
module._camera_quality_waiter = lambda topic, timeout: (
    False,
    "persistent API errors",
)
module.socket.if_nametoindex = lambda interface: 1 if interface == "offline0" else 0
try:
    failed_quality_init = module.initialize(external_config)
finally:
    module.socket.if_nametoindex = original_if_nametoindex
if failed_quality_init != ("ok", None):
    raise SystemExit(f"quality failure fixture did not initialize: {failed_quality_init!r}")
failed_quality_result = module.activate()
if failed_quality_result[0] != "err" or "persistent API errors" not in failed_quality_result[1]:
    raise SystemExit(f"camera quality failure did not fail closed: {failed_quality_result!r}")
if provider.declared:
    raise SystemExit("camera capabilities were declared before the quality gate passed")

print("provider manifest/runtime contract test passed")
