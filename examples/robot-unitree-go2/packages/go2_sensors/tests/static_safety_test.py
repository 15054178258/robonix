#!/usr/bin/env python3
import ast
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
ROS_FILES = [ROOT / "src" / "sensor_relay_node.cpp", ROOT / "src" / "camera_bridge_node.cpp"]
DAEMON_FILE = ROOT / "camera_daemon" / "src" / "camera_daemon.cpp"
PROVIDER_FILE = ROOT / "go2_sensors_provider" / "main.py"
STAMP_GUARD_FILE = ROOT / "include" / "go2_sensors" / "stamp_guard.hpp"
LATEST_FRAME_FILE = ROOT / "include" / "go2_sensors" / "latest_frame_mailbox.hpp"
ERROR_WATERMARK_FILE = ROOT / "include" / "go2_sensors" / "camera_error_watermark.hpp"
JPEG_DECODER_FILES = [
    ROOT / "include" / "go2_sensors" / "strict_jpeg_decoder.hpp",
    ROOT / "src" / "strict_jpeg_decoder.cpp",
]
PRODUCTION_FILES = (
    ROS_FILES
    + [
        DAEMON_FILE,
        PROVIDER_FILE,
        STAMP_GUARD_FILE,
        LATEST_FRAME_FILE,
        ERROR_WATERMARK_FILE,
    ]
    + JPEG_DECODER_FILES
)


def fail(message: str) -> None:
    print(f"FAILED: {message}", file=sys.stderr)
    raise SystemExit(1)


for path in PRODUCTION_FILES:
    if not path.is_file():
        fail(f"missing production source: {path.relative_to(ROOT)}")

forbidden = (
    r"sport(?:_|/)?client",
    r"geometry_msgs",
    r"cmd_vel",
    r"lowcmd",
    r"api/sport/request",
    r"\.(?:move|stopmove)\s*\(",
)
for path in PRODUCTION_FILES:
    text = path.read_text(encoding="utf-8").lower()
    for pattern in forbidden:
        if re.search(pattern, text):
            fail(f"forbidden control surface {pattern!r} in {path.relative_to(ROOT)}")

for path in ROS_FILES:
    if "unitree/" in path.read_text(encoding="utf-8"):
        fail(f"ROS process imports Unitree SDK: {path.relative_to(ROOT)}")

daemon_text = DAEMON_FILE.read_text(encoding="utf-8")
for forbidden_ros_dependency in ("rclcpp", "sensor_msgs", "cv_bridge"):
    if forbidden_ros_dependency in daemon_text:
        fail(f"SDK daemon imports ROS dependency: {forbidden_ros_dependency}")
if "go2/video/video_client.hpp" not in daemon_text or "GetImageSample" not in daemon_text:
    fail("SDK daemon does not use the read-only video API")
unitree_headers = set(re.findall(r'#include\s+"unitree/([^"]+)"', daemon_text))
expected_unitree_headers = {
    "robot/channel/channel_factory.hpp",
    "robot/go2/video/video_client.hpp",
}
if unitree_headers != expected_unitree_headers:
    fail(f"SDK daemon Unitree include boundary changed: {sorted(unitree_headers)}")

ros_cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
if "unitree_sdk2" in ros_cmake:
    fail("ROS CMake target links Unitree SDK")
for jpeg_contract in ("find_package(JPEG REQUIRED)", "JPEG::JPEG", "strict_jpeg_decoder.cpp"):
    if jpeg_contract not in ros_cmake:
        fail(f"strict JPEG decoder build contract is missing: {jpeg_contract}")
daemon_cmake = (ROOT / "camera_daemon" / "CMakeLists.txt").read_text(encoding="utf-8")
for ros_dependency in ("rclcpp", "sensor_msgs", "cv_bridge", "ament"):
    if ros_dependency in daemon_cmake:
        fail(f"SDK daemon CMake depends on ROS: {ros_dependency}")
for isolation_contract in (
    'INSTALL_RPATH "\\$ORIGIN/../lib"',
    "INSTALL_RPATH_USE_LINK_PATH FALSE",
    "libddsc.so libddsc.so.0 libddscxx.so libddscxx.so.0",
    "COMPONENT go2_camera_runtime",
):
    if isolation_contract not in daemon_cmake:
        fail(f"SDK daemon private runtime contract is missing: {isolation_contract}")

try:
    package = ET.parse(ROOT / "package.xml").getroot()
except (ET.ParseError, OSError) as error:
    fail(f"invalid package.xml: {error}")
if package.findtext("name") != "go2_sensors":
    fail("unexpected package name")

try:
    import yaml
except ImportError as error:
    fail(f"PyYAML is required to validate package_manifest.yaml: {error}")
try:
    manifest = yaml.safe_load((ROOT / "package_manifest.yaml").read_text(encoding="utf-8"))
except (OSError, yaml.YAMLError) as error:
    fail(f"invalid package_manifest.yaml: {error}")
allowed_manifest_keys = {
    "manifestVersion", "package", "build", "start", "stop", "capabilities", "depends"
}
unknown_manifest_keys = set(manifest) - allowed_manifest_keys
if unknown_manifest_keys:
    fail(f"package manifest contains fields ignored by rbnx: {sorted(unknown_manifest_keys)}")
expected_capabilities = {
    "robonix/primitive/lidar/driver",
    "robonix/primitive/lidar/lidar3d",
    "robonix/primitive/imu/imu",
    "robonix/primitive/camera/rgb",
}
actual_capabilities = {entry["name"] for entry in manifest.get("capabilities", [])}
if actual_capabilities != expected_capabilities:
    fail(f"unexpected capability IDs: {sorted(actual_capabilities)}")

provider_text = PROVIDER_FILE.read_text(encoding="utf-8")
if 'Primitive(id="go2_sensors", namespace="robonix/primitive/lidar")' not in provider_text:
    fail("provider ID/driver namespace no longer match deployment and lidar driver contract")
try:
    provider_tree = ast.parse(provider_text, filename=str(PROVIDER_FILE))
except SyntaxError as error:
    fail(f"invalid provider Python: {error}")
data_capabilities = None
for statement in provider_tree.body:
    if isinstance(statement, ast.Assign) and any(
        isinstance(target, ast.Name) and target.id == "DATA_CAPABILITIES"
        for target in statement.targets
    ):
        try:
            data_capabilities = ast.literal_eval(statement.value)
        except (ValueError, TypeError) as error:
            fail(f"DATA_CAPABILITIES must remain a literal table: {error}")
        break
if data_capabilities is None:
    fail("provider has no literal DATA_CAPABILITIES table")
runtime_capabilities = {entry[0] for entry in data_capabilities}
if runtime_capabilities != expected_capabilities - {"robonix/primitive/lidar/driver"}:
    fail(f"manifest/runtime capability mismatch: {sorted(runtime_capabilities)}")
if "require_camera" in provider_text or "camera_calibrated" in provider_text:
    fail("Robonix provider must not make a manifest capability optional at runtime")
if "robonix/primitive/camera/intrinsics" in provider_text:
    fail("uncalibrated CameraInfo must not be advertised as an intrinsics capability")

relay_text = ROS_FILES[0].read_text(encoding="utf-8")
for freshness_contract in (
    'declare_stamp_policy("lidar", 0.50, 0.05)',
    'declare_stamp_policy("imu", 0.20, 0.05)',
    'prefix + ".max_stamp_age_seconds"',
    'prefix + ".max_future_stamp_offset_seconds"',
    "evaluate_header_stamp",
    "stamp_policy_within_limits",
    '" stamp freshness thresholds may only be tightened; "',
    "rejected_zero_stamp_count",
    "rejected_stale_stamp_count",
    "rejected_future_stamp_count",
    "rejected_invalid_stamp_count",
):
    if freshness_contract not in relay_text:
        fail(f"sensor timestamp rejection contract is missing: {freshness_contract}")
if relay_text.count("if (!accept_fresh_stamp(") != 2:
    fail("both PointCloud2 and Imu relays must gate publication on a fresh header stamp")

camera_bridge_text = ROS_FILES[1].read_text(encoding="utf-8")
for latest_frame_contract in (
    "LatestFrameMailbox latest_frame_",
    "processor_ = std::thread",
    "latest_frame_.put(std::move(frame))",
    "state_.last_error.clear_if_recovered_by(",
    "qos.keep_last(1)",
    '"superseded_count"',
    '"pending_frame_depth"',
):
    if latest_frame_contract not in camera_bridge_text:
        fail(f"camera latest-frame backpressure contract is missing: {latest_frame_contract}")

sensor_config = yaml.safe_load((ROOT / "config" / "go2_sensors.yaml").read_text(encoding="utf-8"))
relay_config = sensor_config["go2_sensor_relay"]["ros__parameters"]
expected_freshness = {
    ("lidar", "max_stamp_age_seconds"): 0.50,
    ("lidar", "max_future_stamp_offset_seconds"): 0.05,
    ("imu", "max_stamp_age_seconds"): 0.20,
    ("imu", "max_future_stamp_offset_seconds"): 0.05,
}
for (stream, parameter), expected in expected_freshness.items():
    actual = relay_config[stream].get(parameter)
    if actual != expected:
        fail(f"unexpected {stream}.{parameter}: {actual!r}")

build_text = (ROOT / "build.sh").read_text(encoding="utf-8")
if 'rbnx codegen -p "${ROOT}" --ros2' not in build_text:
    fail("build does not generate the canonical Robonix ROS 2 message overlay")
ros_build_text = (ROOT / "scripts" / "build_ros.sh").read_text(encoding="utf-8")
canonical_idl_setup = "rbnx-build/codegen/ros2_idl/install/setup.bash"
if canonical_idl_setup not in ros_build_text:
    fail("ROS build does not source the canonical Robonix message overlay")
start_text = (ROOT / "start.sh").read_text(encoding="utf-8")
if "rbnx path robonix-api" not in start_text:
    fail("start does not add the canonical robonix-api path")
if canonical_idl_setup not in start_text:
    fail("start does not source the canonical Robonix message overlay")
if "LD_LIBRARY_PATH" in start_text:
    fail("provider start must not export the Unitree DDS library path")

daemon_build_text = (ROOT / "scripts" / "build_camera_daemon.sh").read_text(
    encoding="utf-8"
)
for build_contract in (
    "-DBUILD_EXAMPLES=OFF",
    "-DCMAKE_INSTALL_PREFIX=",
    "--target go2_camera_daemon",
    "--component go2_camera_runtime",
    '${INSTALL_ROOT}/bin/go2_camera_daemon',
    '${INSTALL_ROOT}/lib/libddsc.so.0',
    '${INSTALL_ROOT}/lib/libddscxx.so.0',
):
    if build_contract not in daemon_build_text:
        fail(f"camera daemon build isolation is missing: {build_contract}")

if 'CAMERA_INSTALL / "bin" / "go2_camera_daemon"' not in provider_text:
    fail("provider does not use the privately installed camera daemon")
if 'child_env={"LD_LIBRARY_PATH": str(CAMERA_LIBRARY_DIR)}' not in provider_text:
    fail("camera daemon child does not receive its private DDS library path")
if re.search(r"os\.environ\s*\[\s*[\"']LD_LIBRARY_PATH", provider_text):
    fail("provider mutates its own LD_LIBRARY_PATH")
capability_doc = (ROOT / "CAPABILITY.md").read_text(encoding="utf-8")
if not capability_doc.startswith("---\ndescription:"):
    fail("CAPABILITY.md lacks the Pilot discovery description frontmatter")

print("static process-boundary and no-control checks passed")
