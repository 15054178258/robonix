#!/usr/bin/env bash
set -euo pipefail

readonly overlay="/opt/robonix/overlay"
readonly navigation_overlay="/opt/robonix/navigation-overlay"
readonly deploy_root="/opt/robonix/deploy"
readonly profile_root="/opt/robonix/profile"

for package in \
  unitree_api \
  unitree_go \
  go2_chassis_adapter \
  go2_sensors \
  go2_description; do
  [[ -d "${overlay}/share/${package}" ]] || {
    echo "required passive/read-only package missing: ${package}" >&2
    exit 30
  }
done
[[ -d "${navigation_overlay}/share/robonix_nav2_terminal" ]] || {
  echo "Nav2 terminal controller overlay is missing" >&2
  exit 30
}

for package in \
  nav2_bringup \
  pointcloud_to_laserscan \
  rtabmap_slam \
  imu_filter_madgwick; do
  [[ -d "/opt/ros/humble/share/${package}" ]] || {
    echo "required ARM64 ROS package missing: ${package}" >&2
    exit 31
  }
done

for forbidden in \
  "${overlay}/bin/go2_sport_daemon" \
  "${overlay}/lib/go2_chassis_adapter/go2_sport_daemon" \
  "${deploy_root}/packages/go2_chassis/sdk_daemon" \
  "${deploy_root}/third_party/unitree_sdk2" \
  "${deploy_root}/sport_mode_ctrl" \
  "${deploy_root}/go2_sport_client" \
  "${deploy_root}/go2_stand_example" \
  "${deploy_root}/low_level_ctrl"; do
  [[ ! -e "$forbidden" ]] || {
    echo "forbidden motion artifact present: ${forbidden}" >&2
    exit 32
  }
done

# Scan only ELF payloads, not source files or comments, for linked/embedded
# Unitree motion-client surfaces.  The inherited camera reader was already
# checked by jetson-readonly; this repeats the essential absence gate across
# the combined /opt/robonix rootfs.
python3 - <<'PY'
from __future__ import annotations

import os
from pathlib import Path

root = Path("/opt/robonix")
for path in root.rglob("*"):
    if not path.is_file():
        continue
    try:
        with path.open("rb") as stream:
            magic = stream.read(4)
            if magic != b"\x7fELF":
                continue
            payload = magic + stream.read()
    except OSError:
        continue
    for token in (b"SportClient", b"/api/sport/request", b"/lowcmd"):
        if token in payload:
            raise AssertionError(
                f"forbidden Unitree motion token {token!r} in ELF {path}"
            )

for path in root.rglob("*"):
    if not path.is_file() or not os.access(path, os.X_OK):
        continue
    if path.name in {
        "sport_mode_ctrl",
        "go2_sport_client",
        "go2_stand_example",
        "low_level_ctrl",
        "go2_sport_daemon",
    }:
        raise AssertionError(f"forbidden executable in rootfs: {path}")
PY

python3 - "${profile_root}/profile.yaml" "${deploy_root}/robonix_manifest.yaml" <<'PY'
from __future__ import annotations

import sys
from pathlib import Path

import yaml

profile = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
manifest = yaml.safe_load(Path(sys.argv[2]).read_text(encoding="utf-8"))

assert profile["runtime"] == {
    "complete": False,
    "launch_allowed": False,
    "reason": "missing-verified-arm64-robonix-runtime",
    "required_artifacts": [
        {"path": "/opt/robonix/source", "status": "missing"},
        {
            "path": "/opt/robonix/models/funasr-zh-online/model.pt",
            "status": "missing",
        },
        {
            "path": "/opt/robonix/deploy/third_party/service-map-rbnx/rbnx-build/codegen",
            "status": "missing",
        },
        {
            "path": "/opt/robonix/deploy/third_party/service-navigation-rbnx/rbnx-build/codegen",
            "status": "missing",
        },
    ],
}
motion = profile["motion"]
assert motion["enabled"] is False
assert motion["chassis_sdk_daemon"] == "excluded"
assert motion["unitree_sport_client"] == "excluded"
assert motion["navigation_velocity_output"] == "/robonix/nomotion/cmd_vel"

assert manifest["env"]["GO2_ALLOW_MOTION"] == "false"
assert manifest["env"]["ROBONIX_VELOCITY_OUTPUT_TOPIC"] == "/robonix/nomotion/cmd_vel"
chassis = next(item for item in manifest["primitive"] if item["name"] == "go2_chassis")
assert chassis["config"]["allow_motion"] is False
assert chassis["config"]["twist_in_topic"] == "/robonix/nomotion/chassis_input_disabled"
services = {item["name"]: item for item in manifest["service"]}
assert services["mapping"]["manifest"] == "package_manifest.jetson-native.yaml"
assert services["navigation"]["manifest"] == "package_manifest.jetson-native.yaml"
assert services["navigation"]["config"]["velocity_output_topic"] == "/robonix/nomotion/cmd_vel"
mapping_config = services["mapping"]["config"]
dashboard_config = services["go2_dashboard"]["config"]
assert dashboard_config["pose_topic"] == "/robonix/map/pose"
assert dashboard_config["map_frame"] == "map"
assert dashboard_config["base_frame"] == mapping_config["base_frame"]
assert "scene" not in manifest["system"]

strings: list[str] = []
def collect(value: object) -> None:
    if isinstance(value, dict):
        for child in value.values():
            collect(child)
    elif isinstance(value, list):
        for child in value:
            collect(child)
    elif isinstance(value, str):
        strings.append(value)
collect(manifest)
assert "/cmd_vel" not in strings
assert "/api/sport/request" not in strings
assert "/lowcmd" not in strings
PY

for missing_artifact in \
  /opt/robonix/source \
  /opt/robonix/models/funasr-zh-online/model.pt \
  /opt/robonix/deploy/third_party/service-map-rbnx/rbnx-build/codegen \
  /opt/robonix/deploy/third_party/service-navigation-rbnx/rbnx-build/codegen; do
  [[ ! -e "$missing_artifact" ]] || {
    echo "blueprint unexpectedly contains an unverified runtime artifact: ${missing_artifact}" >&2
    exit 33
  }
done

echo "ARM64 image rootfs is a fail-closed no-motion blueprint"
