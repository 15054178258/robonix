#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
BUILD_ROOT="${GO2_SENSORS_BUILD_ROOT:-${ROOT}/.build/ros}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
LOCAL_SETUP="${BUILD_ROOT}/install/setup.bash"

if [[ ! -r "${ROS_SETUP}" || ! -r "${LOCAL_SETUP}" ]]; then
  echo "ROS 2 or the local go2_sensors build is not available." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${LOCAL_SETUP}"
set -u

echo "============================================================"
echo " READ-ONLY INPUTS: relaying lidar/IMU/camera sensor data only"
echo " This process does not change network configuration."
echo " Stop with Ctrl-C"
echo "============================================================"
exec ros2 launch go2_sensors go2_sensors.launch.py "$@"
