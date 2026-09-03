#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEPLOY_ROOT="${ROBONIX_DEPLOY_DIR:-$(cd "$ROOT/../.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
BUILD_ROOT="${GO2_SENSORS_BUILD_ROOT:-${ROOT}/.build/ros}"
ROBONIX_IDL_ROOT="${ROOT}/rbnx-build/codegen/ros2_idl"
ROBONIX_IDL_SETUP="${ROOT}/rbnx-build/codegen/ros2_idl/install/setup.bash"
# shellcheck disable=SC1091
source "$DEPLOY_ROOT/scripts/build_robonix_ros2_overlay.sh"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "Missing ${ROS_SETUP}; install/source ROS 2 Humble before building." >&2
  exit 1
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is not available." >&2
  exit 1
fi
if [[ ! -d "${ROBONIX_IDL_ROOT}" ]]; then
  echo "Missing Robonix ROS 2 IDL source overlay: ${ROBONIX_IDL_ROOT}" >&2
  echo "Run 'rbnx codegen -p ${ROOT} --ros2' first, or use ${ROOT}/build.sh." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

robonix_build_ros2_overlay "${ROBONIX_IDL_ROOT}" \
  --packages-select lifecycle
if [[ ! -r "${ROBONIX_IDL_SETUP}" ]]; then
  echo "Robonix ROS 2 IDL overlay did not produce ${ROBONIX_IDL_SETUP}" >&2
  exit 1
fi
set +u
# shellcheck disable=SC1090
source "${ROBONIX_IDL_SETUP}"
set -u

mkdir -p "${BUILD_ROOT}"
colcon --log-base "${BUILD_ROOT}/log" build \
  --base-paths "${ROOT}" \
  --build-base "${BUILD_ROOT}/build" \
  --install-base "${BUILD_ROOT}/install" \
  --packages-select go2_sensors \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
