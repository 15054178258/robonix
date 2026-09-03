#!/usr/bin/env bash
set -euo pipefail

ROOT="${RBNX_PACKAGE_ROOT:-$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)}"
DEPLOY_ROOT="${ROBONIX_DEPLOY_DIR:-$(cd "${ROOT}/../.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
ROBONIX_IDL_ROOT="${ROOT}/rbnx-build/codegen/ros2_idl"
ROBONIX_IDL_SETUP="${ROBONIX_IDL_ROOT}/install/setup.bash"

if ! command -v rbnx >/dev/null 2>&1; then
  echo "rbnx is not available." >&2
  exit 1
fi
if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "Missing ${ROS_SETUP}; ROS 2 Humble is required to build the registrar overlay." >&2
  exit 1
fi
if [[ ! -r "${DEPLOY_ROOT}/scripts/build_robonix_ros2_overlay.sh" ]]; then
  echo "Missing shared Robonix ROS 2 overlay builder." >&2
  exit 1
fi

echo "[go2_d435i/build] generating Robonix ROS 2 contract bindings"
rbnx codegen -p "${ROOT}" --ros2
# shellcheck disable=SC1090
source "${DEPLOY_ROOT}/scripts/build_robonix_ros2_overlay.sh"
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
echo "[go2_d435i/build] done"
