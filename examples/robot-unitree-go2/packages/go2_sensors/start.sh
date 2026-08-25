#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
ROBONIX_IDL_SETUP="${ROOT}/rbnx-build/codegen/ros2_idl/install/setup.bash"
LOCAL_SETUP="${GO2_SENSORS_BUILD_ROOT:-${ROOT}/.build/ros}/install/setup.bash"
PROTO_STUB="${ROOT}/rbnx-build/codegen/proto_gen/atlas_pb2.py"
SENSOR_SOURCE_MODE="${GO2_SENSOR_SOURCE_MODE:-local}"

case "${SENSOR_SOURCE_MODE}" in
  local|external) ;;
  *)
    echo "GO2_SENSOR_SOURCE_MODE must be exactly 'local' or 'external'." >&2
    exit 1
    ;;
esac

if [[ ! -r "${ROS_SETUP}" || ! -r "${ROBONIX_IDL_SETUP}" ]]; then
  echo "ROS 2 or the Robonix message overlay is unavailable." >&2
  exit 1
fi
if [[ "${SENSOR_SOURCE_MODE}" == local && ! -r "${LOCAL_SETUP}" ]]; then
  echo "The local sensor build is unavailable in local source mode." >&2
  exit 1
fi
if [[ ! -r "${PROTO_STUB}" ]]; then
  echo "Robonix codegen output is unavailable: ${PROTO_STUB}" >&2
  echo "Run this package's build command before start." >&2
  exit 1
fi
if ! command -v rbnx >/dev/null 2>&1; then
  echo "rbnx is not available." >&2
  exit 1
fi
ROBONIX_API_ROOT="$(rbnx path robonix-api)"
if [[ ! -d "${ROBONIX_API_ROOT}/robonix_api" ]]; then
  echo "rbnx returned an invalid robonix-api path: ${ROBONIX_API_ROOT}" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${ROBONIX_IDL_SETUP}"
if [[ "${SENSOR_SOURCE_MODE}" == local ]]; then
  # shellcheck disable=SC1090
  source "${LOCAL_SETUP}"
fi
set -u

export PYTHONPATH="${ROBONIX_API_ROOT}:${ROOT}:${ROOT}/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"

echo "============================================================"
echo " READ-ONLY GO2 SENSOR PROVIDER"
echo " No chassis or robot-control interface is opened by this package."
if [[ "${SENSOR_SOURCE_MODE}" == external ]]; then
  echo " EXTERNAL/NX MODE: no local relay, camera daemon, or bridge is started."
else
  echo " LOCAL MODE: this provider owns the relay and camera publishers."
fi
echo "============================================================"
exec python3 -m go2_sensors_provider.main
