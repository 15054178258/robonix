#!/usr/bin/env bash
# Compile and test the GID-guard relay.  This script starts no ROS node and
# opens no network connection; colcon/ctest only build and run offline code.
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEPLOY_ROOT="${ROBONIX_DEPLOY_DIR:-$(cd "$ROOT/../.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
UNITREE_SETUP="${UNITREE_ROS2_SETUP:-$DEPLOY_ROOT/rbnx-build/unitree_ros2/install/setup.bash}"
BUILD_ROOT="${GO2_MOTION_STATE_RELAY_BUILD_ROOT:-$ROOT/.build/ros}"

[[ -r "$ROS_SETUP" ]] || {
  echo "Missing $ROS_SETUP; ROS 2 Humble is required to compile the relay." >&2
  exit 1
}
[[ -r "$UNITREE_SETUP" ]] || {
  echo "Missing Unitree ROS 2 message overlay: $UNITREE_SETUP" >&2
  exit 1
}
command -v colcon >/dev/null 2>&1 || {
  echo "colcon is not available." >&2
  exit 1
}

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$UNITREE_SETUP"
set -u

mkdir -p "$BUILD_ROOT"
colcon --log-base "$BUILD_ROOT/log" build \
  --base-paths "$ROOT" \
  --build-base "$BUILD_ROOT/build" \
  --install-base "$BUILD_ROOT/install" \
  --packages-select go2_motion_state_relay \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
colcon --log-base "$BUILD_ROOT/test-log" test \
  --build-base "$BUILD_ROOT/build" \
  --install-base "$BUILD_ROOT/install" \
  --packages-select go2_motion_state_relay
colcon test-result --test-result-base "$BUILD_ROOT/build" --verbose

WORKER="$BUILD_ROOT/install/go2_motion_state_relay/lib/go2_motion_state_relay/workstation_motion_state_relay"
[[ -x "$WORKER" ]] || {
  echo "relay build produced no executable: $WORKER" >&2
  exit 1
}
