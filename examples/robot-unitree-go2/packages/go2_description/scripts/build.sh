#!/usr/bin/env bash
set -euo pipefail

ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DEPLOY_ROOT="${ROBONIX_DEPLOY_DIR:-$(cd "$ROOT/../.." && pwd)}"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
DESCRIPTION_BUILD="$ROOT/rbnx-build/description"
# shellcheck disable=SC1091
source "$DEPLOY_ROOT/scripts/build_robonix_ros2_overlay.sh"

command -v rbnx >/dev/null 2>&1 || { echo "rbnx is required" >&2; exit 1; }
command -v colcon >/dev/null 2>&1 || { echo "colcon is required" >&2; exit 1; }
[[ -r "$ROS_SETUP" ]] || { echo "missing $ROS_SETUP" >&2; exit 1; }

rbnx codegen -p "$ROOT" --ros2

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

IDL_ROOT="$ROOT/rbnx-build/codegen/ros2_idl"
robonix_build_ros2_overlay "$IDL_ROOT" \
  --packages-select lifecycle \
  --merge-install

set +u
# shellcheck disable=SC1090
source "$IDL_ROOT/install/setup.bash"
set -u

colcon --log-base "$DESCRIPTION_BUILD/log" build \
  --base-paths "$ROOT" \
  --build-base "$DESCRIPTION_BUILD/build" \
  --install-base "$DESCRIPTION_BUILD/install" \
  --packages-select go2_description \
  --merge-install

ros2 pkg prefix robot_state_publisher >/dev/null
ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null
PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover -s "$ROOT/tests" -p 'test_*.py' -v
