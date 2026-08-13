#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ -f "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash" ]]; then
    source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"
fi
# Source chassis workspace so scout_description meshes are on the package path
CHASSIS_WS="$PKG/../primitive-agilex-ranger_mini_v3-chassis-rbnx/rbnx-build/ws/install"
if [[ -f "$CHASSIS_WS/setup.bash" ]]; then
    source "$CHASSIS_WS/setup.bash"
fi
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG/rbnx-build/codegen/proto_gen:$PKG:${PYTHONPATH:-}"
else
    export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG:${PYTHONPATH:-}"
fi

exec python3 -m robot_description.main
