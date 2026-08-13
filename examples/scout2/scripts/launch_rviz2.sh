#!/usr/bin/env bash
# Launch RViz2 with all Scout2 workspaces sourced so mesh paths resolve.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SCOUT2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

# Chassis workspace — provides the scout_description package (meshes)
CHASSIS_WS="$SCOUT2_DIR/primitives/primitive-agilex-ranger_mini_v3-chassis-rbnx/rbnx-build/ws/install"
if [[ -f "$CHASSIS_WS/setup.bash" ]]; then
    source "$CHASSIS_WS/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec rviz2 "$@"
