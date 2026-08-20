#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# nav2_wrapper native (no-docker) launcher. Runs the atlas bridge as a host
# process against a host-installed ROS2 + Nav2. Picked by scripts/start.sh
# when ROBONIX_NAV2_FORCE=native (or ROBONIX_NAV2_PLATFORM matches the
# native whitelist). The bridge spawns `ros2 launch nav2_bringup
# navigation_launch.py …` inside Driver(CMD_INIT).
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "[nav2-native] ERR: ROS 2 not found at /opt/ros/${ROS_DISTRO}" >&2
    echo "[nav2-native]      set ROBONIX_NAV2_FORCE=docker to use the container path" >&2
    exit 2
fi
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
if [[ -f "$PKG/rbnx-build/terminal_controller/install/setup.bash" ]]; then
    set +u; source "$PKG/rbnx-build/terminal_controller/install/setup.bash"; set -u
fi
if ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
    echo "[nav2-native] ERR: nav2_bringup not installed on the host." >&2
    echo "[nav2-native]      sudo apt install ros-humble-nav2-bringup ros-humble-navigation2" >&2
    echo "[nav2-native]      (or ROBONIX_NAV2_FORCE=docker)" >&2
    exit 2
fi
if ! ros2 pkg prefix rtabmap_util >/dev/null 2>&1; then
    echo "[nav2-native] ERR: rtabmap_util not installed on the host." >&2
    echo "[nav2-native]      sudo apt install ros-humble-rtabmap-util" >&2
    exit 2
fi

export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
if ROBONIX_PY="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_PY:$PYTHONPATH"
fi

mkdir -p "$PKG/rbnx-build/data"
exec python3 -m nav2_wrapper.atlas_bridge
