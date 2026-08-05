#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
if [[ -f "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash" ]]; then
    set +u; source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"; set -u
fi
# Robonix codegen may provide its own setup.bash for generated Python paths.
# The colcon-generated local_setup.bash is the authoritative ROS package
# overlay and must be sourced first so ranger_bringup is discoverable.
for cand in "$PKG/rbnx-build/ws/install/local_setup.bash" "$PKG/rbnx-build/ws/install/setup.bash"; do
    if [[ -f "$cand" ]]; then
        # shellcheck disable=SC1091
        set +u; source "$cand"; set -u
        break
    fi
done

if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
fi

exec python3 -m ranger_chassis.main
