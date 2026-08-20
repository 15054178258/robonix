#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

NATIVE_PLATFORMS=("jetson_orin")
is_native_platform() {
    local p="$1"
    for w in "${NATIVE_PLATFORMS[@]}"; do
        [[ "$p" == "$w" ]] && return 0
    done
    return 1
}

MODE=""
case "${ROBONIX_NAV2_FORCE:-}" in
    native) MODE=native ;;
    docker) MODE=docker ;;
    "") ;;
    *) echo "[nav2/stop] ROBONIX_NAV2_FORCE=${ROBONIX_NAV2_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac
if [[ -z "$MODE" ]]; then
    if is_native_platform "${ROBONIX_NAV2_PLATFORM:-}"; then MODE=native; else MODE=docker; fi
fi

echo "[nav2/stop] mode=${MODE}"
if [[ "$MODE" == "docker" ]]; then
    docker rm -f "${ROBONIX_NAV2_CONTAINER:-robonix_nav2}" >/dev/null 2>&1 || true
    exit 0
fi

pkill -TERM -f "${PKG}.*nav2_wrapper.atlas_bridge" 2>/dev/null || true
pkill -TERM -f "robonix_pointcloud_to_laserscan|pointcloud_to_laserscan_node|nav2_wrapper.scan_filter|nav2_wrapper.velocity_guard" 2>/dev/null || true
pkill -TERM -f "nav2_bringup|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother" 2>/dev/null || true
sleep 1
pkill -KILL -f "${PKG}.*nav2_wrapper.atlas_bridge" 2>/dev/null || true
pkill -KILL -f "robonix_pointcloud_to_laserscan|pointcloud_to_laserscan_node|nav2_wrapper.scan_filter|nav2_wrapper.velocity_guard" 2>/dev/null || true
pkill -KILL -f "nav2_bringup|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother" 2>/dev/null || true
