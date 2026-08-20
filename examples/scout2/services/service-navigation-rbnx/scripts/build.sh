#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# nav2_wrapper build phase. Runs `rbnx codegen --mcp`, then builds for the
# selected deployment target (same pattern as mapping_rbnx).
#
# Target is chosen by the per-target package manifest's `build:` line:
#   x86-docker     x86_64 + docker, ROS2+Nav2 in image (docker/Dockerfile) [default]
#   jetson-docker  arm64 Jetson + docker (same Dockerfile; ros:humble base
#                  is multi-arch, so it builds arm64 on a Jetson)
#   jetson-native  arm64 Jetson + host ROS2 — no docker; verify the host
#                  has ros-humble-nav2-bringup.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# shellcheck disable=SC1091
source "$PKG/scripts/docker_base_image.sh"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"
IMG="${ROBONIX_NAV2_IMAGE:-robonix-nav2}"
TARGET="${RBNX_BUILD_TARGET:-x86-docker}"
ROS_BASE_IMAGE="${ROBONIX_NAV2_ROS_BASE_IMAGE:-robonix-ros:humble-ros-base}"
UPSTREAM_ROS_BASE_IMAGE="ros:humble-ros-base"

if [[ "$CLEAN" == "1" ]]; then
    echo "[nav2/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/data

# ── 1. Codegen (Atlas MCP bindings only) — every target ────────────────────
if command -v rbnx >/dev/null 2>&1; then
    FLAGS=(--mcp)
    [[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
    echo "[nav2/build] rbnx codegen ${FLAGS[*]}"
    rbnx codegen -p "$PKG" "${FLAGS[@]}"
else
    echo "[nav2/build] WARNING: rbnx not in PATH — skipping proto codegen"
fi

echo "[nav2/build] target=$TARGET"

# ── 2. Per-target build ─────────────────────────────────────────────────────
case "$TARGET" in
    x86-docker|jetson-docker)
        if ! command -v docker >/dev/null 2>&1; then
            echo "[nav2/build] error: target $TARGET needs docker on PATH" >&2
            exit 1
        fi
        DOCKER_BUILD_FLAGS=(--network=host --pull=false --build-arg "ROS_BASE_IMAGE=${ROS_BASE_IMAGE}")
            robonix_ensure_local_base_image "$ROS_BASE_IMAGE" "$UPSTREAM_ROS_BASE_IMAGE"
        [[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)
        echo "[nav2/build] docker build -f docker/Dockerfile -t $IMG"
        docker build "${DOCKER_BUILD_FLAGS[@]}" -f docker/Dockerfile -t "$IMG" docker/
        ;;
    jetson-native)
        echo "[nav2/build] native target — verifying host ROS2 + nav2"
        if ! command -v ros2 >/dev/null 2>&1; then
            echo "[nav2/build] ERROR: ros2 not on PATH — source /opt/ros/humble/setup.bash" >&2
            exit 1
        fi
        # Avoid `ros2 pkg list | grep -q` under pipefail: grep exits as soon as
        # it finds a match, ros2 receives SIGPIPE, and the successful lookup is
        # incorrectly reported as a failed pipeline.
        if ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
            echo "[nav2/build] ERROR: nav2_bringup not installed. On the host run:" >&2
            echo "[nav2/build]   sudo apt install ros-humble-nav2-bringup ros-humble-navigation2" >&2
            exit 1
        fi
        if ! ros2 pkg prefix pointcloud_to_laserscan >/dev/null 2>&1; then
            echo "[nav2/build] ERROR: pointcloud_to_laserscan not installed. On the host run:" >&2
            echo "[nav2/build]   sudo apt install ros-humble-pointcloud-to-laserscan" >&2
            exit 1
        fi
        if ! ros2 pkg prefix rtabmap_util >/dev/null 2>&1; then
            echo "[nav2/build] ERROR: rtabmap_util not installed. On the host run:" >&2
            echo "[nav2/build]   sudo apt install ros-humble-rtabmap-util" >&2
            exit 1
        fi
        TERMINAL_BUILD="$PKG/rbnx-build/terminal_controller"
        echo "[nav2/build] building replan-persistent terminal plugins"
        colcon --log-base "$TERMINAL_BUILD/log" build \
            --base-paths "$PKG/terminal_controller" \
            --build-base "$TERMINAL_BUILD/build" \
            --install-base "$TERMINAL_BUILD/install"
        echo "[nav2/build] host nav2_bringup OK"
        ;;
    *)
        echo "[nav2/build] unknown RBNX_BUILD_TARGET: $TARGET (x86-docker|jetson-docker|jetson-native)" >&2
        exit 2
        ;;
esac

touch "$PKG/rbnx-build/.rbnx-built"
echo "[nav2/build] done (target=$TARGET)."
