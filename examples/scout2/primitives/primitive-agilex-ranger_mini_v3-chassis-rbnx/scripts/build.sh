#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build phase: colcon-build the vendored ranger_ros2 + ugv_sdk, then
# rbnx codegen so atlas_bridge can import atlas_pb2 + lifecycle_pb2.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[ranger_chassis/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/ws/src rbnx-build/data

ln -snf "$PKG/src/ranger_ros2" "$PKG/rbnx-build/ws/src/ranger_ros2"
ln -snf "$PKG/src/scout_ros2"  "$PKG/rbnx-build/ws/src/scout_ros2"
ln -snf "$PKG/src/ugv_sdk"     "$PKG/rbnx-build/ws/src/ugv_sdk"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

echo "[ranger_chassis/build] colcon build (ranger_ros2 + scout_ros2 + ugv_sdk)"
cd "$PKG/rbnx-build/ws"
colcon build --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cd "$PKG"

FLAGS=(--ros2 --out-dir "$PKG/rbnx-build/codegen")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[ranger_chassis/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

ROS2_IDL="$PKG/rbnx-build/codegen/ros2_idl"
echo "[ranger_chassis/build] colcon build (Robonix ROS 2 interfaces)"
(cd "$ROS2_IDL" && colcon build)

touch "$PKG/rbnx-build/.rbnx-built"
echo "[ranger_chassis/build] done."
