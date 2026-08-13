#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build only Mapping's Robonix-specific ROS 2 interface package.
#
# `rbnx codegen --ros2` emits a self-contained source tree which also includes
# generated copies of standard ROS packages such as sensor_msgs.  Installing
# those copies over a system Humble installation can hide distro-provided
# support libraries and CMake exports (for example sensor_msgs_library, which
# cv_bridge expects).  Mapping only consumes map/msg/MapLifecycle from this
# tree, so keep the deployment overlay deliberately narrow.
set -euo pipefail

ROS2_IDL="${1:?usage: build_ros2_overlay.sh ROS2_IDL_DIRECTORY}"

if [[ ! -d "$ROS2_IDL/src/map" ]]; then
    echo "[ros2-overlay] ERROR: generated map package missing: $ROS2_IDL/src/map" >&2
    exit 1
fi

# A previous unscoped build may have left generated standard packages in the
# install tree. Selecting map does not prune those stale artifacts. Clean only
# when such an old install is detected; ordinary map-only builds stay
# incremental.
SYSTEM_INTERFACE_PACKAGES=(
    action_msgs actionlib_msgs builtin_interfaces composition_interfaces
    diagnostic_msgs geometry_msgs lifecycle_msgs nav_msgs rcl_interfaces
    rosgraph_msgs sensor_msgs shape_msgs statistics_msgs std_msgs std_srvs
    stereo_msgs test_msgs trajectory_msgs unique_identifier_msgs
    visualization_msgs
)
for package in "${SYSTEM_INTERFACE_PACKAGES[@]}"; do
    if [[ -e "$ROS2_IDL/install/$package" ||
          -e "$ROS2_IDL/install/share/$package" ]]; then
        echo "[ros2-overlay] stale generated $package detected; reset build/install/log"
        rm -rf -- "$ROS2_IDL/build" "$ROS2_IDL/install" "$ROS2_IDL/log"
        break
    fi
done

echo "[ros2-overlay] colcon build --packages-select map"
cd "$ROS2_IDL"
colcon build --packages-select map
