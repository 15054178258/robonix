#!/usr/bin/env bash
# Build generated Robonix interfaces without replacing ROS distribution
# interface packages. This file is sourced by package build scripts.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "source this helper from a package build script" >&2
  exit 2
fi

readonly -a ROBONIX_ROS2_SYSTEM_INTERFACE_PACKAGES=(
  actionlib_msgs
  builtin_interfaces
  composition_interfaces
  diagnostic_msgs
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  rcl_interfaces
  rosgraph_msgs
  sensor_msgs
  shape_msgs
  statistics_msgs
  std_msgs
  std_srvs
  stereo_msgs
  test_msgs
  trajectory_msgs
  visualization_msgs
)

robonix_build_ros2_overlay() {
  local idl_root="$1"
  shift

  [[ -d "$idl_root/src" ]] || {
    echo "generated Robonix ROS 2 source tree is missing: $idl_root/src" >&2
    return 2
  }
  command -v colcon >/dev/null 2>&1 || {
    echo "colcon is required to build the Robonix ROS 2 overlay" >&2
    return 2
  }

  # Codegen emits a self-contained interface workspace, including reduced
  # copies of ROS distribution packages such as sensor_msgs. Those generated
  # copies contain only IDL and do not export support targets (for example
  # sensor_msgs_library) expected by cv_bridge and other Humble packages.
  # Remove an older full overlay only when it actually contains a generated
  # system package. Safe custom-only overlays stay incremental on later builds.
  local package
  for package in "${ROBONIX_ROS2_SYSTEM_INTERFACE_PACKAGES[@]}"; do
    if [[ -e "$idl_root/build/$package" || -L "$idl_root/build/$package" ||
          -e "$idl_root/install/$package" || -L "$idl_root/install/$package" ||
          -e "$idl_root/install/share/$package" ||
          -L "$idl_root/install/share/$package" ]]; then
      echo "removing stale generated ROS system-interface overlay: $package" >&2
      rm -rf -- "$idl_root/build" "$idl_root/install" "$idl_root/log"
      break
    fi
  done

  colcon --log-base "$idl_root/log" build \
    --base-paths "$idl_root" \
    --build-base "$idl_root/build" \
    --install-base "$idl_root/install" \
    --packages-skip "${ROBONIX_ROS2_SYSTEM_INTERFACE_PACKAGES[@]}" \
    "$@"
}
