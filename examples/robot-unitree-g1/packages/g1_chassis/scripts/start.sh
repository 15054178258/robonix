#!/usr/bin/env bash
# Robonix entry point. Driver(DMD_INIT) config is consumed by the provider.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$PKG"

# FastRTPS is the default ROS 2 RMW for Humble and must be active.
if [ "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" != "rmw_fastrtps_cpp" ]; then
  echo "G1 chassis requires RMW_IMPLEMENTATION=rmw_fastrtps_cpp." >&2
  exit 2
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

set +u
source /opt/ros/humble/setup.bash
source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"
source "$PKG/rbnx-build/ros/install/setup.bash"
set -u

# Use a stable IPC socket path (matches g1_chassis Python fallback).
export G1_IPC_SOCKET="${G1_IPC_SOCKET:-$HOME/.robonix/g1_chassis.sock}"
mkdir -p "${G1_IPC_SOCKET%/*}"

export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"

exec python3.10 -m g1_chassis.main
