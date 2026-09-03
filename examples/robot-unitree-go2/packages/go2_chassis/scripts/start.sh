#!/usr/bin/env bash
# Robonix entry point. Driver(CMD_INIT) is the only source of per-instance
# config, so this script starts only the provider. Its on_init uses
# Primitive.spawn() for the adapter and (only after all gates) SDK daemon.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$PKG"

if [ "${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" != "rmw_cyclonedds_cpp" ]; then
  echo "Go2 requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp." >&2
  exit 2
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

set +u
source /opt/ros/humble/setup.bash
if [ -n "${UNITREE_ROS2_SETUP:-}" ] && [ -f "$UNITREE_ROS2_SETUP" ]; then
  source "$UNITREE_ROS2_SETUP"
else
  echo "UNITREE_ROS2_SETUP must point to the built unitree_go overlay." >&2
  exit 2
fi
source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"
source "$PKG/rbnx-build/ros/install/setup.bash"
set -u
if [ -z "${CYCLONEDDS_URI:-}" ]; then
  echo "CYCLONEDDS_URI must be bound to the approved Go2 network interface before startup." >&2
  exit 2
fi
export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"
exec python3 -m go2_chassis.main
