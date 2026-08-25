#!/usr/bin/env bash
# Start the ROS-only process for local, motion-disabled diagnostics only.
# Motion-capable startup must go through Driver(CMD_INIT), whose provider
# validates every gate and injects the read-only-audited modes.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
INSTALL="${GO2_ROS_INSTALL:-$PKG/rbnx-build/ros/install}"
SOCKET="${GO2_SDK_SOCKET:-/tmp/robonix-go2-disabled.sock}"
ALLOW_MOTION_RAW="${GO2_ALLOW_MOTION:-false}"

case "${ALLOW_MOTION_RAW,,}" in
  1|true|yes|on) ALLOW_MOTION=1 ;;
  0|false|no|off|"") ALLOW_MOTION=0 ;;
  *)
    echo "GO2_ALLOW_MOTION must be an explicit boolean." >&2
    exit 3
    ;;
esac

if [ "$ALLOW_MOTION" = "1" ]; then
  echo "Refusing direct motion-enabled adapter startup." >&2
  echo "Use Robonix Driver(CMD_INIT) so all gates and audited modes are validated." >&2
  exit 3
fi

if [ "${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" != "rmw_cyclonedds_cpp" ]; then
  echo "Go2 requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp." >&2
  exit 2
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble setup is unavailable." >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
if [ -z "${UNITREE_ROS2_SETUP:-}" ] || [ ! -f "$UNITREE_ROS2_SETUP" ]; then
  echo "UNITREE_ROS2_SETUP must point to the built unitree_go overlay." >&2
  exit 2
fi
source "$UNITREE_ROS2_SETUP"
source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"
if [ -f "$INSTALL/setup.bash" ]; then
  source "$INSTALL/setup.bash"
else
  echo "Adapter install setup is unavailable: $INSTALL/setup.bash" >&2
  exit 2
fi
set -u
if [ -z "${CYCLONEDDS_URI:-}" ]; then
  echo "CYCLONEDDS_URI must be bound to the approved Go2 network interface." >&2
  exit 2
fi

echo "============================================================"
echo "GO2 ROS ADAPTER: READ-ONLY / MOTION DISABLED"
echo "This manual wrapper cannot enable motion."
echo "============================================================"

exec ros2 run go2_chassis_adapter go2_chassis_adapter_node \
  --ros-args \
  --params-file "$PKG/config/adapter.yaml" \
  -p "allow_motion:=false" \
  -p "allowed_modes:=[255]" \
  -p "sdk_socket:=$SOCKET"
