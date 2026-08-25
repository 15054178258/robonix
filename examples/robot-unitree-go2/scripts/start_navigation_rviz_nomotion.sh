#!/usr/bin/env bash
set -euo pipefail
umask 077

readonly ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly RVIZ_CONFIG="$ROOT/config/go2_navigation_nomotion.rviz"
readonly ROS_SETUP="${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
readonly UNITREE_SETUP="${UNITREE_ROS2_SETUP:-$ROOT/rbnx-build/unitree_ros2/install/setup.bash}"
readonly INTERFACE="${GO2_NETWORK_INTERFACE:-}"

if [[ "$#" -ne 0 ]]; then
  echo "this launcher accepts no arguments" >&2
  exit 2
fi
for path in "$RVIZ_CONFIG" "$ROS_SETUP" "$UNITREE_SETUP"; do
  [[ -f "$path" && ! -L "$path" ]] || {
    echo "required RViz component must be a regular non-symlink file: $path" >&2
    exit 2
  }
done
[[ -n "$INTERFACE" ]] || {
  echo "GO2_NETWORK_INTERFACE is required; pass the same existing interface used by the active stack" >&2
  exit 2
}
[[ "$INTERFACE" =~ ^[A-Za-z0-9_.:-]+$ ]] || {
  echo "GO2_NETWORK_INTERFACE contains unsupported characters" >&2
  exit 2
}
[[ -d "/sys/class/net/$INTERFACE" ]] || {
  echo "GO2_NETWORK_INTERFACE does not exist: $INTERFACE" >&2
  exit 2
}

# This is a process-local DDS binding only. It neither changes addresses,
# routes, DNS, Wi-Fi profiles, nor any other host network configuration.
readonly OWNED_CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$INTERFACE\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
if [[ -n "${CYCLONEDDS_URI:-}" && "$CYCLONEDDS_URI" != "$OWNED_CYCLONEDDS_URI" ]]; then
  echo "inherited CYCLONEDDS_URI does not match the selected Go2 interface" >&2
  exit 2
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$OWNED_CYCLONEDDS_URI"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( ROS_DOMAIN_ID <= 232 )) || {
  echo "ROS_DOMAIN_ID must be an integer in 0..232" >&2
  exit 2
}

# RViz receives telemetry only. These values make the intended launch context
# explicit and cannot enable the chassis or bypass the Robonix runtime gates.
export GO2_ALLOW_MOTION=false
export GO2_OPERATOR_PRESENT=false
export GO2_SAFETY_ACK=""
export GO2_ALLOWED_MODES=""

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$UNITREE_SETUP"
set -u

command -v rviz2 >/dev/null 2>&1 || {
  echo "rviz2 is not installed; install ros-humble-rviz2 only after the separately required apt approval" >&2
  exit 3
}

echo "================================================================"
echo " GO2 NAVIGATION RVIZ — OBSERVATION / LOCALIZATION PROFILE"
echo " Fixed frame: map"
echo " Motion:      disabled by this launcher"
echo " Goal tool:   intentionally absent"
echo " Initial pose publishes only after an explicit 2D Pose Estimate click"
echo " DDS NIC:     $INTERFACE (process-local binding; no network changes)"
echo "================================================================"

exec rviz2 -d "$RVIZ_CONFIG"
