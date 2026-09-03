#!/usr/bin/env bash
set -euo pipefail

readonly root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly duration="${1:-30}"
readonly label="${2:-sport-state}"
readonly ros_setup="${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
readonly unitree_setup="${UNITREE_ROS2_SETUP:-${root}/rbnx-build/unitree_ros2/install/setup.bash}"
readonly interface="${GO2_NETWORK_INTERFACE:-}"

[[ "$duration" =~ ^[1-9][0-9]*$ ]] && (( duration <= 120 )) || {
  echo "duration must be an integer in 1..120 seconds" >&2
  exit 2
}
[[ "$label" =~ ^[A-Za-z0-9._-]+$ ]] || {
  echo "label may contain only letters, digits, dot, underscore and dash" >&2
  exit 2
}
if [[ -n "$interface" ]]; then
  [[ "$interface" =~ ^[A-Za-z0-9_.:-]+$ ]] || {
    echo "GO2_NETWORK_INTERFACE contains unsupported characters" >&2
    exit 2
  }
  command -v ip >/dev/null 2>&1 || {
    echo "ip is required to bind the read-only ROS participant" >&2
    exit 127
  }
  ip link show dev "$interface" >/dev/null 2>&1 || {
    echo "GO2_NETWORK_INTERFACE does not exist: $interface" >&2
    exit 2
  }
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$interface\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
fi

cat <<'BANNER'
=======================================================================
 READ-ONLY SportModeState capture
 Subscriber only: no publisher, Twist, motion API or service switch.
=======================================================================
BANNER

readonly stamp="$(date -u +%Y%m%dT%H%M%SZ)"
readonly output="${root}/logs/go2-readonly/${stamp}-${label}/sport-state-summary.json"
umask 077
[[ -f "$ros_setup" && -f "$unitree_setup" ]] || {
  echo "missing ROS 2 or Unitree message overlay; run ./build.sh first" >&2
  exit 127
}
set +u
# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$unitree_setup"
set -u
timeout --signal=INT --kill-after=3s "$((duration + 5))s" \
  python3 "${root}/scripts/capture_sport_state_readonly.py" \
    --duration "$duration" \
    --output "$output"
