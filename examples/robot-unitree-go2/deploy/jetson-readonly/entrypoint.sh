#!/usr/bin/env bash
set -euo pipefail

readonly profile_root="/opt/robonix/profile"
readonly runtime_root="/tmp/robonix"
readonly pid_root="${runtime_root}/pids"
readonly ros_log_root="${runtime_root}/ros-log"
readonly camera_socket="${runtime_root}/camera.sock"
readonly urdf="/opt/robonix/overlay/share/go2_description/urdf/go2_robonix.urdf"
readonly rsp_parameters="${runtime_root}/robot-description.yaml"

if [[ "$#" -ne 0 ]]; then
  echo "jetson-readonly does not accept commands or pass-through arguments" >&2
  exit 2
fi
if [[ "${GO2_ENABLE_CAMERA:-0}" != "0" && "${GO2_ENABLE_CAMERA:-0}" != "1" ]]; then
  echo "GO2_ENABLE_CAMERA must be exactly 0 or 1" >&2
  exit 2
fi
if [[ "${GO2_NX_RUNTIME_PROFILE:-full}" != "full" \
  && "${GO2_NX_RUNTIME_PROFILE:-full}" != "sensors-only" ]]; then
  echo "GO2_NX_RUNTIME_PROFILE must be exactly full or sensors-only" >&2
  exit 2
fi
readonly runtime_profile="${GO2_NX_RUNTIME_PROFILE:-full}"
if [[ ! -r /etc/os-release ]]; then
  echo "cannot validate container OS" >&2
  exit 3
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "jetson-readonly requires Ubuntu 22.04 userland" >&2
  exit 3
fi

"${profile_root}/validate-network.sh"

umask 077
mkdir -p "$pid_root" "$ros_log_root" "${runtime_root}/camera-ipc" /tmp/home
chmod 0700 "$runtime_root" "$pid_root" "$ros_log_root" \
  "${runtime_root}/camera-ipc" /tmp/home
export ROS_LOG_DIR="$ros_log_root"
export XDG_RUNTIME_DIR="${runtime_root}/xdg"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 0700 "$XDG_RUNTIME_DIR"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /opt/robonix/overlay/setup.bash
set -u

# shellcheck disable=SC1091
source "${profile_root}/runtime-lease.sh"
go2_runtime_lease_acquire "$runtime_root" "nx-${runtime_profile}"

printf '%s\n' "$runtime_profile" > "${runtime_root}/runtime-profile"

if [[ "$runtime_profile" == full ]]; then
  if [[ ! -r "$urdf" ]]; then
    echo "pinned Go2 URDF is missing" >&2
    exit 4
  fi
  {
    printf '/**:\n  ros__parameters:\n    robot_description: |\n'
    sed 's/^/      /' "$urdf"
  } > "$rsp_parameters"
fi

declare -a child_pids=()

start_child() {
  local name="$1"
  shift
  echo "[jetson-readonly] starting ${name}"
  "$@" &
  local pid=$!
  child_pids+=("$pid")
  local start_ticks temporary
  start_ticks="$(go2_runtime_process_start_ticks "$pid")"
  temporary="$(mktemp "${pid_root}/${name}.pid.tmp.XXXXXX")"
  {
    printf 'pid=%s\n' "$pid"
    printf 'start_ticks=%s\n' "$start_ticks"
  } > "$temporary"
  chmod 0600 "$temporary"
  mv -f -- "$temporary" "${pid_root}/${name}.pid"
}

terminate_children() {
  trap - EXIT INT TERM
  local pid
  for pid in "${child_pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  for pid in "${child_pids[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap terminate_children EXIT INT TERM

"${profile_root}/check-runtime-ownership.sh" "nx-${runtime_profile}" pre

if [[ "$runtime_profile" == full ]]; then
  start_child robot-state-publisher \
    /opt/ros/humble/lib/robot_state_publisher/robot_state_publisher \
    --ros-args --params-file "$rsp_parameters"
  start_child chassis-passive \
    /opt/robonix/overlay/lib/go2_chassis_adapter/go2_chassis_adapter_node \
    --ros-args --params-file "${profile_root}/config/chassis-passive.yaml"
else
  echo "[jetson-readonly] sensors-only: odom and tf_static publishers are absent"
fi
start_child sensor-relay \
  /opt/robonix/overlay/lib/go2_sensors/go2_sensor_relay \
  --ros-args --params-file "${profile_root}/config/sensors-readonly.yaml"

if [[ "$GO2_ENABLE_CAMERA" == "1" ]]; then
  printf 'enabled\n' > "${runtime_root}/camera-enabled"
  start_child camera-daemon \
    env LD_LIBRARY_PATH=/opt/robonix/camera/lib \
    /opt/robonix/camera/bin/go2_camera_daemon \
    --interface eth0 \
    --socket "$camera_socket" \
    --domain-id 0 \
    --fps 10 \
    --api-timeout-ms 1000 \
    --socket-timeout-ms 1500 \
    --max-jpeg-bytes 4194304
  start_child camera-bridge \
    /opt/robonix/overlay/lib/go2_sensors/go2_camera_bridge \
    --ros-args --params-file "${profile_root}/config/sensors-readonly.yaml"
else
  echo "[jetson-readonly] camera reader packaged but disabled"
fi

if ! "${profile_root}/check-runtime-ownership.sh" "nx-${runtime_profile}" post; then
  echo "[jetson-readonly] post-start publisher ownership failed" >&2
  exit 74
fi

echo "[jetson-readonly] READ-ONLY ${runtime_profile} runtime active; motion is structurally unavailable"
set +e
wait -n "${child_pids[@]}"
child_status=$?
set -e
echo "[jetson-readonly] a required child exited with status ${child_status}" >&2
exit "$child_status"
