#!/usr/bin/env bash
# Real-hardware receive-side UI for use while source-clock discipline is still
# blocked. This launcher intentionally starts no Robonix systems, mapping,
# localization, navigation, TF, chassis adapter, sensor relay, or voice path.

set -euo pipefail

readonly deploy_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly sensors_root="${deploy_dir}/packages/go2_sensors"
readonly dashboard_root="${deploy_dir}/packages/go2_dashboard"
readonly run_root="${deploy_dir}/rbnx-build/run"
readonly profile_run_root="${run_root}/dui"
readonly camera_socket="${profile_run_root}/cam.sock"
readonly child_lease="${profile_run_root}/children.lease"
readonly camera_daemon="${sensors_root}/.build/camera-daemon/install/bin/go2_camera_daemon"
readonly camera_private_lib="${sensors_root}/.build/camera-daemon/install/lib"
readonly sensor_overlay="${sensors_root}/.build/ros/install/setup.bash"
readonly camera_bridge="${sensors_root}/.build/ros/install/go2_sensors/lib/go2_sensors/go2_camera_bridge"
readonly sensor_config="${sensors_root}/config/go2_sensors.yaml"
readonly dashboard_python="${dashboard_root}/rbnx-build/venv/bin/python"
readonly ros_setup="${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
readonly ownership_checker="${deploy_dir}/scripts/check_readonly_ui_diagnostic_ownership.sh"
readonly interface="${GO2_NETWORK_INTERFACE:-}"
readonly dashboard_port="${GO2_DASHBOARD_PORT:-8092}"

echo "================================================================"
echo " READ-ONLY DIAGNOSTIC / NOT NAVIGATION READY"
echo " Real Go2 camera + raw lidar + raw odometry receive-side UI only."
echo " Source time is NOT trusted. Map, TF, Nav2 and browser voice are absent."
echo " No robot-control service or motion process is started."
echo "================================================================"

[[ "$interface" =~ ^[[:alnum:]_.-]+$ ]] || {
  echo "GO2_NETWORK_INTERFACE must name the approved dedicated wired interface" >&2
  exit 2
}
[[ "$dashboard_port" =~ ^[1-9][0-9]*$ ]] \
  && (( dashboard_port <= 65535 )) || {
  echo "GO2_DASHBOARD_PORT must be an integer in 1..65535" >&2
  exit 2
}
[[ -r "$ros_setup" && -r "$sensor_overlay" && -r "$sensor_config" ]] || {
  echo "ROS or go2_sensors build/config is unavailable; run the audited build first" >&2
  exit 2
}
for executable in "$camera_daemon" "$camera_bridge" "$dashboard_python"; do
  [[ -x "$executable" ]] || {
    echo "required diagnostic executable is unavailable: $executable" >&2
    exit 2
  }
done
for library in libddsc.so libddsc.so.0 libddscxx.so libddscxx.so.0; do
  [[ -e "${camera_private_lib}/${library}" ]] || {
    echo "camera private DDS library is unavailable: $library" >&2
    exit 2
  }
done
for command in flock setsid timeout; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "required diagnostic command is unavailable: $command" >&2
    exit 2
  }
done

mkdir -p -- "$profile_run_root"
chmod 0700 "$run_root" "$profile_run_root"
[[ ${#camera_socket} -lt 100 ]] || {
  echo "workspace camera socket path is too long" >&2
  exit 2
}

# shellcheck disable=SC1091
source "${deploy_dir}/scripts/runtime_lease.sh"
go2_runtime_lease_acquire "$run_root" workstation-readonly-diagnostic

export GO2_ALLOW_MOTION=false
export GO2_OPERATOR_PRESENT=false
unset GO2_SAFETY_ACK GO2_ALLOWED_MODES ROBONIX_VELOCITY_OUTPUT_TOPIC
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
readonly owned_cyclonedds_uri="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${interface}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
if [[ -n "${CYCLONEDDS_URI:-}" && "$CYCLONEDDS_URI" != "$owned_cyclonedds_uri" ]]; then
  echo "Ignoring inherited CYCLONEDDS_URI; diagnostic profile owns the Go2 NIC binding" >&2
fi
export CYCLONEDDS_URI="$owned_cyclonedds_uri"

set +u
# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$sensor_overlay"
set -u
command -v ros2 >/dev/null 2>&1 || {
  echo "ros2 is unavailable after sourcing the diagnostic environment" >&2
  exit 2
}

# This is the final operation before child creation. It requires exactly one
# vendor raw source and zero publishers on every profile-owned or nav surface.
bash "$ownership_checker" pre

readonly stamp="$(date -u +%Y%m%dT%H%M%SZ)"
readonly log_root="${deploy_dir}/logs/readonly-ui-diagnostic/${stamp}"
mkdir -p -- "$log_root"
chmod 0700 "$log_root"

declare -a child_names=()
declare -a child_pids=()
declare -a child_pgids=()
declare -a child_starts=()

process_identity() {
  local pid="$1" stat rest
  local -a fields
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 1
  IFS= read -r stat < "/proc/${pid}/stat" 2>/dev/null || return 1
  rest="${stat#*) }"
  [[ "$rest" != "$stat" ]] || return 1
  read -r -a fields <<< "$rest"
  [[ "${fields[0]:-}" != Z ]] || return 1
  [[ "${fields[2]:-}" =~ ^[1-9][0-9]*$ ]] || return 1
  [[ "${fields[19]:-}" =~ ^[0-9]+$ ]] || return 1
  printf '%s %s\n' "${fields[2]}" "${fields[19]}"
}

child_is_exact() {
  local index="$1" identity pgid start
  identity="$(process_identity "${child_pids[$index]}")" || return 1
  read -r pgid start <<< "$identity"
  [[ "$pgid" == "${child_pgids[$index]}" \
    && "$start" == "${child_starts[$index]}" ]]
}

start_child() {
  local name="$1"
  shift
  local pid candidate identity pgid start index
  setsid -- "$@" >>"${log_root}/${name}.log" 2>&1 &
  pid=$!
  identity=""
  for _ in $(seq 1 20); do
    candidate="$(process_identity "$pid" 2>/dev/null || true)"
    if [[ -n "$candidate" ]]; then
      read -r pgid start <<< "$candidate"
      if [[ "$pgid" == "$pid" ]]; then
        identity="$candidate"
        break
      fi
    fi
    sleep 0.05
  done
  [[ -n "${identity:-}" ]] || {
    kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    echo "diagnostic child did not receive an exclusive process group: $name" >&2
    return 4
  }
  read -r pgid start <<< "$identity"
  [[ "$pgid" == "$pid" ]] || {
    kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    echo "diagnostic child did not receive an exclusive process group: $name" >&2
    return 4
  }
  index="${#child_pids[@]}"
  child_names[$index]="$name"
  child_pids[$index]="$pid"
  child_pgids[$index]="$pgid"
  child_starts[$index]="$start"
}

write_child_lease() {
  local temporary index
  temporary="$(mktemp "${child_lease}.tmp.XXXXXX")"
  chmod 0600 "$temporary"
  {
    printf 'format=go2-readonly-ui-diagnostic-children-v1\n'
    printf 'token=%s\n' "$GO2_RUNTIME_LEASE_TOKEN"
    for index in "${!child_pids[@]}"; do
      printf 'child_%s_name=%s\n' "$index" "${child_names[$index]}"
      printf 'child_%s_pid=%s\n' "$index" "${child_pids[$index]}"
      printf 'child_%s_pgid=%s\n' "$index" "${child_pgids[$index]}"
      printf 'child_%s_start_ticks=%s\n' "$index" "${child_starts[$index]}"
    done
  } > "$temporary"
  mv -f -- "$temporary" "$child_lease"
}

cleanup() {
  local index deadline
  trap - EXIT INT TERM HUP
  for ((index=${#child_pids[@]} - 1; index >= 0; index--)); do
    if child_is_exact "$index"; then
      kill -TERM -- "-${child_pgids[$index]}" 2>/dev/null || true
    fi
  done
  deadline=$((SECONDS + 5))
  while (( SECONDS <= deadline )); do
    any_live=false
    for index in "${!child_pids[@]}"; do
      child_is_exact "$index" && any_live=true
    done
    [[ "$any_live" == false ]] && break
    sleep 0.1
  done
  for ((index=${#child_pids[@]} - 1; index >= 0; index--)); do
    if child_is_exact "$index"; then
      kill -KILL -- "-${child_pgids[$index]}" 2>/dev/null || true
    fi
  done
  for index in "${!child_pids[@]}"; do
    wait "${child_pids[$index]}" 2>/dev/null || true
  done
  rm -f -- "$child_lease"
  [[ ! -S "$camera_socket" ]] || rm -f -- "$camera_socket"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

start_child camera-daemon \
  env LD_LIBRARY_PATH="$camera_private_lib" \
  "$camera_daemon" --interface "$interface" --socket "$camera_socket"
start_child camera-bridge \
  "$camera_bridge" --ros-args --params-file "$sensor_config" \
  -p "camera.socket_path:=${camera_socket}"

export PYTHONPATH="${dashboard_root}${PYTHONPATH:+:${PYTHONPATH}}"
export GO2_DASHBOARD_HOST=127.0.0.1
export GO2_DASHBOARD_PORT="$dashboard_port"
export GO2_DASHBOARD_PROFILE=readonly-diagnostic
export GO2_DASHBOARD_BROWSER_VOICE_ENABLED=0
export GO2_DASHBOARD_CAMERA_TOPIC=/camera/color/image_raw
export GO2_DASHBOARD_CAMERA_STATUS_TOPIC=/go2/sensors/status
export GO2_DASHBOARD_CLOUD_TOPIC=/utlidar/cloud
export GO2_DASHBOARD_SCAN_TOPIC=/scanner/scan
export GO2_DASHBOARD_MAP_TOPIC=/map
export GO2_DASHBOARD_ODOM_TOPIC=/utlidar/robot_odom
export GO2_DASHBOARD_NAV_STATUS_TOPIC=/navigate_to_pose/_action/status
export GO2_DASHBOARD_MAP_FRAME=map
export GO2_DASHBOARD_BASE_FRAME=base_link
export GO2_DASHBOARD_PID_FILE="${profile_run_root}/dashboard.pid"
start_child dashboard \
  "$dashboard_python" -m go2_dashboard.main \
  --host 127.0.0.1 --port "$dashboard_port"

write_child_lease
bash "$ownership_checker" post

echo "================================================================"
echo " READ-ONLY DIAGNOSTIC UI: http://127.0.0.1:${dashboard_port}"
echo " NOT NAVIGATION READY. Source timestamps remain untrusted."
echo " Logs: ${log_root}"
echo " Stop with Ctrl-C; only the exact recorded PID/PGID children are signalled."
echo "================================================================"

set +e
wait -n "${child_pids[@]}"
status=$?
set -e
echo "diagnostic child exited; shutting down the exact owned process groups" >&2
exit "$status"
