#!/usr/bin/env bash
set -euo pipefail

readonly runtime_root="/tmp/robonix"
readonly pid_root="${runtime_root}/pids"

process_start_ticks() {
  local pid="$1" stat rest
  local -a fields
  IFS= read -r stat < "/proc/${pid}/stat" || return 1
  rest="${stat#*) }"
  [[ "$rest" != "$stat" ]] || return 1
  read -r -a fields <<< "$rest"
  [[ "${fields[19]:-}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${fields[19]}"
}

/opt/robonix/profile/validate-network.sh >/dev/null

[[ -r "${runtime_root}/runtime-profile" ]] || exit 19
IFS= read -r runtime_profile < "${runtime_root}/runtime-profile"
case "$runtime_profile" in
  full)
    required=(robot-state-publisher chassis-passive sensor-relay)
    ;;
  sensors-only)
    required=(sensor-relay)
    ;;
  *)
    exit 19
    ;;
esac
if [[ -f "${runtime_root}/camera-enabled" ]]; then
  required+=(camera-daemon camera-bridge)
fi

for name in "${required[@]}"; do
  pid_file="${pid_root}/${name}.pid"
  [[ -r "$pid_file" ]] || exit 20
  pid="$(sed -n 's/^pid=//p' "$pid_file")"
  start_ticks="$(sed -n 's/^start_ticks=//p' "$pid_file")"
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] || exit 21
  [[ "$start_ticks" =~ ^[0-9]+$ ]] || exit 21
  [[ "$(process_start_ticks "$pid" 2>/dev/null)" == "$start_ticks" ]] || exit 22
done

grep -Eq '^[[:space:]]*allow_motion:[[:space:]]*false[[:space:]]*$' \
  /opt/robonix/profile/config/chassis-passive.yaml

if [[ -f "${runtime_root}/camera-enabled" ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=0
  export ROS_LOCALHOST_ONLY=0
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  timeout --signal=TERM --kill-after=1s 6s \
    /opt/robonix/profile/camera-quality-healthcheck.py
fi
