#!/usr/bin/env bash
set -euo pipefail

readonly deploy_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly dashboard_root="${deploy_dir}/packages/go2_dashboard"
readonly python="${dashboard_root}/rbnx-build/venv/bin/python"
readonly run_root="${deploy_dir}/rbnx-build/run"
readonly lock_file="${run_root}/ui-client-dashboard.lock"
readonly lease_file="${run_root}/ui-client-dashboard.lease"
readonly child_pid_file="${run_root}/ui-client-dashboard.child.pid"

# shellcheck disable=SC1091
source "${deploy_dir}/scripts/runtime_lease.sh"

if [[ "${GO2_RUNTIME_PLACEMENT:-}" != "workstation-ui-nx-full" ]]; then
  echo "UI/client-only launcher requires GO2_RUNTIME_PLACEMENT=workstation-ui-nx-full" >&2
  exit 2
fi
if [[ ! -x "$python" ]]; then
  echo "dashboard build is unavailable; run ./build.sh before UI/client-only start" >&2
  exit 2
fi

mkdir -p "$run_root"
chmod 0700 "$run_root"
# Direct invocation receives the same placement exclusion as ./start.sh.  When
# start.sh execs this launcher, its already-held descriptor is inherited.
if [[ ! "${GO2_RUNTIME_LEASE_FD:-}" =~ ^[0-9]+$ \
  || ! -e "/proc/$$/fd/${GO2_RUNTIME_LEASE_FD}" ]]; then
  go2_runtime_lease_acquire "$run_root" workstation-ui-nx-full
fi

exec {UI_LEASE_FD}>"$lock_file"
if ! flock --exclusive --nonblock "$UI_LEASE_FD"; then
  echo "UI/client-only dashboard already has an active atomic lease" >&2
  exit 3
fi
GO2_UI_LEASE_TOKEN="$(go2_runtime_new_token)"
export GO2_UI_LEASE_TOKEN

export PYTHONPATH="${dashboard_root}${PYTHONPATH:+:${PYTHONPATH}}"
export GO2_DASHBOARD_HOST=127.0.0.1
export GO2_DASHBOARD_PORT="${GO2_DASHBOARD_PORT:-8092}"
export GO2_DASHBOARD_CAMERA_TOPIC=/camera/color/image_raw
export GO2_DASHBOARD_CAMERA_STATUS_TOPIC=/go2/sensors/status
export GO2_DASHBOARD_CLOUD_TOPIC=/scanner/cloud
export GO2_DASHBOARD_SCAN_TOPIC=/scanner/scan
export GO2_DASHBOARD_MAP_TOPIC=/map
export GO2_DASHBOARD_ODOM_TOPIC=/odom
export GO2_DASHBOARD_NAV_STATUS_TOPIC=/navigate_to_pose/_action/status
export GO2_DASHBOARD_MAP_FRAME=map
export GO2_DASHBOARD_BASE_FRAME=base_link
export GO2_DASHBOARD_BROWSER_VOICE_ENABLED=0
export GO2_DASHBOARD_PID_FILE="$child_pid_file"

echo "================================================================"
echo " READ-ONLY GO2 UI/CLIENT-ONLY PROFILE"
echo " NX owns camera, lidar, odom and TF. This process only subscribes."
echo " Browser voice, Robonix providers, Mapping, Nav2 and motion are absent."
echo " UI: http://127.0.0.1:${GO2_DASHBOARD_PORT}"
echo "================================================================"

"$python" -m go2_dashboard.main \
  --host 127.0.0.1 \
  --port "$GO2_DASHBOARD_PORT" &
UI_CHILD_PID=$!

write_ui_lease() {
  local owner_start child_start temporary
  owner_start="$(go2_runtime_process_start_ticks "$$")"
  child_start="$(go2_runtime_process_start_ticks "$UI_CHILD_PID")"
  temporary="$(mktemp "${lease_file}.tmp.XXXXXX")"
  chmod 0600 "$temporary"
  {
    printf 'format=go2-ui-lease-v1\n'
    printf 'token=%s\n' "$GO2_UI_LEASE_TOKEN"
    printf 'owner_pid=%s\n' "$$"
    printf 'owner_start_ticks=%s\n' "$owner_start"
    printf 'child_pid=%s\n' "$UI_CHILD_PID"
    printf 'child_start_ticks=%s\n' "$child_start"
  } > "$temporary"
  mv -f -- "$temporary" "$lease_file"
}

remove_own_lease() {
  local recorded_token=""
  if [[ -r "$lease_file" ]]; then
    recorded_token="$(sed -n 's/^token=//p' "$lease_file" 2>/dev/null || true)"
  fi
  if [[ "$recorded_token" == "$GO2_UI_LEASE_TOKEN" ]]; then
    rm -f -- "$lease_file" "$child_pid_file"
  fi
}

terminate_ui() {
  trap - EXIT INT TERM
  if [[ -n "${UI_CHILD_PID:-}" ]] \
    && kill -0 "$UI_CHILD_PID" 2>/dev/null; then
    kill -TERM "$UI_CHILD_PID" 2>/dev/null || true
  fi
  wait "${UI_CHILD_PID:-0}" 2>/dev/null || true
  remove_own_lease
}
trap terminate_ui EXIT INT TERM

write_ui_lease
if ! kill -0 "$UI_CHILD_PID" 2>/dev/null; then
  echo "UI/client-only dashboard exited during startup" >&2
  exit 4
fi
bash "$deploy_dir/scripts/check_runtime_ownership.sh" workstation-ui-nx-full post

set +e
wait "$UI_CHILD_PID"
UI_STATUS=$?
set -e
remove_own_lease
trap - EXIT INT TERM
exit "$UI_STATUS"
