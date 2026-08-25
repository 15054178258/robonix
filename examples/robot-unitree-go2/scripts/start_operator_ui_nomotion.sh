#!/usr/bin/env bash
set -euo pipefail
umask 077

readonly ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly WORKSPACE_ROOT="$(CDPATH= cd -- "$ROOT/../.." && pwd)"
readonly PYTHON="$WORKSPACE_ROOT/.tools/rbnx-python/bin/python3"
readonly RBNX="$WORKSPACE_ROOT/.tools/rbnx/bin/rbnx"
readonly READINESS="$ROOT/scripts/operator_ui_nomotion_readiness.py"
readonly STACK_LAUNCHER="$ROOT/scripts/start_workstation_full_nomotion_corrected.sh"
readonly CLIENT_LAUNCHER="$ROOT/scripts/start_robonix_client_local.sh"
readonly START_TIMEOUT_SECONDS="${GO2_OPERATOR_UI_START_TIMEOUT_SECONDS:-180}"

if [[ "$#" -ne 0 ]]; then
  echo "this launcher accepts no arguments" >&2
  exit 2
fi
[[ "$START_TIMEOUT_SECONDS" =~ ^[0-9]+$ ]] \
  && (( START_TIMEOUT_SECONDS >= 30 && START_TIMEOUT_SECONDS <= 600 )) || {
    echo "GO2_OPERATOR_UI_START_TIMEOUT_SECONDS must be an integer in 30..600" >&2
    exit 2
  }
for path in "$READINESS" "$STACK_LAUNCHER" "$CLIENT_LAUNCHER"; do
  [[ -f "$path" && ! -L "$path" ]] || {
    echo "required launcher component must be a regular non-symlink file: $path" >&2
    exit 2
  }
done
[[ -x "$PYTHON" && -x "$RBNX" ]] || {
  echo "workspace-local Python or rbnx is missing; run the documented build first" >&2
  exit 2
}

# This supervisor can only make the existing corrected no-motion launcher more
# explicit. It cannot turn that launcher's evidence, network, timestamp, DDS,
# ownership, or canonical odometry gates into optional checks.
export GO2_ALLOW_MOTION=false
export GO2_OPERATOR_PRESENT=false
export GO2_SAFETY_ACK=""
export GO2_ALLOWED_MODES=""
export ROBONIX_CLIENT_ENABLE_AUDIO=auto
export ROBONIX_CLIENT_ROBOT_HOST=127.0.0.1
export ROBONIX_CLIENT_ATLAS_PORT=50051
export ROBONIX_CLIENT_PORT=7860
export GO2_DASHBOARD_PORT=8092

STACK_PID=""
STACK_START_TICKS=""
CLIENT_PID=""
CLIENT_START_TICKS=""

process_identity() {
  local pid="$1" line rest
  local -a fields
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/stat" ]] || return 1
  IFS= read -r line < "/proc/$pid/stat" || return 1
  rest="${line##*) }"
  read -r -a fields <<< "$rest"
  [[ "${#fields[@]}" -gt 19 ]] || return 1
  printf '%s %s\n' "${fields[0]}" "${fields[19]}"
}

owned_child_alive() {
  local pid="$1" expected_ticks="$2" identity state ticks
  identity="$(process_identity "$pid")" || return 1
  read -r state ticks <<< "$identity"
  [[ "$state" != Z && "$ticks" == "$expected_ticks" ]]
}

terminate_owned_child() {
  local pid="$1" ticks="$2" label="$3"
  [[ -n "$pid" ]] || return 0
  if owned_child_alive "$pid" "$ticks"; then
    echo "[operator-ui] stopping owned $label child pid=$pid" >&2
    kill -TERM "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup_owned_children() {
  # Stop the client first so it cannot keep a stale Atlas session while the
  # corrected no-motion wrapper performs its own exact-child cleanup.
  terminate_owned_child "$CLIENT_PID" "$CLIENT_START_TICKS" "Client"
  terminate_owned_child "$STACK_PID" "$STACK_START_TICKS" "no-motion stack"
}

on_exit() {
  local status=$?
  trap - EXIT HUP INT TERM
  cleanup_owned_children
  exit "$status"
}

on_signal() {
  local status="$1"
  trap - EXIT HUP INT TERM
  cleanup_owned_children
  exit "$status"
}

trap on_exit EXIT
trap 'on_signal 129' HUP
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

wait_for_phase() {
  local phase="$1" pid="$2" ticks="$3" label="$4"
  local deadline=$((SECONDS + START_TIMEOUT_SECONDS)) child_status
  while (( SECONDS <= deadline )); do
    if ! owned_child_alive "$pid" "$ticks"; then
      child_status=0
      wait "$pid" || child_status=$?
      echo "$label exited before readiness (status=$child_status)" >&2
      if (( child_status == 0 )); then
        return 75
      fi
      return "$child_status"
    fi
    if "$PYTHON" "$READINESS" \
      --phase "$phase" --rbnx "$RBNX" --timeout 0.5 >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.25
  done
  echo "$label readiness deadline expired after ${START_TIMEOUT_SECONDS}s" >&2
  "$PYTHON" "$READINESS" \
    --phase "$phase" --rbnx "$RBNX" --timeout 1 --verbose || true
  return 75
}

echo "================================================================"
echo " ROBONIX GO2 OPERATOR UI — CORRECTED NO-MOTION SUPERVISOR"
echo " Motion:   DISABLED (no command publisher is added by this launcher)"
echo " Audio:    auto (text UI remains available without sounddevice)"
echo "================================================================"

# Refuse an unrelated Client listener before spending time qualifying the
# hardware-facing no-motion stack. No listener is killed or replaced.
"$PYTHON" "$READINESS" --assert-port-free 7860 --timeout 0.2

bash "$STACK_LAUNCHER" &
STACK_PID=$!
stack_identity="$(process_identity "$STACK_PID")" || {
  echo "could not capture no-motion stack child identity" >&2
  exit 75
}
read -r _ STACK_START_TICKS <<< "$stack_identity"
wait_for_phase stack "$STACK_PID" "$STACK_START_TICKS" "no-motion stack"
echo "[operator-ui] Atlas, Scene, Mapping and Dashboard are ready"

# Repeat the free-port check to close the long qualification-window race.
"$PYTHON" "$READINESS" --assert-port-free 7860 --timeout 0.2
bash "$CLIENT_LAUNCHER" &
CLIENT_PID=$!
client_identity="$(process_identity "$CLIENT_PID")" || {
  echo "could not capture Client child identity" >&2
  exit 75
}
read -r _ CLIENT_START_TICKS <<< "$client_identity"
wait_for_phase full "$CLIENT_PID" "$CLIENT_START_TICKS" "official Client"

echo "================================================================"
echo " ROBONIX GO2 OPERATOR UI READY — MOTION DISABLED"
echo " Client:    http://127.0.0.1:7860/"
echo " Scene:     http://127.0.0.1:50107/user"
echo " Mapping:   http://127.0.0.1:8091/"
echo " Dashboard: http://127.0.0.1:8092/"
echo " Atlas:     127.0.0.1:50051 (gRPC; no browser page)"
echo " Stop:      Ctrl-C (only children owned by this supervisor are stopped)"
echo "================================================================"

WAIT_PIDS=("$STACK_PID" "$CLIENT_PID")
while ((${#WAIT_PIDS[@]} > 0)); do
  set +e
  EXITED_PID=""
  wait -n -p EXITED_PID "${WAIT_PIDS[@]}"
  status=$?
  set -e
  if [[ "$EXITED_PID" == "$STACK_PID" ]]; then
    echo "no-motion stack exited; closing the owned Client" >&2
    exit "$status"
  fi
  if [[ "$EXITED_PID" == "$CLIENT_PID" ]]; then
    echo "WARNING: official Client exited; Mapping, Scene and the no-motion stack remain running" >&2
    CLIENT_PID=""
  else
    echo "managed-child wait ended without an exact child identity" >&2
    exit 75
  fi
  remaining_pids=()
  for managed_pid in "${WAIT_PIDS[@]}"; do
    [[ "$managed_pid" == "$EXITED_PID" ]] || remaining_pids+=("$managed_pid")
  done
  WAIT_PIDS=("${remaining_pids[@]}")
done
exit 75
