#!/usr/bin/env bash
set -uo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
WORKSPACE_ROOT="$(cd "$DEPLOY_DIR/../.." 2>/dev/null && pwd)"
WORKSPACE_TOOLS_BIN="$WORKSPACE_ROOT/.tools/rbnx/bin"
export ROBONIX_HOME="$WORKSPACE_ROOT/.tools/robonix-home"
if ! command -v rbnx >/dev/null 2>&1 && [[ -x "$WORKSPACE_TOOLS_BIN/rbnx" ]]; then
  export PATH="$WORKSPACE_TOOLS_BIN:$PATH"
fi

ROBONIX_CONFIG_STATUS=0
if ! command -v python3 >/dev/null 2>&1; then
  echo "warning: python3 is unavailable; workspace-local Robonix config was not validated" >&2
  ROBONIX_CONFIG_STATUS=78
elif ! python3 "$DEPLOY_DIR/scripts/validate_robonix_home.py" \
  "$ROBONIX_HOME/config.yaml" \
  "$WORKSPACE_ROOT/upstream/robonix-go2-build"; then
  echo "warning: refusing to invoke rbnx with an invalid workspace-local config" >&2
  ROBONIX_CONFIG_STATUS=78
fi

readonly RUN_ROOT="$DEPLOY_DIR/rbnx-build/run"
readonly UI_LOCK_FILE="$RUN_ROOT/ui-client-dashboard.lock"
readonly UI_LEASE_FILE="$RUN_ROOT/ui-client-dashboard.lease"
readonly UI_CHILD_PID_FILE="$RUN_ROOT/ui-client-dashboard.child.pid"
readonly SEMANTIC_LOCK_FILE="$RUN_ROOT/semantic-intent-router.lock"
readonly SEMANTIC_LEASE_FILE="$RUN_ROOT/semantic-intent-router.lease"
RUNTIME_STOP_WARNING=0

# shellcheck disable=SC1091
source "$DEPLOY_DIR/scripts/runtime_lease.sh"

process_identity_matches() {
  local pid="$1"
  local expected_start="$2"
  local stat rest
  local -a fields
  IFS= read -r stat < "/proc/${pid}/stat" 2>/dev/null || return 1
  rest="${stat#*) }"
  [[ "$rest" != "$stat" ]] || return 1
  read -r -a fields <<< "$rest"
  [[ "${fields[0]:-}" != Z && "${fields[19]:-}" == "$expected_start" ]]
}

process_holds_lock() {
  local pid="$1"
  local lock_file="$2"
  local expected_identity candidate
  expected_identity="$(stat -Lc '%d:%i' "$lock_file" 2>/dev/null)" || return 1
  for candidate in "/proc/${pid}/fd/"*; do
    [[ -e "$candidate" ]] || continue
    if [[ "$(stat -Lc '%d:%i' "$candidate" 2>/dev/null || true)" == "$expected_identity" ]]; then
      return 0
    fi
  done
  return 1
}

read_ui_lease() {
  local line key value
  unset UI_FORMAT UI_TOKEN UI_OWNER_PID UI_OWNER_START UI_CHILD_PID UI_CHILD_START
  declare -A seen=()
  while IFS= read -r line || [[ -n "$line" ]]; do
    [[ "$line" == *=* ]] || return 1
    key="${line%%=*}"
    value="${line#*=}"
    [[ -z "${seen[$key]:-}" ]] || return 1
    seen[$key]=1
    case "$key" in
      format) UI_FORMAT="$value" ;;
      token) UI_TOKEN="$value" ;;
      owner_pid) UI_OWNER_PID="$value" ;;
      owner_start_ticks) UI_OWNER_START="$value" ;;
      child_pid) UI_CHILD_PID="$value" ;;
      child_start_ticks) UI_CHILD_START="$value" ;;
      *) return 1 ;;
    esac
  done < "$UI_LEASE_FILE"
  [[ "${UI_FORMAT:-}" == go2-ui-lease-v1 ]]
  [[ "${UI_TOKEN:-}" =~ ^[0-9a-f-]{36}$ ]]
  [[ "${UI_OWNER_PID:-}" =~ ^[1-9][0-9]*$ ]]
  [[ "${UI_OWNER_START:-}" =~ ^[0-9]+$ ]]
  [[ "${UI_CHILD_PID:-}" =~ ^[1-9][0-9]*$ ]]
  [[ "${UI_CHILD_START:-}" =~ ^[0-9]+$ ]]
}

stop_ui_client_only() {
  [[ -e "$UI_LOCK_FILE" || -e "$UI_LEASE_FILE" ]] || return 0
  mkdir -p "$RUN_ROOT"
  exec {probe_fd}>"$UI_LOCK_FILE"
  if flock --exclusive --nonblock "$probe_fd"; then
    # No process owns the kernel lease. Any metadata/PID file is stale and may
    # be safely recovered without signalling a numeric PID.
    rm -f -- "$UI_LEASE_FILE" "$UI_CHILD_PID_FILE"
    flock --unlock "$probe_fd"
    exec {probe_fd}>&-
    echo "removed stale UI/client-only lease metadata"
    return 0
  fi
  exec {probe_fd}>&-

  if [[ ! -r "$UI_LEASE_FILE" ]] || ! read_ui_lease; then
    echo "warning: active UI lease has missing/malformed metadata; no process was signalled" >&2
    RUNTIME_STOP_WARNING=1
    return 0
  fi
  if ! process_identity_matches "$UI_OWNER_PID" "$UI_OWNER_START" \
    || ! process_holds_lock "$UI_OWNER_PID" "$UI_LOCK_FILE"; then
    echo "warning: UI lease owner identity did not validate; no process was signalled" >&2
    RUNTIME_STOP_WARNING=1
    return 0
  fi

  kill -TERM "$UI_OWNER_PID" 2>/dev/null || true
  for _ in $(seq 1 50); do
    process_identity_matches "$UI_OWNER_PID" "$UI_OWNER_START" || break
    sleep 0.2
  done
  if process_identity_matches "$UI_OWNER_PID" "$UI_OWNER_START"; then
    echo "warning: validated UI launcher did not stop within 10 seconds" >&2
    RUNTIME_STOP_WARNING=1
  else
    echo "UI/client-only dashboard stopped"
  fi
}

read_semantic_lease() {
  local line key value
  unset SEM_FORMAT SEM_TOKEN SEM_OWNER_PID SEM_OWNER_START SEM_CHILD_PID SEM_CHILD_START
  declare -A seen=()
  while IFS= read -r line || [[ -n "$line" ]]; do
    [[ "$line" == *=* ]] || return 1
    key="${line%%=*}"
    value="${line#*=}"
    [[ -z "${seen[$key]:-}" ]] || return 1
    seen[$key]=1
    case "$key" in
      format) SEM_FORMAT="$value" ;;
      token) SEM_TOKEN="$value" ;;
      owner_pid) SEM_OWNER_PID="$value" ;;
      owner_start_ticks) SEM_OWNER_START="$value" ;;
      child_pid) SEM_CHILD_PID="$value" ;;
      child_start_ticks) SEM_CHILD_START="$value" ;;
      *) return 1 ;;
    esac
  done < "$SEMANTIC_LEASE_FILE"
  [[ "${SEM_FORMAT:-}" == go2-semantic-router-lease-v1 ]]
  [[ "${SEM_TOKEN:-}" =~ ^[0-9a-f-]{36}$ ]]
  [[ "${SEM_OWNER_PID:-}" =~ ^[1-9][0-9]*$ ]]
  [[ "${SEM_OWNER_START:-}" =~ ^[0-9]+$ ]]
  [[ "${SEM_CHILD_PID:-}" =~ ^[1-9][0-9]*$ ]]
  [[ "${SEM_CHILD_START:-}" =~ ^[0-9]+$ ]]
}

stop_semantic_router() {
  [[ -e "$SEMANTIC_LOCK_FILE" || -e "$SEMANTIC_LEASE_FILE" ]] || return 0
  mkdir -p "$RUN_ROOT"
  exec {probe_fd}>"$SEMANTIC_LOCK_FILE"
  if flock --exclusive --nonblock "$probe_fd"; then
    rm -f -- "$SEMANTIC_LEASE_FILE"
    flock --unlock "$probe_fd"
    exec {probe_fd}>&-
    echo "removed stale semantic router lease metadata"
    return 0
  fi
  exec {probe_fd}>&-

  if [[ ! -r "$SEMANTIC_LEASE_FILE" ]] || ! read_semantic_lease; then
    echo "warning: active semantic router lease has malformed metadata; no process was signalled" >&2
    RUNTIME_STOP_WARNING=1
    return 0
  fi
  if ! process_identity_matches "$SEM_OWNER_PID" "$SEM_OWNER_START" \
    || ! process_holds_lock "$SEM_OWNER_PID" "$SEMANTIC_LOCK_FILE" \
    || ! process_identity_matches "$SEM_CHILD_PID" "$SEM_CHILD_START" \
    || ! process_holds_lock "$SEM_CHILD_PID" "$SEMANTIC_LOCK_FILE"; then
    echo "warning: semantic router owner/child identity did not validate; no process was signalled" >&2
    RUNTIME_STOP_WARNING=1
    return 0
  fi
  kill -TERM "$SEM_CHILD_PID" 2>/dev/null || true
  for _ in $(seq 1 50); do
    process_identity_matches "$SEM_CHILD_PID" "$SEM_CHILD_START" || break
    sleep 0.2
  done
  if process_identity_matches "$SEM_CHILD_PID" "$SEM_CHILD_START"; then
    echo "warning: validated semantic router did not stop within 10 seconds" >&2
    RUNTIME_STOP_WARNING=1
  else
    echo "semantic intent router stopped"
  fi
}

# A stale or malformed optional UI lease must never prevent the independent
# Robonix shutdown path from running.
stop_ui_client_only

RBNX_STATUS=0
if (( ROBONIX_CONFIG_STATUS == 0 )) \
  && command -v rbnx >/dev/null 2>&1 \
  && [[ -f "$DEPLOY_DIR/rbnx-boot/state.json" ]]; then
  rbnx shutdown -f "$MANIFEST" "$@" || RBNX_STATUS=$?
elif (( ROBONIX_CONFIG_STATUS != 0 )); then
  echo "Robonix shutdown skipped because its workspace-local config is invalid" >&2
else
  echo "no Robonix boot state; nothing to stop"
fi

stop_semantic_router

if (( RBNX_STATUS != 0 )); then
  exit "$RBNX_STATUS"
fi
if (( ROBONIX_CONFIG_STATUS != 0 )); then
  exit "$ROBONIX_CONFIG_STATUS"
fi
exit "$RUNTIME_STOP_WARNING"
