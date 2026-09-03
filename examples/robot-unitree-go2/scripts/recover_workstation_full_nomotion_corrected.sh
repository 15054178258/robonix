#!/usr/bin/env bash
set -euo pipefail
umask 077

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ROOT="$ROOT/rbnx-build/run"
POINTER="$RUN_ROOT/workstation-nomotion-current.session"
DISCIPLINE_LOCK="$RUN_ROOT/workstation-nomotion-stamp.lock"
INDEX_LOCK="$RUN_ROOT/workstation-nomotion-session-index.lock"
uid="$(id -u)"
# shellcheck disable=SC1091
source "$ROOT/scripts/workstation_nomotion_session.sh"

[[ "$#" -eq 0 ]] || { echo "this recovery command accepts no arguments" >&2; exit 2; }
if [[ ! -e "$POINTER" && ! -L "$POINTER" ]]; then
  echo "no corrected no-motion session needs recovery"
  exit 0
fi
go2_nomotion_validate_session_files "$RUN_ROOT" "$POINTER" "$uid" || {
  echo "refusing recovery: session owner/mode/path/metadata validation failed" >&2
  exit 74
}
if go2_nomotion_process_identity_matches \
  "$GO2_SESSION_PID" "$GO2_SESSION_START_TICKS" "$uid"; then
  echo "refusing recovery: validated wrapper is still alive; use the stop script" >&2
  exit 74
else
  status=$?
fi
[[ "$status" -eq 1 ]] || {
  echo "refusing recovery: PID exists with mismatched start_ticks/owner" >&2
  exit 74
}

exec {discipline_fd}>"$DISCIPLINE_LOCK"
chmod 600 -- "$DISCIPLINE_LOCK"
flock --exclusive --nonblock "$discipline_fd" || {
  echo "refusing recovery: discipline lock is still owned" >&2
  exit 74
}
# Revalidate after taking the kernel lock; never recover a replaced pointer.
go2_nomotion_validate_session_files "$RUN_ROOT" "$POINTER" "$uid" || {
  echo "refusing recovery: session metadata changed during validation" >&2
  exit 74
}
if [[ -d "/proc/$GO2_SESSION_PID" ]]; then
  echo "refusing recovery: recorded PID appeared again" >&2
  exit 74
fi
go2_nomotion_remove_pointer_if_owned \
  "$POINTER" "$GO2_SESSION_METADATA_FILE" "$INDEX_LOCK" "$uid"
[[ ! -e "$POINTER" && ! -L "$POINTER" ]] || {
  echo "refusing recovery: current pointer was not the recorded session" >&2
  exit 74
}
echo "removed only the stale current-session pointer; RUN_DIR evidence was preserved"
