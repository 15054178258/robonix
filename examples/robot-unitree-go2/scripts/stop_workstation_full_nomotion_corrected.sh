#!/usr/bin/env bash
set -euo pipefail
umask 077

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ROOT="$ROOT/rbnx-build/run"
POINTER="$RUN_ROOT/workstation-nomotion-current.session"
DISCIPLINE_LOCK="$RUN_ROOT/workstation-nomotion-stamp.lock"
uid="$(id -u)"
# shellcheck disable=SC1091
source "$ROOT/scripts/workstation_nomotion_session.sh"

[[ "$#" -eq 0 ]] || { echo "this stop command accepts no arguments" >&2; exit 2; }
if [[ ! -e "$POINTER" && ! -L "$POINTER" ]]; then
  echo "no corrected no-motion session is registered"
  exit 0
fi
go2_nomotion_validate_session_files "$RUN_ROOT" "$POINTER" "$uid" || {
  echo "refusing stop: session owner/mode/path/metadata validation failed" >&2
  exit 74
}
if go2_nomotion_process_identity_matches \
  "$GO2_SESSION_PID" "$GO2_SESSION_START_TICKS" "$uid"; then
  go2_nomotion_process_holds_lock "$GO2_SESSION_PID" "$DISCIPLINE_LOCK" || {
    echo "refusing stop: validated PID does not own the discipline lock" >&2
    exit 74
  }
  kill -TERM "$GO2_SESSION_PID"
  echo "TERM sent to the exactly validated corrected no-motion wrapper; no KILL was used"
  exit 0
else
  status=$?
fi
if [[ "$status" -eq 1 ]]; then
  echo "wrapper is absent; use recover_workstation_full_nomotion_corrected.sh" >&2
else
  echo "refusing stop: PID start_ticks/owner identity mismatch" >&2
fi
exit 74
