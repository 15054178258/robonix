#!/usr/bin/env bash

# Source-only helpers for a launcher that supervises multiple runtime children.
# The caller must provide cleanup_runtime(), which is kept deployment-specific
# so it can validate exact process identities before terminating anything.

go2_runtime_exit_on_signal() {
  local status="$1"

  # A signal can interrupt `wait -n -p` after Bash has unset its output
  # variable.  Exit from the handler instead of returning to that interrupted
  # command, and preserve the conventional signal-derived status.
  trap - EXIT INT TERM
  set +e
  cleanup_runtime
  exit "$status"
}

go2_runtime_install_cleanup_traps() {
  trap cleanup_runtime EXIT
  trap 'go2_runtime_exit_on_signal 130' INT
  trap 'go2_runtime_exit_on_signal 143' TERM
}

go2_runtime_wait_for_first_exit() {
  local restore_errexit=false

  (( "$#" >= 1 )) || return 2
  [[ "$-" == *e* ]] && restore_errexit=true

  EXITED_RUNTIME_PID=""
  RUNTIME_STATUS=0
  set +e
  wait -n -p EXITED_RUNTIME_PID "$@"
  RUNTIME_STATUS=$?
  "$restore_errexit" && set -e

  # Bash documents that -p unsets the variable when it cannot identify a
  # completed child.  Keep the caller's fail-closed check safe under `set -u`.
  EXITED_RUNTIME_PID="${EXITED_RUNTIME_PID:-}"
}
