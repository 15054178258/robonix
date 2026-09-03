#!/usr/bin/env bash
# Bounded ROS bootstrap for the graph-only writer-identity verifier.

set -euo pipefail

readonly SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly ROS_SETUP="${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
readonly UNITREE_SETUP="${UNITREE_ROS2_SETUP:-${PROJECT_ROOT}/rbnx-build/unitree_ros2/install/setup.bash}"

cat <<'BANNER'
=============================================================================
 READ-ONLY writer identity check
 Graph discovery only: no subscription, publisher, service, action, Unitree
 client, network configuration, clock adjustment, or motion operation.
=============================================================================
BANNER

source_setup_or_exit() {
  local label="$1"
  local setup_file="$2"
  local source_status

  if [[ ! -r "${setup_file}" ]]; then
    printf 'Missing %s setup: %s; graph check did not start.\n' \
      "${label}" "${setup_file}" >&2
    exit 127
  fi
  set +u
  # shellcheck disable=SC1090
  if source "${setup_file}"; then
    source_status=0
  else
    source_status=$?
  fi
  set -u
  if (( source_status != 0 )); then
    printf 'Could not load %s setup (status %d): %s.\n' \
      "${label}" "${source_status}" "${setup_file}" >&2
    exit 127
  fi
}

source_setup_or_exit "ROS 2" "${ROS_SETUP}"
source_setup_or_exit "Unitree ROS 2 overlay" "${UNITREE_SETUP}"

# The Python verifier has its own 30-second discovery ceiling.  This outer
# bound also covers ROS bootstrap and teardown and never turns a timeout into a
# successful identity receipt.
timeout --signal=TERM --kill-after=5s --preserve-status 40s \
  python3 "${SCRIPT_DIR}/verify_workstation_writer_identity_readonly.py" "$@"
