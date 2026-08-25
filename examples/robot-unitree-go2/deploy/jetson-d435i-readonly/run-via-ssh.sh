#!/usr/bin/env bash
set -euo pipefail

# Workstation-side launcher. The Python source is the SSH stdin stream consumed
# by `python3 -`; it is never copied to an Orin path.

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
readonly BRIDGE_SOURCE="${SCRIPT_DIR}/d435i_ssh_stream_bridge.py"
readonly EXPECTED_ACK="I_ACKNOWLEDGE_D435I_READONLY_SENSOR_STREAMING_ONLY"
readonly SSH_TARGET="${D435I_SSH_TARGET:-unitree@192.168.123.18}"
readonly SSH_CONTROL_PATH="${D435I_SSH_CONTROL_PATH:-}"
# The D435i uses its verified 640x480 hardware profile.  The Orin downsamples
# to the dashboard's 320x240 preview before compression, so the robot Wi-Fi is
# not spent transporting discarded pixels.  Five Hz remains visibly live and
# stays above the workstation preview provider's 3 Hz sustained-rate gate.
readonly STREAM_WIDTH=320
readonly STREAM_HEIGHT=240
readonly STREAM_PUBLISH_HZ=5
readonly REMOTE_COMMAND="export PYTHONDONTWRITEBYTECODE=1 && exec python3 - --remote-producer --readonly-streaming-ack ${EXPECTED_ACK} --width ${STREAM_WIDTH} --height ${STREAM_HEIGHT} --publish-hz ${STREAM_PUBLISH_HZ}"

if [[ "${D435I_READONLY_STREAMING_ACK:-}" != "${EXPECTED_ACK}" ]]; then
  printf 'Refusing D435i access: set D435I_READONLY_STREAMING_ACK exactly to:\n  %s\n' \
    "${EXPECTED_ACK}" >&2
  exit 2
fi
if [[ ! -r "${BRIDGE_SOURCE}" ]]; then
  printf 'D435i bridge source is not readable: %s\n' "${BRIDGE_SOURCE}" >&2
  exit 2
fi
if [[ "${SSH_TARGET}" == -* || "${SSH_TARGET}" =~ [[:space:]] ]]; then
  printf 'Unsafe D435I_SSH_TARGET: %q\n' "${SSH_TARGET}" >&2
  exit 2
fi
if ! command -v ssh >/dev/null 2>&1; then
  printf 'OpenSSH client is unavailable on the workstation.\n' >&2
  exit 127
fi

ssh_options=(
  -T
  -o ClearAllForwardings=yes
  -o ServerAliveInterval=3
  -o ServerAliveCountMax=3
)
if [[ -n "${SSH_CONTROL_PATH}" ]]; then
  case "${SSH_CONTROL_PATH}" in
    "${REPOSITORY_ROOT}"/.run/*) ;;
    *)
      printf 'D435I_SSH_CONTROL_PATH must stay under %s/.run/: %q\n' \
        "${REPOSITORY_ROOT}" "${SSH_CONTROL_PATH}" >&2
      exit 2
      ;;
  esac
  if [[ "${SSH_CONTROL_PATH}" =~ [[:space:]] ]]; then
    printf 'Unsafe D435I_SSH_CONTROL_PATH: %q\n' "${SSH_CONTROL_PATH}" >&2
    exit 2
  fi
  ssh_options+=(-S "${SSH_CONTROL_PATH}")
fi

printf 'Starting ephemeral D435i read-only bridge on %s.\n' "${SSH_TARGET}"
printf 'Orin captures/compresses only; ROS 2 publishers run on this workstation.\n'
printf 'No source file will be copied to the Orin; Ctrl-C requests strict cleanup.\n'

set +u
source /opt/ros/humble/setup.bash
set -u

set +e
ssh \
  "${ssh_options[@]}" \
  "${SSH_TARGET}" \
  "${REMOTE_COMMAND}" \
  < "${BRIDGE_SOURCE}" |
  PYTHONDONTWRITEBYTECODE=1 python3 "${BRIDGE_SOURCE}" \
    --workstation-publisher \
    --readonly-streaming-ack "${EXPECTED_ACK}" \
    --width "${STREAM_WIDTH}" \
    --height "${STREAM_HEIGHT}" \
    --publish-hz "${STREAM_PUBLISH_HZ}"
pipeline_status=("${PIPESTATUS[@]}")
set -e

readonly remote_status="${pipeline_status[0]}"
readonly workstation_status="${pipeline_status[1]}"
if [[ "${remote_status}" -eq 0 && "${workstation_status}" -eq 0 ]]; then
  exit 0
fi
printf 'D435i pipeline exited: remote=%s workstation=%s\n' \
  "${remote_status}" "${workstation_status}" >&2
if [[ "${workstation_status}" -ne 0 ]]; then
  exit "${workstation_status}"
fi
exit "${remote_status}"
