#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_ROOT="${GO2_CAMERA_DAEMON_BUILD_ROOT:-${ROOT}/.build/camera-daemon}"
INSTALL_ROOT="${BUILD_ROOT}/install"
BINARY="${INSTALL_ROOT}/bin/go2_camera_daemon"
PRIVATE_LIBRARY_DIR="${INSTALL_ROOT}/lib"

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <go2-wired-interface>" >&2
  echo "This script never changes network configuration." >&2
  exit 2
fi
if [[ ! -x "${BINARY}" ]]; then
  echo "Camera daemon not built: ${BINARY}" >&2
  exit 1
fi
for library in libddsc.so libddsc.so.0 libddscxx.so libddscxx.so.0; do
  if [[ ! -e "${PRIVATE_LIBRARY_DIR}/${library}" ]]; then
    echo "Camera daemon private DDS library is missing: ${library}" >&2
    exit 1
  fi
done

echo "============================================================"
echo " READ-ONLY: camera samples only; no robot-control interfaces"
echo " Network interface: $1"
echo " Stop with Ctrl-C"
echo "============================================================"
# This assignment affects only the process replaced by exec.  It is never
# exported by start.sh and therefore cannot enter the provider or ROS nodes.
export LD_LIBRARY_PATH="${PRIVATE_LIBRARY_DIR}"
exec "${BINARY}" --interface "$1"
