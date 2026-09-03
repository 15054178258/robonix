#!/usr/bin/env bash
set -euo pipefail

ROOT="${RBNX_PACKAGE_ROOT:-$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)}"

if ! command -v rbnx >/dev/null 2>&1; then
  echo "rbnx is not available." >&2
  exit 1
fi

echo "[go2_sensors/build] generating Robonix gRPC contract bindings"
rbnx codegen -p "${ROOT}" --ros2
bash "${ROOT}/scripts/build_ros.sh"
bash "${ROOT}/scripts/build_camera_daemon.sh"
echo "[go2_sensors/build] done"
