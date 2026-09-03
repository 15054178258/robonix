#!/usr/bin/env bash
set -euo pipefail

PKG="$(cd "$(dirname "$0")/.." && pwd)"
if [[ -n "${ROBONIX_SOURCE_PATH:-}" && -d "${ROBONIX_SOURCE_PATH}/pylib/robonix-api/robonix_api" ]]; then
  ROBONIX_API_ROOT="${ROBONIX_SOURCE_PATH}/pylib/robonix-api"
else
  command -v rbnx >/dev/null 2>&1 || {
    echo "rbnx is required to locate robonix-api" >&2
    exit 2
  }
  ROBONIX_API_ROOT="$(rbnx path robonix-api)"
fi
[[ -d "$ROBONIX_API_ROOT/robonix_api" ]] || {
  echo "invalid robonix-api path: $ROBONIX_API_ROOT" >&2
  exit 2
}
ROS2_IDL_SETUP="$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"
[[ -f "$ROS2_IDL_SETUP" ]] || {
  echo "missing MapLifecycle ROS 2 overlay: $ROS2_IDL_SETUP; run the package build first" >&2
  exit 2
}
set +u
# shellcheck disable=SC1090
source "$ROS2_IDL_SETUP"
set -u
export PYTHONPATH="$PKG:$ROBONIX_API_ROOT:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
exec python3 -m semantic_navigation.service
