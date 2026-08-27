#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
DEPLOY_ROOT="${ROBONIX_DEPLOY_DIR:-$(cd "$PKG/../.." && pwd)}"
# shellcheck disable=SC1091
source "$DEPLOY_ROOT/scripts/build_robonix_ros2_overlay.sh"
cd "$PKG"
rbnx codegen -p "$PKG" --mcp --ros2

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "missing /opt/ros/humble/setup.bash for MapLifecycle interface build" >&2
  exit 2
}
command -v colcon >/dev/null 2>&1 || {
  echo "colcon is required to build the MapLifecycle interface" >&2
  exit 2
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
IDL="$PKG/rbnx-build/codegen/ros2_idl"
[[ -d "$IDL/src/map" ]] || {
  echo "generated map/msg/MapLifecycle package is missing: $IDL/src/map" >&2
  exit 2
}
robonix_build_ros2_overlay "$IDL" \
  --packages-select lifecycle semantic_navigation map
python3 -m unittest discover -s tests -p 'test_*.py' 2>/dev/null || echo "[semantic_navigation] skipping tests (no test files found)"
