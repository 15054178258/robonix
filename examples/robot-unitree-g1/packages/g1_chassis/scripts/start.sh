#!/usr/bin/env bash
# Start the G1 chassis provider.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DEPLOY_ROOT="${ROBONIX_DEPLOY_DIR:-$(cd "$PKG/../.." && pwd)}"
source "$DEPLOY_ROOT/scripts/build_robonix_ros2_overlay.sh"

echo "[g1/start] launching g1_chassis provider"

# shellcheck disable=SC1091
source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"
source "$PKG/rbnx-build/ros/install/setup.bash"

exec python3 -m g1_chassis.main
