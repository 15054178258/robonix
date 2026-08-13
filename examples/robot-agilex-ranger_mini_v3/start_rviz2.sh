#!/usr/bin/env bash
set -euo pipefail

# Launch RViz as a Zenoh ROS 2 client for an already running Robonix deployment.
# This script deliberately does not start rbnx, a router, Nav2, or any motion.
DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG="${ROBONIX_RVIZ_CONFIG:-$DEPLOY_DIR/rviz/ranger_mapping.rviz}"

if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi

set +u
source /opt/ros/humble/setup.bash
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
if [[ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" ]]; then
  export ZENOH_CONFIG_OVERRIDE="${ZENOH_CONFIG_OVERRIDE:-connect/endpoints=[\"tcp/127.0.0.1:7447\"]}"
  python3 - <<'PY'
import socket

try:
    with socket.create_connection(("127.0.0.1", 7447), timeout=0.5):
        pass
except OSError as exc:
    raise SystemExit(
        "Zenoh router is not reachable at 127.0.0.1:7447. "
        "Start the deployment first with ./start.sh. "
        f"({exc})"
    )
PY
fi

[[ -f "$CONFIG" ]] || {
  echo "RViz config not found: $CONFIG" >&2
  exit 2
}

exec rviz2 -d "$CONFIG" "$@"
