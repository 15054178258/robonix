#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
IDL_SETUP="$ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash"
DESCRIPTION_SETUP="$ROOT/rbnx-build/description/install/setup.bash"

for setup in "$ROS_SETUP" "$IDL_SETUP" "$DESCRIPTION_SETUP"; do
  [[ -r "$setup" ]] || { echo "missing setup: $setup; run the package build first" >&2; exit 1; }
done

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$IDL_SETUP"
# shellcheck disable=SC1090
source "$DESCRIPTION_SETUP"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROBONIX_API="${ROBONIX_SOURCE_PATH:-}/pylib/robonix-api"
if [[ ! -d "$ROBONIX_API" ]]; then
  ROBONIX_API="$(rbnx path robonix-api)"
fi
export PYTHONPATH="$ROOT:$ROBONIX_API:$ROOT/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"

echo "[go2_description] READ-ONLY TF provider; no Unitree SDK or control API"
exec python3 -m go2_description_provider.main
