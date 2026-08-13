#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# greet skill start (native). Runs the atlas bridge under the skill venv,
# with ROS + codegen + robonix-api on PYTHONPATH.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ROS's setup.bash (and cargo env) reference unset vars; under `set -u` that is
# a FATAL exit, not a catchable non-zero — `|| true` can't save it, the shell
# dies before exec python. Relax `set -u` around the sources (same as nav2's
# start_native). rbnx must be on PATH so `rbnx path robonix-api` resolves.
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/.cargo/env" 2>/dev/null || export PATH="$HOME/.cargo/bin:$PATH"
set -u

# torch/torchvision are symlinked into the venv at build time (host JetPack
# CUDA torch), so no ~/.local on PYTHONPATH (it carries enum34 which shadows
# stdlib enum). rclpy comes from the sourced ROS env; ultralytics/fastmcp/
# requests from the venv itself.
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
if ROBONIX_PY="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_PY:$PYTHONPATH"
fi
export GREET_YOLO_WEIGHTS="${GREET_YOLO_WEIGHTS:-$PKG/weights/yolov8n.pt}"

# -u: unbuffered stdio so log lines flush immediately and scribe (boot's
# stdout collector) captures them — otherwise greet.log stays empty.
exec "$PKG/.venv/bin/python" -u -m greet_skill.atlas_bridge
