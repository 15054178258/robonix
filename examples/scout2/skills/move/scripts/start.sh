#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# move skill start (native). Runs the atlas bridge under the conda env,
# with ROS + codegen + robonix-api on PYTHONPATH.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ROS's setup.bash references unset vars; under `set -u` that is
# a FATAL exit. Relax `set -u` around the sources.
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/.cargo/env" 2>/dev/null || export PATH="$HOME/.cargo/bin:$PATH"
set -u

# Find conda
CONDA=""
for candidate in /opt/conda/bin/conda /opt/miniconda3/bin/conda /home/user/miniforge3/bin/conda; do
    [ -x "$candidate" ] && CONDA="$candidate" && break
done
if [ -z "${CONDA:-}" ]; then
    echo "[move] error: could not find conda binary."
    exit 1
fi

# Get Python from robonix-scout2 conda env
CONDA_ENV="robonix-scout2"
CONDA_PY="$("$CONDA" run -n "$CONDA_ENV" python3 -c 'import sys; print(sys.executable)' 2>/dev/null)"
if [ -z "${CONDA_PY:-}" ]; then
    echo "[move] error: failed to get Python from conda env '$CONDA_ENV'."
    exit 1
fi

# Set PYTHONPATH: codegen + robonix-api
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
if ROBONIX_PY="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_PY:$PYTHONPATH"
fi

# -u: unbuffered stdio so log lines flush immediately
exec "$CONDA_PY" -u -m move_skill.atlas_bridge
