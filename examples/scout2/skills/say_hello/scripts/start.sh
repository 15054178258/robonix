#!/usr/bin/env bash
# say_hello skill start (native). Runs under the conda env,
# with codegen + robonix-api on PYTHONPATH.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# Find conda
CONDA=""
for candidate in /opt/conda/bin/conda /opt/miniconda3/bin/conda /home/user/miniforge3/bin/conda; do
    [ -x "$candidate" ] && CONDA="$candidate" && break
done
if [ -z "${CONDA:-}" ]; then
    echo "[say_hello] error: could not find conda binary."
    exit 1
fi

# Get Python from robonix-scout2 conda env
CONDA_ENV="robonix-scout2"
CONDA_PY="$("$CONDA" run -n "$CONDA_ENV" python3 -c 'import sys; print(sys.executable)' 2>/dev/null)"
if [ -z "${CONDA_PY:-}" ]; then
    echo "[say_hello] error: failed to get Python from conda env '$CONDA_ENV'."
    exit 1
fi

# Set PYTHONPATH: codegen + robonix-api
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
if ROBONIX_PY="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_PY:$PYTHONPATH"
fi

# -u: unbuffered stdio so log lines flush immediately
exec "$CONDA_PY" -u -m say_hello.main
