#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Build phase: run rbnx codegen --mcp so:
#   * atlas_pb2 / atlas_pb2_grpc Python stubs exist.
#   * perception_mcp.py is generated from
#     capabilities/lib/perception/srv/DetectObject.srv with
#     DetectObject_Request / DetectObject_Response dataclasses.
#
# Note: we do NOT vendor openai / numpy — those come from pip install.
#
# Output layout:
#   rbnx-build/codegen/proto_gen/                atlas stubs
#   rbnx-build/codegen/robonix_mcp_types/        perception_mcp.py
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[llm_detect/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build

# Sanity: warn if openai isn't pip-installed yet.
if ! python3 -c "import openai" 2>/dev/null; then
    echo "[llm_detect/build] NOTE: openai not importable. Install with:"
    echo "                       pip install openai"
    echo "                     (deploy will fail at on_init without it)."
fi

FLAGS=(--out-dir "$PKG/rbnx-build/codegen" --mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[llm_detect/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[llm_detect/build] done."
