#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[scout2_chassis/build] rbnx codegen ${FLAGS[*]}"
"$PKG/../../scripts/run_python_codegen.sh" "$PKG" "${FLAGS[@]}"
echo "[scout2_chassis/build] done."
