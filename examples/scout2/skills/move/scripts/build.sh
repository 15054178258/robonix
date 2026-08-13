#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# move skill build (native). Codegen + venv that inherits system
# site-packages so ROS rclpy stays usable.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

if command -v rbnx >/dev/null 2>&1; then
    echo "[build] rbnx codegen --mcp"
    rbnx codegen -p "$PKG" --mcp
else
    echo "[build] WARNING: rbnx not in PATH — skipping codegen"
fi

echo "[build] done."
