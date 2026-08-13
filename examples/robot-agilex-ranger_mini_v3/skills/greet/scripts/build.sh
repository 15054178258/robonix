#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# greet skill build (native). Codegen + a venv that INHERITS system
# site-packages so ROS rclpy and the host JetPack (CUDA) torch stay usable;
# we only add YOLO + MCP/HTTP libs on top — never touching the global env.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

if command -v rbnx >/dev/null 2>&1; then
    echo "[build] rbnx codegen --mcp"
    rbnx codegen -p "$PKG" --mcp
else
    echo "[build] WARNING: rbnx not in PATH — skipping codegen"
fi

if [[ ! -d .venv ]]; then
    echo "[build] creating venv (--system-site-packages: keeps rclpy + host torch)"
    python3 -m venv --system-site-packages .venv
fi
.venv/bin/pip install -q --upgrade pip
.venv/bin/pip install -q ultralytics fastmcp requests

# On Jetson, ultralytics' pip torch is CPU-only (PyPI aarch64 torch mismatches
# the CUDA driver). Replace it by symlinking the host JetPack CUDA torch family
# into the venv — the same approach speech uses. We deliberately do NOT add
# ~/.local to PYTHONPATH: it carries enum34, which shadows stdlib enum and
# breaks every import. Symlinking only the torch family is precise + clean.
if [[ -f /etc/nv_tegra_release ]]; then
    SITEPKG="$(.venv/bin/python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
    for mod in torch torchvision torchaudio torchgen functorch torio; do
        d="$(python3 -c "import ${mod} as m, os; print(os.path.dirname(m.__file__))" 2>/dev/null)" || continue
        [[ -n "$d" && -d "$d" ]] || continue
        rm -rf "${SITEPKG:?}/$mod"
        ln -sfn "$d" "$SITEPKG/$mod"
        echo "[build]   linked host $mod → venv"
    done
    .venv/bin/python -c 'import torch; print("[build] venv torch.cuda =", torch.cuda.is_available())' || true
fi

if [[ ! -f weights/yolov8n.pt ]]; then
    echo "[build] WARNING: weights/yolov8n.pt missing — the robot has no internet,"
    echo "[build]          place the YOLO weight file before start (scp it in)."
fi
echo "[build] done."
