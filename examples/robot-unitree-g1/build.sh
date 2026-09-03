#!/usr/bin/env bash
# Unitree G1 deployment — build entry point.
#
# Usage:
#   bash build.sh                  # build all primitives and services
#   bash build.sh --clean          # clean previous build artifacts first
#
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="$DIR/robonix_manifest.yaml"
FLAGS=()
if [[ "${1:-}" == "--clean" ]]; then
  FLAGS+=(--clean)
fi

echo "[g1] building deployment from $MANIFEST..."
rbnx build -f "$MANIFEST" "${FLAGS[@]}"
echo "[g1] build complete."
