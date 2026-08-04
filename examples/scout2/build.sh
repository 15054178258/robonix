#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scout2 deployment — build entry point.
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

echo "[scout2] building deployment from $MANIFEST..."
rbnx build -f "$MANIFEST" "${FLAGS[@]}"
echo "[scout2] build complete."
