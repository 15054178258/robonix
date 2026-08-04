#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scout2 deployment — boot entry point.
#
# Usage:
#   bash start.sh                  # boot with full manifest
#   bash start.sh --no-motion      # boot without motion primitives (first-time safety)
#   bash start.sh -v               # verbose boot output
#
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="$DIR/robonix_manifest.yaml"

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-motion)
      MANIFEST="$DIR/robonix_manifest.no-motion.yaml"
      shift
      ;;
    -v|--verbose)
      shift
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

echo "[scout2] booting from $MANIFEST..."
exec rbnx boot -f "$MANIFEST"
