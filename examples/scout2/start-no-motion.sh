#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scout2 deployment — no-motion boot entry point.
#
# Usage:
#   bash start-no-motion.sh          # boot without motion primitives (first-time safety)
#
# Removes chassis primitive from the manifest so the robot cannot move on first
# bring-up. Sensors and services still start for verification.
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
FULL_MANIFEST="$DIR/robonix_manifest.yaml"
NO_MOTION_MANIFEST="$DIR/robonix_manifest.no-motion.yaml"

if [ ! -f "$NO_MOTION_MANIFEST" ]; then
  echo "[scout2] generating no-motion manifest..."
  # Remove the chassis primitive entry from the manifest
  # Keep: atlas, scene, executor, pilot, liaison
  # Keep: camera, lidar primitives
  # Skip: scout2_chassis primitive
  # Keep: all services, skills
  python3 - "$FULL_MANIFEST" "$NO_MOTION_MANIFEST" <<'PYEOF'
import yaml, sys

with open(sys.argv[1]) as f:
    manifest = yaml.safe_load(f)

# Remove chassis primitive
manifest["primitive"] = [
    p for p in manifest.get("primitive", [])
    if p["name"] != "scout2_chassis"
]

with open(sys.argv[2], "w") as f:
    yaml.dump(manifest, f, default_flow_style=False, sort_keys=False)
PYEOF
  echo "[scout2] no-motion manifest created at $NO_MOTION_MANIFEST"
fi

exec rbnx boot -f "$NO_MOTION_MANIFEST" -v
