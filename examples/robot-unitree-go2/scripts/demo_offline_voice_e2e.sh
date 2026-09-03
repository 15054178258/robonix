#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "[offline-voice-e2e] OFFLINE / NO ROS / NO NETWORK / NO MOTION"
echo "[offline-voice-e2e] running isolation and contract tests"
env \
  -u ROS_DOMAIN_ID \
  -u CYCLONEDDS_URI \
  -u RMW_IMPLEMENTATION \
  -u HTTP_PROXY \
  -u HTTPS_PROXY \
  -u ALL_PROXY \
  PYTHONNOUSERSITE=1 \
  python3 -m unittest discover \
    -s "$DEPLOY_DIR/tests" \
    -p 'test_offline_voice_e2e.py'

echo "[offline-voice-e2e] running deterministic fixture"
env \
  -u ROS_DOMAIN_ID \
  -u CYCLONEDDS_URI \
  -u RMW_IMPLEMENTATION \
  -u HTTP_PROXY \
  -u HTTPS_PROXY \
  -u ALL_PROXY \
  PYTHONNOUSERSITE=1 \
  python3 "$DEPLOY_DIR/scripts/offline_voice_e2e.py"
