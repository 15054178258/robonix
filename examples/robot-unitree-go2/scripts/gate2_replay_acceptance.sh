#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

cat <<'NOTICE'
======================================================================
 GATE 2 — LOCALHOST-ONLY ROSBAG REPLAY
 No physical Go2 NIC. No Unitree SDK or chassis forwarder.
 Nav2's isolated /cmd_vel terminates at a subscription-only evidence sink.
 Fixture output is SKIP/FIXTURE_ONLY and is never an acceptance pass.
======================================================================
NOTICE

if [[ -r /opt/ros/humble/setup.bash ]]; then
  # Environment setup only; this performs no installation or network change.
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

exec python3 "$ROOT/scripts/gate2_replay.py" "$@"
