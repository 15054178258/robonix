#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[go2_d435i/test] OFFLINE ONLY - no ROS graph, USB, network, or robot access"
PYTHONPATH="${ROOT}${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m unittest discover -s "${ROOT}/tests" -p 'test_*.py' -v
