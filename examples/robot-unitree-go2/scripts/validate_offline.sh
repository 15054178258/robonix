#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")/.." && pwd)"
echo "[validate] OFFLINE ONLY - no ROS graph, network, or motion command is used"

"$DEPLOY_DIR/scripts/verify_submodule_pins.sh"

find "$DEPLOY_DIR" -type f -name '*.sh' \
  -not -path '*/rbnx-build/*' \
  -not -path '*/third_party/*' -print0 \
  | xargs -0 -n1 bash -n

DEPLOY_DIR="$DEPLOY_DIR" python3 - <<'PY'
from pathlib import Path
import os
import xml.etree.ElementTree as ET
import yaml

root = Path(os.environ["DEPLOY_DIR"])
excluded_yaml_trees = {"rbnx-build", "third_party", "logs"}
for path in sorted(root.rglob("*.yaml")):
    if excluded_yaml_trees.isdisjoint(path.parts):
        yaml.safe_load(path.read_text(encoding="utf-8"))
for path in sorted(root.rglob("*.yml")):
    if excluded_yaml_trees.isdisjoint(path.parts):
        yaml.safe_load(path.read_text(encoding="utf-8"))
for path in (root / "config" / "navigate.xml", root / "packages" / "go2_description" / "urdf" / "go2_robonix.urdf"):
    ET.parse(path)

urdf = ET.parse(root / "packages" / "go2_description" / "urdf" / "go2_robonix.urdf").getroot()
links = {node.attrib["name"] for node in urdf.findall("link")}
required = {"base_link", "base", "imu", "radar", "utlidar_lidar", "front_camera"}
missing = required - links
if missing:
    raise SystemExit(f"URDF missing required links: {sorted(missing)}")
PY

python3 -m unittest discover \
  -s "$DEPLOY_DIR/packages/semantic_navigation/tests" -p 'test_*.py'
PYTHONPATH="$DEPLOY_DIR/packages/semantic_intent_router:$DEPLOY_DIR/packages/semantic_navigation${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover \
    -s "$DEPLOY_DIR/packages/semantic_intent_router/tests" -p 'test_*.py'
python3 -m unittest discover \
  -s "$DEPLOY_DIR/packages/go2_chassis/tests" -p 'test_*.py'
PYTHONPATH="$DEPLOY_DIR/packages/go2_description${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover \
    -s "$DEPLOY_DIR/packages/go2_description/tests" -p 'test_*.py'

command -v g++ >/dev/null 2>&1 || {
  echo "[validate] missing offline test compiler: g++" >&2
  exit 1
}
TEMP_DIR="$(mktemp -d)"
trap 'rm -rf -- "$TEMP_DIR"' EXIT
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  -I"$DEPLOY_DIR/packages/go2_chassis/include" \
  "$DEPLOY_DIR/packages/go2_chassis/tests/protocol_guard_test.cpp" \
  -o "$TEMP_DIR/protocol_guard_test"
"$TEMP_DIR/protocol_guard_test"

g++ -std=c++17 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I"$DEPLOY_DIR/packages/go2_motion_state_relay/include" \
  "$DEPLOY_DIR/packages/go2_motion_state_relay/tests/gid_guard_test.cpp" \
  -o "$TEMP_DIR/gid_guard_test"
"$TEMP_DIR/gid_guard_test"

bash "$DEPLOY_DIR/packages/go2_sensors/tests/run_offline_tests.sh"
bash "$DEPLOY_DIR/packages/go2_d435i/tests/run_offline_tests.sh"
python3 -m unittest discover \
  -s "$DEPLOY_DIR/deploy/jetson-d435i-readonly" -p 'test_*.py'
PYTHONPATH="$DEPLOY_DIR/packages/go2_dashboard${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover \
    -s "$DEPLOY_DIR/packages/go2_dashboard/tests" -p 'test_*.py'
python3 -m unittest discover -s "$DEPLOY_DIR/tests" -p 'test_*.py'

# Contract-level voice closed loop. This uses fixed in-memory ASR/Pilot/Nav
# fixtures and the real semantic/lifecycle/run-state/dashboard cores. It opens
# no ROS graph or socket and contains no robot command surface.
python3 "$DEPLOY_DIR/scripts/offline_voice_e2e.py" --compact >/dev/null

echo "[validate] PASS"
