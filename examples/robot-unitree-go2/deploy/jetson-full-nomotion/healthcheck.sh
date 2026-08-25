#!/usr/bin/env bash
set -euo pipefail

readonly profile_root="/opt/robonix/profile"
readonly minimum_epoch=1704067200
readonly safe_velocity_topic="/robonix/nomotion/cmd_vel"
readonly zero_cap_hex="0000000000000000"

[[ "${ROBONIX_MOTION_ENABLED:-}" == "false" ]]
[[ "${GO2_CHASSIS_ALLOW_MOTION:-}" == "false" ]]
[[ "${ROBONIX_TIME_READY:-}" == "1" ]]
[[ "${ROBONIX_RUNTIME_COMPLETE:-}" == "0" ]]
[[ "${ROBONIX_VELOCITY_OUTPUT_TOPIC:-}" == "$safe_velocity_topic" ]]
(( $(date +%s) >= minimum_epoch ))
[[ ! -e /var/run/docker.sock && ! -e /run/docker.sock ]]

capability_value() {
  awk -v wanted="$1:" '$1 == wanted {print tolower($2)}' /proc/self/status
}
for field in CapEff CapPrm CapInh CapAmb CapBnd; do
  [[ "$(capability_value "$field")" == "$zero_cap_hex" ]]
done
"${profile_root}/validate-network.sh" >/dev/null

python3 - "${profile_root}/profile.yaml" <<'PY'
from __future__ import annotations

import sys
from pathlib import Path

import yaml

profile = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
assert profile["runtime"]["complete"] is False
assert profile["runtime"]["launch_allowed"] is False
assert profile["motion"]["enabled"] is False
PY

# An incomplete blueprint is never healthy.  A later reviewed runtime must
# replace this with
# freshness, ownership, TF, localization, Nav2-sink, speech, and UI checks.
echo "full-stack ARM64 blueprint is intentionally unhealthy" >&2
exit 78
