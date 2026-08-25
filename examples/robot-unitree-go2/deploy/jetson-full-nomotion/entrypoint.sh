#!/usr/bin/env bash
set -euo pipefail

readonly profile_root="/opt/robonix/profile"
readonly minimum_epoch=1704067200
readonly safe_velocity_topic="/robonix/nomotion/cmd_vel"
readonly zero_cap_hex="0000000000000000"
readonly profile_file="${profile_root}/profile.yaml"
readonly manifest_file="/opt/robonix/deploy/robonix_manifest.yaml"

if [[ "$#" -ne 0 ]]; then
  echo "jetson-full-nomotion rejects commands and pass-through arguments" >&2
  exit 2
fi
if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "jetson-full-nomotion requires an aarch64 runtime" >&2
  exit 3
fi
if [[ ! -r /etc/os-release ]]; then
  echo "cannot validate container OS" >&2
  exit 3
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" \
  || "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "jetson-full-nomotion requires Ubuntu 22.04 and ROS Humble" >&2
  exit 3
fi
if [[ "$$" -ne 1 ]]; then
  echo "host PID namespace or a wrapper entrypoint is forbidden" >&2
  exit 4
fi
if [[ -e /var/run/docker.sock || -e /run/docker.sock ]]; then
  echo "Docker socket exposure is forbidden" >&2
  exit 4
fi

capability_value() {
  awk -v wanted="$1:" '$1 == wanted {print tolower($2)}' /proc/self/status
}
for field in CapEff CapPrm CapInh CapAmb CapBnd; do
  if [[ "$(capability_value "$field")" != "$zero_cap_hex" ]]; then
    echo "all Linux capability sets must be zero (${field})" >&2
    exit 4
  fi
done

if [[ "${ROBONIX_MOTION_ENABLED:-}" != "false" \
  || "${GO2_CHASSIS_ALLOW_MOTION:-}" != "false" ]]; then
  echo "both motion gates must be exactly false" >&2
  exit 5
fi
if [[ "${ROBONIX_VELOCITY_OUTPUT_TOPIC:-}" != "$safe_velocity_topic" ]]; then
  echo "velocity output must remain on the non-actuating sink topic" >&2
  exit 5
fi
if [[ "${ROBONIX_TIME_READY:-}" != "1" ]] || (( $(date +%s) < minimum_epoch )); then
  echo "time-ready gate is absent or host time is invalid" >&2
  exit 6
fi
if [[ "${ROBONIX_RUNTIME_COMPLETE:-}" != "0" ]]; then
  echo "this blueprint cannot be promoted through an environment variable" >&2
  exit 7
fi

"${profile_root}/validate-network.sh"

python3 - "$profile_file" "$manifest_file" <<'PY'
from __future__ import annotations

import sys
from pathlib import Path

import yaml

profile = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
manifest = yaml.safe_load(Path(sys.argv[2]).read_text(encoding="utf-8"))
assert profile["runtime"]["complete"] is False
assert profile["runtime"]["launch_allowed"] is False
assert profile["motion"]["enabled"] is False
assert profile["motion"]["navigation_velocity_output"] == "/robonix/nomotion/cmd_vel"
assert manifest["env"]["GO2_ALLOW_MOTION"] == "false"
assert manifest["env"]["ROBONIX_VELOCITY_OUTPUT_TOPIC"] == "/robonix/nomotion/cmd_vel"
PY

# The ARM64 ROS/Nav2/RTAB-Map build blueprint exists, but the pinned Robonix
# ARM64 binaries, generated contracts, speech artifacts, and supervised
# process graph do not.  The immutable image/profile marker above is false;
# never idle or claim readiness without a separately reviewed image.
echo "jetson-full-nomotion is an incomplete ARM64 blueprint; refusing startup" >&2
exit 78
