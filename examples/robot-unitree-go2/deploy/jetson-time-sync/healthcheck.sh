#!/usr/bin/env bash
set -euo pipefail

readonly role="${GO2_TIME_ROLE:-probe}"
readonly runtime_root="/dev/shm/robonix-time"
readonly socket_root="/run/robonix-go2-time"
readonly refclock_socket="${socket_root}/refclock/go2.sock"
readonly zero_cap_hex="0000000000000000"

capability_value() {
  awk -v wanted="$1:" '$1 == wanted {print tolower($2)}' /proc/1/status
}

require_zero_caps() {
  local field
  for field in CapEff CapPrm CapInh CapAmb CapBnd; do
    [[ "$(capability_value "${field}")" == "${zero_cap_hex}" ]]
  done
}

case "${role}" in
  probe|graph-preflight)
    require_zero_caps
    [[ -d "${runtime_root}" ]]
    ;;
  feeder)
    require_zero_caps
    [[ -S "${refclock_socket}" && -r "${runtime_root}/feeder-health.json" ]]
    [[ "$(stat -c '%u:%g' "${refclock_socket}")" == "10002:10001" ]]
    socket_mode="$(stat -c '%a' "${refclock_socket}")"
    (( (8#${socket_mode} & 2) == 0 ))
    python3 - "${runtime_root}/feeder-health.json" <<'PY'
import json, sys, time
with open(sys.argv[1], encoding="utf-8") as stream:
    value = json.load(stream)
if value.get("unsafe_latched") is not False:
    raise SystemExit(1)
if value.get("feed_count", 0) <= 0:
    raise SystemExit(1)
if set(value.get("required_topics", [])) - set(value.get("authorized_topics", [])):
    raise SystemExit(1)
updated = int(value.get("updated_monotonic_ns", 0))
if updated <= 0 or time.monotonic_ns() - updated > 5_000_000_000:
    raise SystemExit(1)
last_sample = int(value.get("last_valid_sample_monotonic_ns", 0))
if last_sample <= 0 or time.monotonic_ns() - last_sample > 2_000_000_000:
    raise SystemExit(1)
PY
    ;;
  *) exit 1 ;;
esac
