#!/usr/bin/env bash
set -euo pipefail

readonly root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

for script in \
  build.sh \
  entrypoint.sh \
  export.sh \
  healthcheck.sh \
  run.sh \
  stop.sh \
  validate-network.sh \
  verify-runtime.sh; do
  bash -n "${root}/${script}"
done
python3 -m py_compile "${root}/camera-quality-healthcheck.py"
bash -n "${root}/../../scripts/check_runtime_ownership.sh"
bash -n "${root}/../../scripts/runtime_lease.sh"
python3 "${root}/tests/test_profile.py"
