#!/usr/bin/env bash
set -euo pipefail

readonly profile_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly repository_root="$(CDPATH= cd -- "${profile_root}/../.." && pwd)"

for script in \
  build.sh \
  entrypoint.sh \
  healthcheck.sh \
  run.sh \
  stop.sh \
  validate-network.sh \
  verify-image-rootfs.sh; do
  bash -n "${profile_root}/${script}"
done

for required in \
  Dockerfile \
  Dockerfile.dockerignore \
  profile.yaml \
  robonix_manifest.yaml; do
  [[ -s "${profile_root}/${required}" ]] || {
    echo "missing no-motion build input: ${required}" >&2
    exit 1
  }
done

python3 "${repository_root}/tests/test_jetson_full_nomotion_profile.py"
echo "jetson-full-nomotion ARM64 blueprint static validation passed"
