#!/usr/bin/env bash
set -euo pipefail

readonly script_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly repository_root="$(CDPATH= cd -- "${script_root}/../.." && pwd)"
readonly image="${JETSON_TIME_SYNC_IMAGE:-robonix-go2-jetson-time-sync:local}"
readonly base_image="${JETSON_READONLY_IMAGE:-robonix-go2-jetson-readonly:local}"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "build this derivative on the JetPack 5 ARM64 host" >&2
  exit 40
fi
command -v docker >/dev/null 2>&1 || {
  echo "docker is required" >&2
  exit 41
}
for reference in "${image}" "${base_image}"; do
  if [[ ! "${reference}" =~ ^[A-Za-z0-9][A-Za-z0-9._/:@-]*$ ]]; then
    echo "invalid image reference: ${reference}" >&2
    exit 42
  fi
done

metadata="$(docker image inspect --format \
  '{{.Architecture}} {{.Os}} {{.Config.User}} {{index .Config.Labels "io.robonix.go2.profile"}} {{index .Config.Labels "io.robonix.go2.motion"}}' \
  "${base_image}")"
read -r architecture operating_system image_user base_profile motion_label extra \
  <<<"${metadata}"
if [[ "${architecture}" != "arm64" \
  || "${operating_system}" != "linux" \
  || "${image_user}" != "10001:10001" \
  || "${base_profile}" != "jetson-readonly" \
  || "${motion_label}" != "disabled" \
  || -n "${extra:-}" ]]; then
  echo "base image is not the reviewed ARM64 jetson-readonly image" >&2
  exit 43
fi

"${script_root}/validate-static.sh"
revision="$(git -C "${repository_root}" rev-parse --verify HEAD 2>/dev/null \
  || printf unknown)"

echo "This Docker build installs chrony inside the derivative image only."
echo "It does not install or start chrony on the NX host."
DOCKER_BUILDKIT=1 docker build \
  --platform linux/arm64 \
  --network host \
  --pull=false \
  --build-arg "JETSON_READONLY_IMAGE=${base_image}" \
  --build-arg "VCS_REF=${revision}" \
  --file "${script_root}/Dockerfile" \
  --tag "${image}" \
  "${repository_root}"
