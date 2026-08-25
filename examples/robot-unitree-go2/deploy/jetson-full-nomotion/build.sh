#!/usr/bin/env bash
set -euo pipefail
umask 077

readonly script_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly repository_root="$(CDPATH= cd -- "${script_root}/../.." && pwd)"
readonly image="${JETSON_FULL_NOMOTION_IMAGE:-robonix-go2-jetson-full-nomotion:blueprint}"
readonly base_image="${JETSON_FULL_NOMOTION_BASE_IMAGE:-}"
readonly base_image_id="${JETSON_FULL_NOMOTION_BASE_IMAGE_ID:-}"

if [[ "$#" -ne 0 ]]; then
  echo "build.sh accepts no arguments" >&2
  exit 2
fi
if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "build natively on the reviewed JetPack 5 ARM64 host" >&2
  exit 40
fi
command -v docker >/dev/null 2>&1 || {
  echo "docker is required" >&2
  exit 41
}
command -v sha256sum >/dev/null 2>&1 || {
  echo "sha256sum is required" >&2
  exit 41
}
for reference in "$image" "$base_image"; do
  if [[ ! "$reference" =~ ^[A-Za-z0-9][A-Za-z0-9._/:@-]*$ ]]; then
    echo "invalid or absent image reference: ${reference:-<empty>}" >&2
    exit 42
  fi
done
if [[ ! "$base_image_id" =~ ^sha256:[0-9a-f]{64}$ ]]; then
  echo "JETSON_FULL_NOMOTION_BASE_IMAGE_ID must be a complete sha256 image ID" >&2
  exit 43
fi
base_image_hex="${base_image_id#sha256:}"
expected_base_reference="robonix-local/jetson-readonly:sha256-${base_image_hex}"
if [[ "$base_image" != "$expected_base_reference" ]]; then
  echo "JETSON_FULL_NOMOTION_BASE_IMAGE must be the content-named local alias:" >&2
  echo "  ${expected_base_reference}" >&2
  exit 43
fi

metadata="$(docker image inspect --format \
  '{{.Architecture}} {{.Os}} {{.Id}} {{.Config.User}} {{index .Config.Labels "io.robonix.go2.profile"}} {{index .Config.Labels "io.robonix.go2.motion"}}' \
  "$base_image")" || {
    echo "the reviewed local jetson-readonly image is absent; this build never pulls" >&2
    exit 43
  }
read -r architecture operating_system inspected_id image_user profile motion extra \
  <<< "$metadata"
if [[ "$architecture" != "arm64" \
  || "$operating_system" != "linux" \
  || "$inspected_id" != "$base_image_id" \
  || "$image_user" != "10001:10001" \
  || "$profile" != "jetson-readonly" \
  || "$motion" != "disabled" \
  || -n "${extra:-}" ]]; then
  echo "base image is not the exact reviewed ARM64 jetson-readonly image" >&2
  exit 43
fi

"${script_root}/validate-static.sh"
revision="$(git -C "$repository_root" rev-parse --verify HEAD 2>/dev/null \
  || printf unknown)"
manifest_sha256="$(sha256sum "${script_root}/robonix_manifest.yaml" | awk '{print $1}')"

echo "Building ARM64 no-motion BLUEPRINT image: ${image}"
echo "The result remains runtime-complete=false and run.sh will reject it."
DOCKER_BUILDKIT=1 docker build \
  --platform linux/arm64 \
  --pull=false \
  --build-arg "JETSON_READONLY_IMAGE=${base_image}" \
  --build-arg "VCS_REF=${revision}" \
  --build-arg "BASE_IMAGE_ID=${base_image_id}" \
  --build-arg "MANIFEST_SHA256=${manifest_sha256}" \
  --file "${script_root}/Dockerfile" \
  --tag "$image" \
  "$repository_root"

result="$(docker image inspect --format \
  '{{.Architecture}} {{.Os}} {{index .Config.Labels "io.robonix.go2.profile"}} {{index .Config.Labels "io.robonix.go2.motion"}} {{index .Config.Labels "io.robonix.go2.runtime-complete"}} {{index .Config.Labels "io.robonix.go2.manifest.sha256"}}' \
  "$image")"
if [[ "$result" != "arm64 linux jetson-full-nomotion false false ${manifest_sha256}" ]]; then
  echo "built image metadata is not the expected fail-closed blueprint" >&2
  exit 44
fi
echo "Built blueprint only; no container was started and runtime readiness is false."
