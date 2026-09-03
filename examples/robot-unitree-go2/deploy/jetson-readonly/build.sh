#!/usr/bin/env bash
set -euo pipefail

readonly script_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly repository_root="$(CDPATH= cd -- "${script_root}/../.." && pwd)"
readonly image="${JETSON_READONLY_IMAGE:-robonix-go2-jetson-readonly:local}"
readonly base_image="${JETSON_READONLY_ROS_IMAGE:-}"
readonly base_image_id="${JETSON_READONLY_ROS_IMAGE_ID:-}"
readonly upstream_digest="${JETSON_READONLY_ROS_UPSTREAM_DIGEST:-}"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "build on the JetPack 5 ARM64 host; cross-building is intentionally unsupported" >&2
  exit 40
fi
if ! command -v docker >/dev/null 2>&1; then
  echo "docker is required" >&2
  exit 41
fi
if [[ ! "$image" =~ ^[a-zA-Z0-9][a-zA-Z0-9._/:@-]*$ ]]; then
  echo "invalid image reference" >&2
  exit 42
fi
if [[ ! "$base_image_id" =~ ^sha256:[0-9a-f]{64}$ ]]; then
  echo "JETSON_READONLY_ROS_IMAGE_ID must be a complete sha256 image ID" >&2
  exit 43
fi
base_image_hex="${base_image_id#sha256:}"
expected_local_reference="robonix-local/ros-humble-ros-base-jammy:sha256-${base_image_hex}"
if [[ "$base_image" != "$expected_local_reference" ]]; then
  echo "JETSON_READONLY_ROS_IMAGE must be the content-named local alias:" >&2
  echo "  ${expected_local_reference}" >&2
  exit 43
fi
if [[ ! "$upstream_digest" =~ ^sha256:[0-9a-f]{64}$ ]]; then
  echo "JETSON_READONLY_ROS_UPSTREAM_DIGEST must record the reviewed official digest" >&2
  exit 43
fi
if ! base_metadata="$(
  docker image inspect --format '{{.Architecture}} {{.Os}} {{.Id}}' "$base_image"
)"; then
  echo "the content-named base image is not loaded locally" >&2
  exit 43
fi
read -r inspected_arch inspected_os inspected_id extra_metadata <<< "$base_metadata"
if [[ "$inspected_arch" != "arm64" || "$inspected_os" != "linux" \
  || "$inspected_id" != "$base_image_id" || -n "${extra_metadata:-}" ]]; then
  echo "loaded base image metadata does not match linux/arm64 ${base_image_id}" >&2
  exit 43
fi

revision="unknown"
if command -v git >/dev/null 2>&1; then
  revision="$(git -C "$repository_root" rev-parse --verify HEAD 2>/dev/null || printf unknown)"
fi

echo "Building CPU-only ARM64 image: ${image}"
build_arguments=(
  --build-arg "ROS_IMAGE=${base_image}"
  --build-arg "VCS_REF=${revision}"
  --build-arg "BASE_IMAGE_ID=${base_image_id}"
  --build-arg "BASE_IMAGE_UPSTREAM_DIGEST=${upstream_digest}"
)
# Docker treats these six names as predefined proxy args and excludes their
# values from image history/cache metadata.  Do not declare them in Dockerfile.
for proxy_name in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  proxy_value="${!proxy_name:-}"
  if [[ -n "$proxy_value" ]]; then
    if [[ "$proxy_name" == "NO_PROXY" || "$proxy_name" == "no_proxy" ]]; then
      if [[ ! "$proxy_value" =~ ^[a-zA-Z0-9._,:*/-]+$ ]]; then
        echo "${proxy_name} contains unsupported characters" >&2
        exit 44
      fi
    elif [[ "$proxy_value" =~ ^https?://(127\.0\.0\.1|localhost):([0-9]{1,5})/?$ ]]; then
      proxy_port="${BASH_REMATCH[2]}"
      if (( proxy_port < 1 || proxy_port > 65535 )); then
        echo "${proxy_name} has an invalid local proxy port" >&2
        exit 44
      fi
    else
      echo "${proxy_name} must be an unauthenticated loopback HTTP(S) proxy URL" >&2
      exit 44
    fi
    build_arguments+=(--build-arg "${proxy_name}=${proxy_value}")
  fi
done
DOCKER_BUILDKIT=1 docker build \
  --platform linux/arm64 \
  --network host \
  --pull=false \
  "${build_arguments[@]}" \
  --file "${script_root}/Dockerfile" \
  --tag "$image" \
  "$repository_root"
