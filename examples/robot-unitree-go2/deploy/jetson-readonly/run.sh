#!/usr/bin/env bash
set -euo pipefail

readonly script_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly image="${JETSON_READONLY_IMAGE:-robonix-go2-jetson-readonly:local}"
readonly container="robonix-go2-jetson-readonly"
camera=0
runtime_profile=full

if [[ "$#" -gt 2 ]]; then
  echo "usage: $0 [--camera] [--sensors-only]" >&2
  exit 2
fi
for option in "$@"; do
  case "$option" in
    --camera)
      [[ "$camera" == 0 ]] || { echo "duplicate option: $option" >&2; exit 2; }
      camera=1
      ;;
    --sensors-only)
      [[ "$runtime_profile" == full ]] \
        || { echo "duplicate option: $option" >&2; exit 2; }
      runtime_profile=sensors-only
      ;;
    *)
      echo "unknown option: $option" >&2
      exit 2
      ;;
  esac
done
if [[ ! "$image" =~ ^[a-zA-Z0-9][a-zA-Z0-9._/:@-]*$ ]]; then
  echo "invalid image reference" >&2
  exit 42
fi

"${script_root}/validate-network.sh"
if [[ "$(docker image inspect --format '{{.Architecture}}' "$image")" != "arm64" ]]; then
  echo "refusing to run a non-ARM64 image" >&2
  exit 43
fi
if docker container inspect "$container" >/dev/null 2>&1; then
  echo "container already exists: ${container}" >&2
  exit 45
fi

echo "Starting READ-ONLY ${runtime_profile} profile; motion, control daemons and persistence are absent."
docker run \
  --detach \
  --rm \
  --name "$container" \
  --hostname "$container" \
  --runtime runc \
  --network host \
  --read-only \
  --tmpfs /tmp:rw,noexec,nosuid,nodev,size=128m,mode=1777 \
  --cap-drop ALL \
  --security-opt no-new-privileges=true \
  --pids-limit 192 \
  --memory 1536m \
  --cpus 2.0 \
  --ulimit nofile=1024:1024 \
  --restart no \
  --log-driver none \
  --stop-timeout 10 \
  --user 10001:10001 \
  --env "GO2_ENABLE_CAMERA=${camera}" \
  --env "GO2_NX_RUNTIME_PROFILE=${runtime_profile}" \
  "$image"
