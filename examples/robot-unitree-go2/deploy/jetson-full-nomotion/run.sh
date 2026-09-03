#!/usr/bin/env bash
set -euo pipefail
umask 077

readonly script_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly repository_root="$(CDPATH= cd -- "${script_root}/../.." && pwd)"
readonly container="robonix-go2-jetson-full-nomotion"
readonly image="${JETSON_FULL_NOMOTION_IMAGE:-}"
readonly expected_image_id="${JETSON_FULL_NOMOTION_IMAGE_ID:-}"
readonly safe_velocity_topic="/robonix/nomotion/cmd_vel"

if [[ "$#" -ne 0 ]]; then
  echo "jetson-full-nomotion accepts no arguments or pass-through command" >&2
  exit 2
fi

"${script_root}/validate-network.sh"
for command in docker date; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "missing required command: ${command}" >&2
    exit 41
  }
done

# Time readiness is a live host-side gate.  It validates both dedicated time
# containers, exact capabilities, fresh feeder evidence, and GO2 selection.
"${repository_root}/deploy/jetson-time-sync/status.sh" >/dev/null
readonly minimum_epoch=1704067200
if (( $(date +%s) < minimum_epoch )); then
  echo "host clock is not ready (earlier than 2024-01-01 UTC)" >&2
  exit 40
fi

# No default tag is supplied.  Until a reviewed ARM64 build exists, this is an
# unconditional refusal instead of an implicit pull or a misleading launch.
if [[ -z "$image" ]]; then
  echo "refusing unbuilt profile: JETSON_FULL_NOMOTION_IMAGE is unset" >&2
  exit 42
fi
if [[ ! "$image" =~ ^[A-Za-z0-9][A-Za-z0-9._/:@-]*$ ]]; then
  echo "invalid image reference" >&2
  exit 42
fi
if [[ ! "$expected_image_id" =~ ^sha256:[0-9a-f]{64}$ ]]; then
  echo "JETSON_FULL_NOMOTION_IMAGE_ID must be a full sha256 image ID" >&2
  exit 42
fi

metadata="$(docker image inspect --format \
  '{{.Id}} {{.Architecture}} {{.Os}} {{index .Config.Labels "io.robonix.go2.profile"}} {{index .Config.Labels "io.robonix.go2.motion"}} {{index .Config.Labels "io.robonix.go2.runtime-complete"}}' \
  "$image")" || {
    echo "required local image is absent; this launcher never pulls" >&2
    exit 43
  }
if [[ "$metadata" != "$expected_image_id arm64 linux jetson-full-nomotion false true" ]]; then
  echo "refusing image with mismatched ID, architecture, OS, profile, motion, or completion label" >&2
  exit 43
fi
if [[ "$(docker image inspect --format '{{if .Config.Healthcheck}}present{{else}}absent{{end}}' "$image")" != "present" ]]; then
  echo "reviewed image health check is absent" >&2
  exit 43
fi
if [[ "$(docker image inspect --format '{{json .Config.Entrypoint}}' "$image")" \
  != '["/opt/robonix/profile/entrypoint.sh"]' ]]; then
  echo "reviewed image entrypoint is absent" >&2
  exit 43
fi
if docker container inspect "$container" >/dev/null 2>&1; then
  echo "container already exists: ${container}" >&2
  exit 44
fi

echo "Starting ARM64 FULL-STACK NO-MOTION profile; physical control remains structurally disabled."
docker run \
  --detach \
  --rm \
  --name "$container" \
  --hostname "$container" \
  --runtime runc \
  --network host \
  --read-only \
  --tmpfs /tmp:rw,noexec,nosuid,nodev,size=512m,mode=1777 \
  --cap-drop ALL \
  --security-opt no-new-privileges=true \
  --pids-limit 512 \
  --memory 10g \
  --cpus 6.0 \
  --ulimit nofile=4096:4096 \
  --restart no \
  --log-driver local \
  --log-opt max-size=10m \
  --log-opt max-file=2 \
  --stop-timeout 20 \
  --user 10001:10001 \
  --env ROBONIX_MOTION_ENABLED=false \
  --env GO2_CHASSIS_ALLOW_MOTION=false \
  --env ROBONIX_TIME_READY=1 \
  --env "ROBONIX_VELOCITY_OUTPUT_TOPIC=${safe_velocity_topic}" \
  "$expected_image_id"
