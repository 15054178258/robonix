#!/usr/bin/env bash
set -euo pipefail

readonly container="robonix-go2-jetson-full-nomotion"

if ! docker container inspect "$container" >/dev/null 2>&1; then
  echo "container is not running: ${container}"
  exit 0
fi
metadata="$(docker container inspect --format \
  '{{index .Config.Labels "io.robonix.go2.profile"}} {{index .Config.Labels "io.robonix.go2.motion"}}' \
  "$container")"
if [[ "$metadata" != "jetson-full-nomotion false" ]]; then
  echo "refusing to stop an unrecognized container with protected name: ${container}" >&2
  exit 41
fi
docker stop --time 20 "$container" >/dev/null
echo "stopped and removed ${container}; no motion command was sent"
