#!/usr/bin/env bash
set -euo pipefail

readonly container="robonix-go2-jetson-readonly"
if ! docker container inspect "$container" >/dev/null 2>&1; then
  echo "container is not running: ${container}"
  exit 0
fi
docker stop --time 10 "$container" >/dev/null
echo "stopped and removed ${container}"
