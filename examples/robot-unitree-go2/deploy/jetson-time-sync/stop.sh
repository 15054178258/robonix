#!/usr/bin/env bash
set -euo pipefail

readonly chronyd_container="robonix-go2-time-chronyd"
readonly feeder_container="robonix-go2-time-feeder"
readonly socket_volume="robonix-go2-time-socket"

for container in "${feeder_container}" "${chronyd_container}"; do
  if docker container inspect "${container}" >/dev/null 2>&1; then
    docker stop --time 10 "${container}" >/dev/null || true
    docker rm "${container}" >/dev/null
  fi
done
if docker volume inspect "${socket_volume}" >/dev/null 2>&1; then
  docker volume rm "${socket_volume}" >/dev/null
fi
echo "Time containers and dedicated ephemeral socket volume removed."
echo "A past CLOCK_REALTIME step is not reversed; verify time before ROS/Nav2."
