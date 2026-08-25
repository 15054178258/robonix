#!/usr/bin/env bash
set -euo pipefail

readonly chronyd_container="robonix-go2-time-chronyd"
readonly feeder_container="robonix-go2-time-feeder"
for container in "${chronyd_container}" "${feeder_container}"; do
  if ! docker container inspect "${container}" >/dev/null 2>&1; then
    echo "required time container is absent: ${container}" >&2
    exit 1
  fi
  docker container inspect --format \
    'name={{.Name}} status={{.State.Status}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} network={{.HostConfig.NetworkMode}} caps={{json .HostConfig.CapAdd}}' \
    "${container}"
  docker logs --tail 40 "${container}"
done

echo "chronyd capability sets (Eff/Prm/Bnd SYS_TIME; no set may contain another bit):"
chronyd_host_pid="$(docker inspect --format '{{.State.Pid}}' "${chronyd_container}")"
awk '$1 ~ /^Cap(Eff|Prm|Inh|Amb|Bnd):$/ {print}' \
  "/proc/${chronyd_host_pid}/status"
echo "feeder capability sets (all must be zero):"
docker exec "${feeder_container}" \
  awk '$1 ~ /^Cap(Eff|Prm|Inh|Amb|Bnd):$/ {print}' /proc/1/status
echo "feeder health (source-vs-local-clock measurements):"
docker exec "${feeder_container}" \
  cat /dev/shm/robonix-time/feeder-health.json
echo "chronyd GO2 selection log evidence:"
if ! docker logs "${chronyd_container}" 2>&1 \
  | grep -E 'Selected source .*GO2([^[:alnum:]_]|$)'; then
  echo "GO2 has not been selected according to the bounded chronyd log" >&2
  exit 1
fi
