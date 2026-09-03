#!/usr/bin/env bash
set -euo pipefail
umask 077

readonly script_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly repository_root="$(CDPATH= cd -- "${script_root}/../.." && pwd)"
readonly image="${JETSON_TIME_SYNC_IMAGE:-robonix-go2-jetson-time-sync:local}"
readonly probe_container="robonix-go2-time-probe"
readonly preflight_container="robonix-go2-time-preflight"
readonly chronyd_container="robonix-go2-time-chronyd"
readonly feeder_container="robonix-go2-time-feeder"
readonly socket_volume="robonix-go2-time-socket"
readonly exact_gate="I_APPROVE_NX_HOST_CLOCK_DISCIPLINE_V1"
mode="probe"
bundle_path=""

case "$#" in
  0) ;;
  1)
    case "$1" in
      --probe) mode="probe" ;;
      *) echo "usage: $0 [--probe | --bootstrap-step EVIDENCE_BUNDLE | --steady-slew EVIDENCE_BUNDLE]" >&2; exit 2 ;;
    esac
    ;;
  2)
    case "$1" in
      --bootstrap-step) mode="bootstrap-step" ;;
      --steady-slew) mode="steady-slew" ;;
      *) echo "usage: $0 [--probe | --bootstrap-step EVIDENCE_BUNDLE | --steady-slew EVIDENCE_BUNDLE]" >&2; exit 2 ;;
    esac
    bundle_path="$2"
    ;;
  *)
    echo "usage: $0 [--probe | --bootstrap-step EVIDENCE_BUNDLE | --steady-slew EVIDENCE_BUNDLE]" >&2
    exit 2
    ;;
esac

"${script_root}/validate-network.sh"
for command in docker realpath sha256sum base64 python3 awk; do
  command -v "${command}" >/dev/null 2>&1 || {
    echo "missing command: ${command}" >&2
    exit 41
  }
done
if [[ ! "${image}" =~ ^[A-Za-z0-9][A-Za-z0-9._/:@-]*$ ]]; then
  echo "invalid image reference" >&2
  exit 42
fi
if [[ "$(docker image inspect --format '{{.Architecture}} {{.Os}} {{index .Config.Labels "io.robonix.go2.profile"}} {{index .Config.Labels "io.robonix.go2.time-topology"}}' "${image}")" \
  != "arm64 linux jetson-time-sync dual-container" ]]; then
  echo "refusing an unreviewed/non-ARM64/non-separated time image" >&2
  exit 43
fi
for name in "${probe_container}" "${preflight_container}" "${chronyd_container}" "${feeder_container}" ; do
  if docker container inspect "${name}" >/dev/null 2>&1; then
    echo "time component already exists: ${name}" >&2
    exit 44
  fi
done
if docker volume inspect "${socket_volume}" >/dev/null 2>&1; then
  echo "stale dedicated socket volume exists; inspect/remove it before launch" >&2
  exit 44
fi

readonly_base=(
  --detach
  --runtime runc
  --read-only
  --cap-drop ALL
  --security-opt no-new-privileges=true
  --pids-limit 96
  --memory 512m
  --cpus 1.0
  --ulimit nofile=512:512
  --restart no
  --log-driver local
  --log-opt max-size=10m
  --log-opt max-file=2
  --stop-timeout 10
)

if [[ "${mode}" == "probe" ]]; then
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  evidence_dir="${repository_root}/logs/go2-readonly/${stamp}-nx-time-sidecar"
  [[ ! -e "${evidence_dir}" ]] || {
    echo "refusing to overwrite evidence: ${evidence_dir}" >&2
    exit 45
  }
  mkdir -m 0700 -p "${evidence_dir}"
  echo "Starting bounded no-adjust profile: host network, all capability sets zero."
  docker run "${readonly_base[@]}" \
    --name "${probe_container}" \
    --hostname "${probe_container}" \
    --network host \
    --user 10001:10001 \
    --env GO2_TIME_ROLE=probe \
    --env GO2_TIME_PROFILE=probe \
    --env "GO2_TIME_PROBE_DURATION_SECONDS=${GO2_TIME_PROBE_DURATION_SECONDS:-60}" \
    --env "GO2_TIME_PROBE_MAX_SAMPLES=${GO2_TIME_PROBE_MAX_SAMPLES:-50000}" \
    "${image}" >/dev/null
  set +e
  docker wait "${probe_container}" >/dev/null
  probe_status="$(docker inspect --format '{{.State.ExitCode}}' "${probe_container}")"
  set -e
  if ! docker cp "${probe_container}:/dev/shm/robonix-time/evidence/." "${evidence_dir}/"; then
    docker logs "${probe_container}" >"${evidence_dir}/container.log" 2>&1 || true
    docker rm -f "${probe_container}" >/dev/null 2>&1 || true
    echo "probe evidence copy failed: ${evidence_dir}" >&2
    exit 1
  fi
  docker logs "${probe_container}" >"${evidence_dir}/container.log" 2>&1 || true
  docker rm "${probe_container}" >/dev/null
  echo "READ-ONLY evidence copied without a host bind: ${evidence_dir}"
  exit "${probe_status}"
fi

if [[ "${GO2_TIME_ALLOW_SYS_TIME:-}" != "${exact_gate}" ]]; then
  echo "Refusing SYS_TIME: exact one-shot approval token is absent." >&2
  exit 50
fi
bundle_path="$(realpath -e -- "${bundle_path}")"
case "${bundle_path}" in
  "${repository_root}/logs/"*) ;;
  *) echo "evidence bundle must be under repository logs/" >&2; exit 50 ;;
esac
[[ -d "${bundle_path}" ]] || {
  echo "evidence bundle must be a directory" >&2
  exit 50
}
readonly post_gate_audit="${bundle_path}/post-bootstrap-quality.json"
readonly post_gate_samples="${bundle_path}/post-bootstrap-quality-samples.jsonl"
if [[ -e "${post_gate_audit}" || -e "${post_gate_samples}" ]]; then
  echo "post-bootstrap audit output already exists; refusing overwrite" >&2
  exit 50
fi

# This is intentionally performed at formal launch time, not only when the
# approval was prepared.  Raw PCAP, topic-info and correlation files are all
# reread, rehashed and independently correlated.
verified_approval="$(python3 "${script_root}/validate_approval.py" \
  --bundle "${bundle_path}" --emit-canonical)" || exit 50
approval_sha256="$(printf '%s' "${verified_approval}" | sha256sum | awk '{print $1}')"
approval_b64="$(printf '%s' "${verified_approval}" | base64 -w 0)"

competing_clock_processes() {
  local excluded_pid="${1:-0}" comm_file pid comm
  local -a found=()
  for comm_file in /proc/[0-9]*/comm; do
    [[ -r "${comm_file}" ]] || continue
    pid="${comm_file#/proc/}"
    pid="${pid%/comm}"
    [[ "${pid}" == "${excluded_pid}" ]] && continue
    comm="$(<"${comm_file}")"
    case "${comm}" in
      chronyd|ntpd|ntp|openntpd|systemd-timesyn|ptp4l|phc2sys|timemaster)
        found+=("${comm}:${pid}")
        ;;
    esac
  done
  if (( ${#found[@]} > 0 )); then
    printf 'competing clock process: %s\n' "${found[*]}" >&2
    return 1
  fi
}

competing_clock_services() {
  command -v systemctl >/dev/null 2>&1 || return 0
  local service
  local -a found=()
  for service in \
    systemd-timesyncd.service chrony.service chronyd.service ntp.service \
    ntpd.service ptp4l.service phc2sys.service timemaster.service; do
    if systemctl is-active --quiet "${service}" 2>/dev/null; then
      found+=("${service}")
    fi
  done
  if (( ${#found[@]} > 0 )); then
    printf 'competing clock service: %s\n' "${found[*]}" >&2
    return 1
  fi
}

competing_sys_time_containers() {
  local excluded_name="${1:-}" name caps
  local -a found=()
  while IFS= read -r name; do
    [[ -n "${name}" && "${name}" != "${excluded_name}" ]] || continue
    caps="$(docker inspect --format '{{json .HostConfig.CapAdd}}' "${name}")"
    if [[ "${caps}" == *SYS_TIME* ]]; then
      found+=("${name}")
    fi
  done < <(docker ps --format '{{.Names}}')
  if (( ${#found[@]} > 0 )); then
    printf 'competing SYS_TIME container: %s\n' "${found[*]}" >&2
    return 1
  fi
}

if ! competing_clock_services \
  || ! competing_clock_processes \
  || ! competing_sys_time_containers; then
  echo "Stopping/disabling a competing servo is a separate approved action." >&2
  exit 51
fi

if [[ "${mode}" == "bootstrap-step" ]]; then
  mapfile -t running_containers < <(docker ps --format '{{.Names}}')
  if (( ${#running_containers[@]} > 0 )); then
    echo "FIRST STEP REFUSED: stop every other container first." >&2
    printf '  running: %s\n' "${running_containers[@]}" >&2
    exit 52
  fi
  if [[ "${GO2_TIME_ALL_ROS_CONTAINERS_STOPPED:-}" != "YES" ]]; then
    echo "FIRST STEP REFUSED: GO2_TIME_ALL_ROS_CONTAINERS_STOPPED=YES is absent" >&2
    exit 52
  fi
fi

# A no-capability, no-adjust container now compares the live graph writer GID
# against the GID whose source IP was just recomputed from the raw PCAP.
echo "Running zero-capability live graph/source-evidence preflight."
set +e
docker run --rm \
  --name "${preflight_container}" \
  --hostname "${preflight_container}" \
  --runtime runc \
  --network host \
  --read-only \
  --cap-drop ALL \
  --security-opt no-new-privileges=true \
  --pids-limit 64 \
  --memory 384m \
  --cpus 1.0 \
  --user 10001:10001 \
  --env GO2_TIME_ROLE=graph-preflight \
  --env "GO2_TIME_PROFILE=${mode}" \
  --env "GO2_TIME_FORMAL_GATE=${exact_gate}" \
  --env "GO2_TIME_APPROVAL_SHA256=${approval_sha256}" \
  --env "GO2_TIME_APPROVAL_B64=${approval_b64}" \
  "${image}"
preflight_status=$?
set -e
if (( preflight_status != 0 )); then
  echo "formal launch refused: live writer no longer matches evidence" >&2
  exit 53
fi

docker volume create --label io.robonix.go2.time-socket=true \
  "${socket_volume}" >/dev/null
cleanup_failed_start() {
  docker rm -f "${feeder_container}" "${chronyd_container}" >/dev/null 2>&1 || true
  docker volume rm "${socket_volume}" >/dev/null 2>&1 || true
}
trap cleanup_failed_start ERR INT TERM

echo "Starting networkless chronyd with the sole CAP_SYS_TIME grant."
if [[ "${mode}" == "bootstrap-step" ]]; then
  chrony_config="/opt/robonix/time-sync/config/chrony-bootstrap.conf"
else
  chrony_config="/opt/robonix/time-sync/config/chrony-steady.conf"
fi
docker run "${readonly_base[@]}" \
  --name "${chronyd_container}" \
  --hostname "${chronyd_container}" \
  --network none \
  --user 10002:10002 \
  --cap-add SYS_TIME \
  --no-healthcheck \
  --mount "type=volume,source=${socket_volume},target=/run/robonix-go2-time" \
  --entrypoint /usr/sbin/chronyd \
  "${image}" -U -F 2 -n -d -f "${chrony_config}" >/dev/null

# Do not use docker exec here: any exec in this container would itself inherit
# SYS_TIME.  Read PID 1's status from the host and require the exact sets.
chronyd_host_pid="$(docker inspect --format '{{.State.Pid}}' "${chronyd_container}")"
chronyd_caps_exact() {
  local status_file="/proc/${chronyd_host_pid}/status" field value expected
  [[ -r "${status_file}" ]] || return 1
  for field in CapEff CapPrm CapInh CapAmb CapBnd; do
    value="$(awk -v wanted="${field}:" '$1 == wanted {print tolower($2)}' "${status_file}")"
    case "${field}" in
      CapEff|CapPrm|CapBnd) expected="0000000002000000" ;;
      CapInh|CapAmb)
        # A non-root OCI launch can need inheritable/ambient SYS_TIME to
        # preserve that single capability across execve.  It may also clear
        # them after exec.  Both states are safe; every other bit is refused.
        [[ "${value}" == "0000000000000000" \
          || "${value}" == "0000000002000000" ]] || return 1
        continue
        ;;
    esac
    [[ "${value}" == "${expected}" ]] || return 1
  done
}
chronyd_single_process() {
  local line
  local -a chronyd_process_lines=()
  mapfile -t chronyd_process_lines < <(
    docker top "${chronyd_container}" -eo pid=,comm= 2>/dev/null \
      | awk 'NF {print $1 " " $2}'
  )
  [[ "${#chronyd_process_lines[@]}" -eq 1 ]] || return 1
  line="${chronyd_process_lines[0]}"
  [[ "${line}" == "${chronyd_host_pid} chronyd" ]]
}
chronyd_selected_go2() {
  docker logs "${chronyd_container}" 2>&1 \
    | grep -Eq 'Selected source .*GO2([^[:alnum:]_]|$)'
}
caps_stable=0
for _ in $(seq 1 50); do
  if chronyd_caps_exact && chronyd_single_process; then
    caps_stable=$((caps_stable + 1))
    (( caps_stable >= 10 )) && break
  else
    caps_stable=0
  fi
  sleep 0.1
done
if (( caps_stable < 10 )); then
  echo "chronyd runtime capability sets are not exact/stable" >&2
  exit 55
fi

echo "Starting host-network DDS feeder with all five capability sets zero."
docker run "${readonly_base[@]}" \
  --name "${feeder_container}" \
  --hostname "${feeder_container}" \
  --network host \
  --user 10001:10001 \
  --mount "type=volume,source=${socket_volume},target=/run/robonix-go2-time" \
  --env GO2_TIME_ROLE=feeder \
  --env "GO2_TIME_PROFILE=${mode}" \
  --env "GO2_TIME_FORMAL_GATE=${exact_gate}" \
  --env "GO2_TIME_APPROVAL_SHA256=${approval_sha256}" \
  --env "GO2_TIME_APPROVAL_B64=${approval_b64}" \
  "${image}" >/dev/null
unset approval_b64 verified_approval

# Recheck the host after both starts to close the ordinary start-up race.  The
# only ignored process is PID 1 of our reviewed chronyd container.
if ! competing_clock_services \
  || ! competing_clock_processes "${chronyd_host_pid}" \
  || ! competing_sys_time_containers "${chronyd_container}"; then
  echo "competing servo appeared during launch; removing both time containers" >&2
  cleanup_failed_start
  exit 54
fi

echo "Waiting for feeder freshness and chrony GO2 selection readiness."
deadline=$((SECONDS + 180))
refclock_selected=0
while (( SECONDS < deadline )); do
  feeder_health="$(docker inspect --format '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' "${feeder_container}")"
  if [[ "${feeder_health}" == "healthy" \
    && "$(docker inspect --format '{{.State.Running}}' "${chronyd_container}")" == "true" ]] \
    && chronyd_caps_exact \
    && chronyd_single_process \
    && chronyd_selected_go2; then
    refclock_selected=1
    echo "REFCLOCK SELECTED: beginning mandatory five-minute quality audit."
    break
  fi
  if [[ "$(docker inspect --format '{{.State.Running}}' "${feeder_container}")" != "true" \
    || "$(docker inspect --format '{{.State.Running}}' "${chronyd_container}")" != "true" ]]; then
    echo "a required time container exited before readiness" >&2
    exit 55
  fi
  sleep 2
done
if (( refclock_selected == 0 )); then
  echo "readiness timeout; removing both time containers" >&2
  cleanup_failed_start
  exit 56
fi

readonly container_runtime_root="/dev/shm/robonix-time"
set +e
docker exec --user 10001:10001 "${feeder_container}" \
  python3 /opt/robonix/deploy/time-sync/post_bootstrap_quality_gate.py \
  --health-file "${container_runtime_root}/feeder-health.json" \
  --approval-file "${container_runtime_root}/go2-clock-ref-approval.json" \
  --output "${container_runtime_root}/post-bootstrap-quality.json" \
  --samples-output "${container_runtime_root}/post-bootstrap-quality-samples.jsonl"
post_gate_status=$?
set -e

for filename in post-bootstrap-quality.json post-bootstrap-quality-samples.jsonl; do
  if ! docker cp \
    "${feeder_container}:${container_runtime_root}/${filename}" \
    "${bundle_path}/${filename}"; then
    echo "failed to retain the post-bootstrap quality audit" >&2
    cleanup_failed_start
    exit 57
  fi
  chmod 0600 "${bundle_path}/${filename}"
done
if (( post_gate_status != 0 )); then
  echo "five-minute post-bootstrap quality gate failed closed" >&2
  cleanup_failed_start
  exit 58
fi
if [[ "$(docker inspect --format '{{.State.Running}}' "${feeder_container}")" != "true" \
  || "$(docker inspect --format '{{.State.Running}}' "${chronyd_container}")" != "true" ]] \
  || ! chronyd_caps_exact \
  || ! chronyd_single_process \
  || ! chronyd_selected_go2; then
  echo "time components lost readiness after the five-minute audit" >&2
  cleanup_failed_start
  exit 59
fi
trap - ERR INT TERM
echo "READY: audited continuous five-minute offset/drift gate passed."
