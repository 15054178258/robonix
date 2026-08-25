#!/usr/bin/env bash
set -euo pipefail
umask 0077

readonly role="${GO2_TIME_ROLE:-probe}"
readonly profile="${GO2_TIME_PROFILE:-probe}"
readonly runtime_root="/dev/shm/robonix-time"
readonly socket_root="/run/robonix-go2-time"
readonly refclock_socket="${socket_root}/refclock/go2.sock"
readonly enable_file="${runtime_root}/ENABLE_GO2_REFCLOCK"
readonly approval_file="${runtime_root}/go2-clock-ref-approval.json"
readonly evidence_root="${runtime_root}/evidence"
readonly health_file="${runtime_root}/feeder-health.json"
readonly exact_gate="I_APPROVE_NX_HOST_CLOCK_DISCIPLINE_V1"
readonly profile_root="/opt/robonix/time-sync"
readonly refclock_program="/opt/robonix/deploy/time-sync/go2_clock_ref.py"
readonly probe_program="/opt/robonix/scripts/probe_go2_time_readonly.py"
readonly zero_cap_hex="0000000000000000"

if [[ "$#" -ne 0 ]]; then
  echo "jetson-time-sync rejects command/pass-through arguments" >&2
  exit 2
fi
case "${role}" in
  probe|graph-preflight|feeder) ;;
  *) echo "invalid GO2_TIME_ROLE: ${role}" >&2; exit 2 ;;
esac

capability_value() {
  local field="$1"
  awk -v wanted="${field}:" '$1 == wanted {print tolower($2)}' /proc/self/status
}

assert_zero_capabilities() {
  local field value
  for field in CapEff CapPrm CapInh CapAmb CapBnd; do
    value="$(capability_value "${field}")"
    if [[ "${value}" != "${zero_cap_hex}" ]]; then
      echo "${role} requires ${field}=0, observed ${value:-missing}" >&2
      exit 4
    fi
  done
}

decode_formal_approval() {
  if [[ "${GO2_TIME_FORMAL_GATE:-}" != "${exact_gate}" ]]; then
    echo "formal profile gate is absent or not exact" >&2
    exit 5
  fi
  if [[ -z "${GO2_TIME_APPROVAL_B64:-}" \
    || -z "${GO2_TIME_APPROVAL_SHA256:-}" ]]; then
    echo "formal profile requires recomputed approval payload and SHA-256" >&2
    exit 5
  fi
  if (( ${#GO2_TIME_APPROVAL_B64} > 131072 )) \
    || [[ ! "${GO2_TIME_APPROVAL_SHA256}" =~ ^[0-9a-f]{64}$ ]]; then
    echo "formal approval payload is malformed or too large" >&2
    exit 5
  fi
  mkdir -m 0700 -p "${runtime_root}"
  printf 'ENABLE_GO2_CHRONY_REFCLOCK_V1\n' >"${enable_file}"
  printf '%s' "${GO2_TIME_APPROVAL_B64}" | base64 --decode >"${approval_file}"
  chmod 0600 "${enable_file}" "${approval_file}"
  actual_sha256="$(sha256sum "${approval_file}" | awk '{print $1}')"
  if [[ "${actual_sha256}" != "${GO2_TIME_APPROVAL_SHA256}" ]]; then
    echo "decoded recomputed approval SHA-256 mismatch" >&2
    exit 5
  fi
  unset GO2_TIME_APPROVAL_B64
}

assert_zero_capabilities
"${profile_root}/validate-network.sh"
for required in python3 base64 sha256sum; do
  command -v "${required}" >/dev/null 2>&1 || {
    echo "missing runtime command: ${required}" >&2
    exit 3
  }
done

mkdir -m 0700 -p "${runtime_root}" "${runtime_root}/home"
export HOME="${runtime_root}/home"
export ROS_LOG_DIR="${runtime_root}/ros-log"
export XDG_RUNTIME_DIR="${runtime_root}/xdg"
mkdir -m 0700 -p "${ROS_LOG_DIR}" "${XDG_RUNTIME_DIR}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /opt/robonix/overlay/setup.bash
set -u

if [[ "${role}" == "probe" ]]; then
  duration="${GO2_TIME_PROBE_DURATION_SECONDS:-60}"
  max_samples="${GO2_TIME_PROBE_MAX_SAMPLES:-50000}"
  if [[ ! "${duration}" =~ ^[1-9][0-9]*$ \
    || ! "${max_samples}" =~ ^[1-9][0-9]*$ \
    || "${duration}" -gt 86400 \
    || "${max_samples}" -gt 10000000 ]]; then
    echo "invalid bounded probe duration/sample limit" >&2
    exit 2
  fi
  echo "READ-ONLY probe: all five capability sets are zero; no chronyd"
  exec python3 "${probe_program}" \
    --output-dir "${evidence_root}" \
    --duration-seconds "${duration}" \
    --max-samples "${max_samples}"
fi

decode_formal_approval

if [[ "${role}" == "graph-preflight" ]]; then
  echo "NO-ADJUST preflight: live writer GID must match raw-PCAP-proven source"
  exec python3 "${refclock_program}" \
    --mode verify-graph \
    --duration-seconds 30 \
    --startup-timeout-seconds 20 \
    --enable-file "${enable_file}" \
    --approval-file "${approval_file}" \
    --gate-owner-uid "$(id -u)"
fi

if [[ "${role}" != "feeder" || "${profile}" == "probe" ]]; then
  echo "invalid feeder role/profile combination" >&2
  exit 2
fi
for _ in $(seq 1 200); do
  [[ -S "${refclock_socket}" ]] && break
  sleep 0.1
done
if [[ ! -S "${refclock_socket}" ]]; then
  echo "chronyd did not create the dedicated refclock socket" >&2
  exit 6
fi
echo "FORMAL FEEDER: ROS/DDS process has CapEff/Prm/Inh/Amb/Bnd all zero"
exec python3 "${refclock_program}" \
  --mode feed-chrony \
  --duration-seconds 0 \
  --startup-timeout-seconds 20 \
  --enable-file "${enable_file}" \
  --approval-file "${approval_file}" \
  --gate-owner-uid "$(id -u)" \
  --chrony-socket "${refclock_socket}" \
  --health-file "${health_file}"
