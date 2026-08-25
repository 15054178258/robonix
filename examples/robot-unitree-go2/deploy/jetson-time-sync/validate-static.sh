#!/usr/bin/env bash
set -euo pipefail

readonly script_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly repository_root="$(CDPATH= cd -- "${script_root}/../.." && pwd)"

for script in \
  "${script_root}/build.sh" \
  "${script_root}/entrypoint.sh" \
  "${script_root}/healthcheck.sh" \
  "${script_root}/run.sh" \
  "${script_root}/status.sh" \
  "${script_root}/stop.sh" \
  "${script_root}/validate-network.sh"; do
  bash -n "${script}"
done
python3 -m py_compile \
  "${repository_root}/deploy/time-sync/go2_time_core.py" \
  "${repository_root}/deploy/time-sync/evidence_bundle.py" \
  "${repository_root}/deploy/time-sync/go2_clock_ref.py" \
  "${repository_root}/deploy/time-sync/post_bootstrap_quality_gate.py" \
  "${repository_root}/scripts/probe_go2_time_readonly.py" \
  "${repository_root}/scripts/prepare_go2_time_approval.py" \
  "${repository_root}/scripts/correlate_rtps_writer_locator.py" \
  "${script_root}/validate_approval.py"

if grep -ERn \
  'create_publisher|/cmd_vel|/lowcmd|/api/sport/request|SportClient|clock_settime|settimeofday|timedatectl' \
  "${repository_root}/deploy/time-sync/"*.py \
  "${repository_root}/scripts/probe_go2_time_readonly.py"; then
  echo "time tools contain a forbidden publisher, motion, or direct clock API" >&2
  exit 20
fi

runtime_files=(
  "${script_root}/entrypoint.sh"
  "${script_root}/run.sh"
)
if grep -En -- \
  '--privileged|--device([=[:space:]]|$)|type=bind|--volume([=[:space:]]|$)|docker\.sock|--pid[=[:space:]]*host|--ipc[=[:space:]]*host' \
  "${runtime_files[@]}"; then
  echo "time components must have no privilege mode, device, host bind, or Docker socket" >&2
  exit 21
fi
if grep -Eq '^[[:space:]]*VOLUME([[:space:]]|$)' "${script_root}/Dockerfile"; then
  echo "image must not declare an implicit persistent volume" >&2
  exit 21
fi

grep -Fq -- '--network host' "${script_root}/run.sh"
grep -Fq -- '--network none' "${script_root}/run.sh"
grep -Fq -- '--read-only' "${script_root}/run.sh"
grep -Fq -- '--cap-drop ALL' "${script_root}/run.sh"
grep -Fq -- '--cap-add SYS_TIME' "${script_root}/run.sh"
grep -Fq 'GO2_TIME_ROLE=feeder' "${script_root}/run.sh"
grep -Fq 'type=volume,source=${socket_volume}' "${script_root}/run.sh"
grep -Fq 'validate_approval.py' "${script_root}/run.sh"
grep -Fq -- '--bundle' "${script_root}/run.sh"
grep -Fq 'GO2_TIME_PROFILE=probe' "${script_root}/run.sh"
grep -Fq 'GO2_TIME_ALL_ROS_CONTAINERS_STOPPED' "${script_root}/run.sh"
grep -Fq "mapfile -t running_containers" "${script_root}/run.sh"
grep -Fq 'GO2_TIME_FORMAL_GATE' "${script_root}/entrypoint.sh"
grep -Fq 'assert_zero_capabilities' "${script_root}/entrypoint.sh"
grep -Fq -- '--entrypoint /usr/sbin/chronyd' "${script_root}/run.sh"
grep -Fq -- '--no-healthcheck' "${script_root}/run.sh"
grep -Fq -- '--user 10002:10002' "${script_root}/run.sh"
grep -Fq -- '"${image}" -U -F 2' "${script_root}/run.sh"
grep -Fq 'chronyd_caps_exact' "${script_root}/run.sh"
grep -Fq 'chronyd_selected_go2' "${script_root}/run.sh"
grep -Fq 'install -d -o 10002 -g 10001 -m 2770' "${script_root}/Dockerfile"
grep -Fq '/run/robonix-go2-time/control' "${script_root}/Dockerfile"
grep -Fq '/run/robonix-go2-time/state' "${script_root}/Dockerfile"
if grep -Fq 'chronyc' "${script_root}/healthcheck.sh" "${script_root}/status.sh"; then
  echo "zero-capability feeder/status path must not have chronyd command access" >&2
  exit 21
fi
for capability in CapEff CapPrm CapInh CapAmb CapBnd; do
  grep -Fq "${capability}" "${script_root}/entrypoint.sh"
done

bootstrap="${script_root}/config/chrony-bootstrap.conf"
steady="${script_root}/config/chrony-steady.conf"
[[ "$(grep -Ec '^[[:space:]]*makestep[[:space:]]' "${bootstrap}")" == 1 ]]
if grep -Eq '^[[:space:]]*makestep[[:space:]]' "${steady}"; then
  echo "steady chrony profile must never step time" >&2
  exit 22
fi
if grep -E \
  '^[[:space:]]*(server|pool|peer|local|allow|rtcsync)[[:space:]]' \
  "${bootstrap}" "${steady}"; then
  echo "time sidecar must use only the approved Go2 refclock" >&2
  exit 23
fi
for config in "${bootstrap}" "${steady}"; do
  grep -Fq 'refclock SOCK /run/robonix-go2-time/refclock/go2.sock' "${config}"
  grep -Fq 'bindcmdaddress /run/robonix-go2-time/control/chronyd.sock' "${config}"
  grep -Fq 'driftfile /run/robonix-go2-time/state/drift' "${config}"
  grep -Fq 'pidfile /run/robonix-go2-time/state/chronyd.pid' "${config}"
  grep -Fq 'cmdport 0' "${config}"
  grep -Fq 'port 0' "${config}"
done

echo "jetson-time-sync static safety validation passed"
