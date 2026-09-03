#!/usr/bin/env bash
# Read-only inventory for the Go2 EDU onboard NVIDIA Jetson/Orin computer.

set -uo pipefail

COMMAND_TIMEOUT_SECONDS="${NX_AUDIT_COMMAND_TIMEOUT_SECONDS:-8}"

if [[ ! "${COMMAND_TIMEOUT_SECONDS}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'NX_AUDIT_COMMAND_TIMEOUT_SECONDS must be a positive integer.\n' >&2
  exit 2
fi
if ! command -v timeout >/dev/null 2>&1; then
  printf 'timeout is required; refusing to run an unbounded audit.\n' >&2
  exit 127
fi

cat <<'BANNER'
=======================================================================
  READ-ONLY / 只读审计：Go2 EDU NX 机载计算单元
  不使用 sudo，不安装软件，不修改网络、时间、服务、文件或机器人状态。
  不枚举或打印环境变量、SSH 密钥、shell 历史或其他凭据。
=======================================================================
BANNER

run() {
  local label="$1"
  shift
  printf '\n## %s\n' "${label}"
  printf '$'
  printf ' %q' "$@"
  printf '\n'
  timeout --signal=INT --kill-after=1s "${COMMAND_TIMEOUT_SECONDS}s" "$@" 2>&1
  local status=$?
  if (( status != 0 )); then
    printf '[status=%d]\n' "${status}"
  fi
  return 0
}

optional_run() {
  local label="$1"
  local command="$2"
  shift 2
  if command -v "${command}" >/dev/null 2>&1; then
    run "${label}" "${command}" "$@"
  else
    printf '\n## %s\nNOT INSTALLED: %s\n' "${label}" "${command}"
  fi
}

read_public_file() {
  local label="$1"
  local path="$2"
  if [[ -r "${path}" ]]; then
    run "${label}" cat -- "${path}"
  else
    printf '\n## %s\nNOT READABLE: %s\n' "${label}" "${path}"
  fi
}

run "UTC wall clock" date -u '+epoch=%s.%N utc=%FT%T.%NZ'
read_public_file "Kernel uptime" /proc/uptime
optional_run "timedatectl status" timedatectl status
optional_run "timedatectl timesync status" timedatectl timesync-status

read_public_file "OS release" /etc/os-release
run "Kernel and architecture" uname -a
optional_run "Hostname" hostnamectl status
optional_run "CPU inventory" lscpu
optional_run "Memory" free -h
optional_run "Filesystem capacity" df -hT

read_public_file "Jetson Linux release" /etc/nv_tegra_release
if [[ -r /proc/device-tree/model ]] && command -v strings >/dev/null 2>&1; then
  run "Device-tree model" strings /proc/device-tree/model
else
  printf '\n## Device-tree model\nNOT AVAILABLE\n'
fi
if command -v dpkg-query >/dev/null 2>&1; then
  run "NVIDIA L4T packages" dpkg-query -W 'nvidia-l4t-*'
else
  printf '\n## NVIDIA L4T packages\nNOT INSTALLED: dpkg-query\n'
fi
optional_run "NVIDIA SMI" nvidia-smi
optional_run "CUDA compiler" nvcc --version
optional_run "Python" python3 --version

optional_run "Network links" ip -br link
optional_run "Network addresses" ip -br addr
optional_run "IPv4 routes" ip -4 route show
optional_run "IPv6 routes" ip -6 route show
optional_run "NetworkManager devices" nmcli device status
optional_run "Listening sockets" ss -lntup

if [[ -d /opt/ros ]]; then
  run "Installed ROS roots" find /opt/ros -mindepth 1 -maxdepth 1 -type d -print
else
  printf '\n## Installed ROS roots\nNOT FOUND: /opt/ros\n'
fi
optional_run "colcon" colcon --help
optional_run "CMake" cmake --version
optional_run "Git" git --version

if command -v docker >/dev/null 2>&1; then
  run "Docker client" docker --version
  if command -v systemctl >/dev/null 2>&1 &&
    timeout --signal=INT --kill-after=1s "${COMMAND_TIMEOUT_SECONDS}s" \
      systemctl is-active --quiet docker.service &&
    [[ -S /var/run/docker.sock ]]
  then
    # The daemon is queried only if it was already active. Pinning the local
    # Unix socket and an intentionally nonexistent config directory prevents
    # inherited Docker contexts, remote hosts, TLS credentials, or user config
    # from redirecting this read-only inventory.
    run "Docker client/server" docker --config /nonexistent-go2-audit \
      --host unix:///var/run/docker.sock version
    run "Docker containers" docker --config /nonexistent-go2-audit \
      --host unix:///var/run/docker.sock ps --no-trunc \
      --format '{{.ID}} {{.Image}} {{.Status}} {{.Names}}'
    run "Docker images" docker --config /nonexistent-go2-audit \
      --host unix:///var/run/docker.sock images --no-trunc \
      --format '{{.ID}} {{.Repository}}:{{.Tag}} {{.Size}}'
  else
    printf '\n## Docker daemon inventory\nSKIPPED: docker.service was not already active or the local socket is absent\n'
  fi
else
  printf '\n## Docker\nNOT INSTALLED: docker\n'
fi

optional_run "Running services" systemctl --no-pager --plain --type=service --state=running
optional_run "Processes (arguments intentionally omitted)" ps -eo pid,user,stat,comm --sort=pid

printf '\nREAD-ONLY NX audit complete. No changes were requested or performed.\n'
