#!/usr/bin/env bash
# Inspect the local host's Go2-facing network state without changing it.

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${1:-${PROJECT_ROOT}/logs/go2-readonly/${STAMP}-network}"
OUTPUT_DIR="$(realpath -m -- "${OUTPUT_DIR}")"
REPORT="${OUTPUT_DIR}/network-report.txt"

case "${OUTPUT_DIR}" in
  "${PROJECT_ROOT}/logs/"*) ;;
  *)
    printf '错误：输出目录必须位于 %s/logs/ 内。\n' "${PROJECT_ROOT}" >&2
    exit 2
    ;;
esac

cat <<'BANNER'
=======================================================================
  READ-ONLY / 只读审计：Go2 网络检查
  仅读取本机接口、地址、路由、邻居缓存和 DDS 环境。
  不修改 NetworkManager，不设置地址/网关/DNS，不主动探测机器人。
=======================================================================
BANNER

mkdir -p -- "${OUTPUT_DIR}"
umask 077

run_readonly() {
  local title="$1"
  shift

  {
    printf '\n## %s\n' "${title}"
    printf '$'
    printf ' %q' "$@"
    printf '\n'
    "$@"
    local status=$?
    if (( status != 0 )); then
      printf '[exit status: %d]\n' "${status}"
    fi
  } >>"${REPORT}" 2>&1
}

{
  printf 'READ-ONLY Go2 network audit\n'
  printf 'UTC timestamp: %s\n' "${STAMP}"
  printf 'Project root: %s\n' "${PROJECT_ROOT}"
  printf 'Expected dedicated host address (not configured here): 192.168.123.99/24\n'
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-<unset>}"
  printf 'CYCLONEDDS_URI=%s\n' "${CYCLONEDDS_URI:-<unset>}"
} >"${REPORT}"

if command -v ip >/dev/null 2>&1; then
  run_readonly "Links (brief)" ip -br link
  run_readonly "Addresses (brief)" ip -br addr
  run_readonly "IPv4 routes" ip -4 route show
  run_readonly "IPv6 routes" ip -6 route show
  run_readonly "Cached neighbours" ip neigh show
  run_readonly "Multicast memberships" ip maddr show
else
  printf '\n## ip\nMISSING: ip command is not available.\n' >>"${REPORT}"
fi

if command -v nmcli >/dev/null 2>&1; then
  run_readonly "NetworkManager device status" nmcli device status
  run_readonly "NetworkManager device details" nmcli --fields GENERAL.DEVICE,GENERAL.TYPE,GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY,IP4.DNS device show
  run_readonly "NetworkManager profiles (read only)" nmcli --fields NAME,UUID,TYPE,DEVICE,AUTOCONNECT connection show
else
  printf '\n## nmcli\nMISSING: nmcli is not available.\n' >>"${REPORT}"
fi

{
  printf '\n## Interface candidates\n'
  printf 'interface\tkind\toperstate\tcarrier\tmac\taddresses\n'
  for iface_path in /sys/class/net/*; do
    [[ -e "${iface_path}" ]] || continue
    iface="${iface_path##*/}"
    [[ "${iface}" == "lo" ]] && continue

    kind="wired-physical-candidate"
    resolved="$(readlink -f -- "${iface_path}" 2>/dev/null || true)"
    if [[ -d "${iface_path}/wireless" ]]; then
      kind="wireless"
    elif [[ "${resolved}" == *"/virtual/"* ]]; then
      kind="virtual"
    fi

    operstate="$(cat -- "${iface_path}/operstate" 2>/dev/null || printf 'unknown')"
    carrier="$(cat -- "${iface_path}/carrier" 2>/dev/null || printf 'unknown')"
    mac="$(cat -- "${iface_path}/address" 2>/dev/null || printf 'unknown')"
    addresses="<ip unavailable>"
    if command -v ip >/dev/null 2>&1; then
      addresses="$(ip -o addr show dev "${iface}" 2>/dev/null | awk '{print $3 "=" $4}' | paste -sd, -)"
      [[ -n "${addresses}" ]] || addresses="<none>"
    fi
    printf '%s\t%s\t%s\t%s\t%s\t%s\n' "${iface}" "${kind}" "${operstate}" "${carrier}" "${mac}" "${addresses}"
  done

  printf '\n## Dedicated-address check\n'
  if command -v ip >/dev/null 2>&1 && ip -o -4 addr show | awk '$4 == "192.168.123.99/24" {found=1} END {exit !found}'; then
    ip -o -4 addr show | awk '$4 == "192.168.123.99/24" {print "FOUND on " $2 ": " $4}'
  else
    printf 'NOT FOUND: 192.168.123.99/24 is not currently assigned. No change was made.\n'
  fi
} >>"${REPORT}" 2>&1

printf '只读网络报告已保存：%s\n' "${REPORT}"
