#!/usr/bin/env bash
set -euo pipefail

readonly expected_arch="aarch64"
readonly expected_interface="eth0"
readonly expected_cidr="192.168.123.18/24"

if [[ "$(uname -m)" != "$expected_arch" ]]; then
  echo "jetson-full-nomotion requires ARM64 (${expected_arch})" >&2
  exit 10
fi
if [[ ! -d "/sys/class/net/${expected_interface}" ]]; then
  echo "required robot interface is absent: ${expected_interface}" >&2
  exit 11
fi
if [[ ! -r "/sys/class/net/${expected_interface}/carrier" ]] \
  || [[ "$(<"/sys/class/net/${expected_interface}/carrier")" != "1" ]]; then
  echo "required robot interface has no carrier: ${expected_interface}" >&2
  exit 12
fi

mapfile -t addresses < <(
  ip -o -4 address show dev "$expected_interface" scope global \
    | awk '{print $4}'
)
if [[ "${#addresses[@]}" -ne 1 || "${addresses[0]:-}" != "$expected_cidr" ]]; then
  echo "${expected_interface} must have exactly ${expected_cidr}; found: ${addresses[*]:-(none)}" >&2
  echo "this check never modifies network configuration" >&2
  exit 13
fi

mapfile -t robot_defaults < <(
  ip -o -4 route show default dev "$expected_interface"
)
if (( ${#robot_defaults[@]} != 0 )); then
  echo "${expected_interface} must not carry a default route" >&2
  exit 14
fi

mapfile -t global_ipv6 < <(
  ip -o -6 address show dev "$expected_interface" scope global
)
if (( ${#global_ipv6[@]} != 0 )); then
  echo "${expected_interface} must not carry a global IPv6 address" >&2
  exit 15
fi
readonly ipv6_disable="/proc/sys/net/ipv6/conf/${expected_interface}/disable_ipv6"
if [[ ! -r "$ipv6_disable" || "$(<"$ipv6_disable")" != "1" ]]; then
  echo "IPv6 must be disabled on ${expected_interface}" >&2
  exit 16
fi

echo "NO-MOTION network gate passed: ${expected_interface}=${expected_cidr}; no default route or IPv6"
