#!/usr/bin/env bash
set -euo pipefail

readonly expected_arch="aarch64"
readonly expected_interface="eth0"
readonly expected_cidr="192.168.123.18/24"

if [[ "$(uname -m)" != "$expected_arch" ]]; then
  echo "jetson-readonly requires ARM64 (${expected_arch})" >&2
  exit 10
fi
if [[ ! -d "/sys/class/net/${expected_interface}" ]]; then
  echo "required interface is absent: ${expected_interface}" >&2
  exit 11
fi
if [[ -r "/sys/class/net/${expected_interface}/carrier" ]] \
  && [[ "$(<"/sys/class/net/${expected_interface}/carrier")" != "1" ]]; then
  echo "required interface has no carrier: ${expected_interface}" >&2
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

echo "READ-ONLY network check passed: ${expected_interface}=${expected_cidr}"
