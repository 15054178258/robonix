#!/usr/bin/env bash
# Static fail-closed contract check for every Navigation source/cache that
# Robonix could execute. It starts no process from the inspected checkout.

set -euo pipefail

(( $# > 0 )) || {
  echo "usage: bash check_navigation_velocity_contract.sh NAVIGATION_ROOT [...]" >&2
  exit 2
}

require_text() {
  local root="$1" relative="$2" expected="$3" label="$4"
  local path="$root/$relative"
  [[ -f "$path" ]] || {
    echo "unsafe Navigation source: missing $path" >&2
    exit 1
  }
  grep -Fq -- "$expected" "$path" || {
    echo "unsafe Navigation source: $label is absent from $path" >&2
    exit 1
  }
}

for root in "$@"; do
  [[ -d "$root" ]] || {
    echo "unsafe Navigation source: directory is missing: $root" >&2
    exit 1
  }

  require_text "$root" config.spec \
    'velocity_output_topic:' 'configurable velocity output contract'
  require_text "$root" nav2_wrapper/configuration.py \
    'def resolve_velocity_output_topic(' 'validated velocity resolver'
  require_text "$root" nav2_wrapper/atlas_bridge.py \
    'resolve_velocity_output_topic(cfg)' 'config validation before Nav2 startup'
  require_text "$root" nav2_wrapper/velocity_guard.py \
    'create_publisher(Twist, output_topic, 10)' 'resolved velocity publisher'

  if grep -Fq -- 'create_publisher(Twist, "/cmd_vel", 10)' \
    "$root/nav2_wrapper/velocity_guard.py"; then
    echo "unsafe Navigation source: fixed /cmd_vel publisher remains in $root" >&2
    exit 1
  fi
done

echo "[navigation-velocity-contract] PASS"
