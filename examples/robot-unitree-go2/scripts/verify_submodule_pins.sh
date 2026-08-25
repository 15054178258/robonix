#!/usr/bin/env bash
# Fail closed unless safety-critical service sources match their gitlink pins.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

verify_pin() {
  local name="$1"
  local rel="$2"
  local expected_url="$3"
  local path="$ROOT/$rel"
  local entry mode pinned actual configured origin dirty

  [[ -d "$path" ]] || {
    echo "missing pinned submodule: $rel; run git submodule update --init --recursive" >&2
    exit 1
  }
  entry="$(git -C "$ROOT" ls-files --stage -- "$rel")"
  read -r mode pinned _ <<<"$entry"
  [[ "$mode" == 160000 && "$pinned" =~ ^[0-9a-f]{40}$ ]] || {
    echo "not a gitlink pin: $rel" >&2
    exit 1
  }
  actual="$(git -C "$path" rev-parse HEAD)"
  [[ "$actual" == "$pinned" ]] || {
    echo "submodule HEAD mismatch: $rel expected=$pinned actual=$actual" >&2
    exit 1
  }
  configured="$(git -C "$ROOT" config -f .gitmodules --get "submodule.$name.url")"
  origin="$(git -C "$path" remote get-url origin)"
  [[ "$configured" == "$expected_url" && "$origin" == "$expected_url" ]] || {
    echo "submodule URL mismatch: $rel expected=$expected_url configured=$configured origin=$origin" >&2
    exit 1
  }
  dirty="$(git -C "$path" status --porcelain --untracked-files=normal)"
  [[ -z "$dirty" ]] || {
    echo "pinned submodule is dirty: $rel" >&2
    printf '%s\n' "$dirty" >&2
    exit 1
  }
  printf '[source-pin] %s %s\n' "$rel" "$actual"
}

verify_pin \
  third_party/service-map-rbnx \
  third_party/service-map-rbnx \
  https://github.com/syswonder/service-map-rbnx.git
verify_pin \
  third_party/service-navigation-rbnx \
  third_party/service-navigation-rbnx \
  https://github.com/syswonder/service-navigation-rbnx.git

status="$(git -C "$ROOT" submodule status --recursive)"
if grep -Eq '^[-+U]' <<<"$status"; then
  echo "one or more recursive submodules are uninitialized or do not match their gitlink" >&2
  printf '%s\n' "$status" >&2
  exit 1
fi

echo '[source-pin] PASS'
