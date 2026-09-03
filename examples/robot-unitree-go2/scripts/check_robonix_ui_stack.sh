#!/usr/bin/env bash
set -euo pipefail

echo "================================================================"
echo " READ-ONLY ROBONIX UI CHECK — no publishers and no motion APIs"
echo "================================================================"

readonly TIMEOUT_SECONDS="${ROBONIX_UI_CHECK_TIMEOUT_SECONDS:-3}"
readonly urls=(
  "http://127.0.0.1:50107/"
  "http://127.0.0.1:8091/api/state"
  "http://127.0.0.1:8092/healthz"
  "http://127.0.0.1:7860/api/defaults"
)

status=0
for url in "${urls[@]}"; do
  if curl --fail --silent --show-error \
    --max-time "$TIMEOUT_SECONDS" --output /dev/null "$url"; then
    printf 'PASS %s\n' "$url"
  else
    printf 'FAIL %s\n' "$url" >&2
    status=1
  fi
done
exit "$status"
