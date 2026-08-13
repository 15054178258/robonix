#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
NO_ARM_MANIFEST="$DEPLOY_DIR/robonix_manifest.no-arm.yaml"

if [[ ! -f "$NO_ARM_MANIFEST" ]]; then
  echo "missing no-arm manifest: $NO_ARM_MANIFEST" >&2
  exit 1
fi

# Do not inherit a caller-provided manifest: this entrypoint must always use
# the persistent no-arm deployment profile.
export ROBONIX_MANIFEST="$NO_ARM_MANIFEST"

exec "$DEPLOY_DIR/start.sh" "$@"
