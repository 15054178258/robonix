#!/usr/bin/env bash
set -euo pipefail
docker rm -f "robonix_${RBNX_INSTANCE_NAME:-robot_description}" >/dev/null 2>&1 || true

