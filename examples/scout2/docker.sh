#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scout2 driver container — build, start, or stop.
#
# Usage:
#   bash examples/scout2/docker.sh build   # build Docker image
#   bash examples/scout2/docker.sh up      # start container
#   bash examples/scout2/docker.sh down    # stop container
set -euo pipefail

DIR="$(cd "$(dirname "$0")/docker" && pwd)"
CT="${ROBONIX_SCOUT2_CONTAINER:-scout2-driver}"

case "${1:-}" in
  build)
    echo "[scout2] building docker image..."
    docker compose -f "$DIR/compose.yaml" build
    echo "[scout2] image built."
    ;;
  up)
    echo "[scout2] starting container..."
    docker compose -f "$DIR/compose.yaml" up -d
    echo "[scout2] container '$CT' started."
    ;;
  down)
    echo "[scout2] stopping container..."
    docker compose -f "$DIR/compose.yaml" down
    echo "[scout2] container '$CT' stopped."
    ;;
  *)
    echo "Usage: $0 {build|up|down}"
    exit 1
    ;;
esac
