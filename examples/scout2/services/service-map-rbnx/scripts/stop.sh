#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

NATIVE_PLATFORMS=("jetson_orin")
is_native_platform() {
    local p="$1"
    for w in "${NATIVE_PLATFORMS[@]}"; do
        [[ "$p" == "$w" ]] && return 0
    done
    return 1
}

MODE=""
case "${ROBONIX_MAPPING_FORCE:-}" in
    native) MODE=native ;;
    docker) MODE=docker ;;
    "") ;;
    *) echo "[mapping/stop] ROBONIX_MAPPING_FORCE=${ROBONIX_MAPPING_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac
if [[ -z "$MODE" ]]; then
    if is_native_platform "${ROBONIX_MAPPING_PLATFORM:-}"; then MODE=native; else MODE=docker; fi
fi

echo "[mapping/stop] mode=${MODE}"
if [[ "$MODE" == "docker" ]]; then
    docker rm -f "${ROBONIX_MAPPING_CONTAINER:-robonix_mapping}" >/dev/null 2>&1 || true
    exit 0
fi

pkill -TERM -f "${PKG}.*mapping_rbnx.atlas_bridge" 2>/dev/null || true
pkill -TERM -f "${PKG}/scripts/start_engine.sh" 2>/dev/null || true
sleep 1
pkill -KILL -f "${PKG}.*mapping_rbnx.atlas_bridge" 2>/dev/null || true
pkill -KILL -f "${PKG}/scripts/start_engine.sh" 2>/dev/null || true
