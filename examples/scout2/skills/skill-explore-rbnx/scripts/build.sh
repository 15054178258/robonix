#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# explore_rbnx build phase — runs `rbnx codegen` (emits proto_gen for
# both atlas-side stubs and the package's own srv definitions under
# capabilities/srv/), then `docker build`. Same shape as mapping_rbnx
# and system/scene.
#
# RBNX_BUILD_CLEAN=1 wipes rbnx-build/ and rebuilds without docker cache.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# shellcheck disable=SC1091
source "$PKG/scripts/docker_base_image.sh"
cd "$PKG"

BUILD="rbnx-build"
CLEAN="${RBNX_BUILD_CLEAN:-}"
IMG="${ROBONIX_EXPLORE_IMAGE:-robonix-explore}"
ROS_BASE_IMAGE="${ROBONIX_EXPLORE_ROS_BASE_IMAGE:-robonix-ros:humble-ros-base}"
UPSTREAM_ROS_BASE_IMAGE="ros:humble-ros-base"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. Codegen (atlas proto stubs + this package's capabilities/srv/*.srv) ──
if command -v rbnx >/dev/null 2>&1; then
    FLAGS=(--mcp)
    [[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
    echo "[build] rbnx codegen ${FLAGS[*]}"
    bash "$PKG/scripts/run_python_codegen.sh" "$PKG" "${FLAGS[@]}"
else
    echo "[build] WARNING: rbnx not in PATH — skipping proto codegen"
    echo "[build]   install robonix-cli + run \`rbnx setup\` once from the robonix source root"
fi

# ── 2. Docker image ─────────────────────────────────────────────────────────
if ! command -v docker >/dev/null 2>&1; then
    echo "[build] error: docker not found on PATH" >&2
    exit 1
fi

DOCKER_BUILD_FLAGS=(--network=host --pull=false --build-arg "ROS_BASE_IMAGE=${ROS_BASE_IMAGE}")
robonix_ensure_local_base_image "$ROS_BASE_IMAGE" "$UPSTREAM_ROS_BASE_IMAGE"
[[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)

if [[ "$CLEAN" != "1" ]] && docker image inspect "$IMG" >/dev/null 2>&1; then
    echo "[build] image $IMG present; rebuilding incrementally"
fi

echo "[build] docker build -f docker/Dockerfile -t $IMG"
docker build "${DOCKER_BUILD_FLAGS[@]}" -f docker/Dockerfile -t "$IMG" docker/

echo "[build] done."
