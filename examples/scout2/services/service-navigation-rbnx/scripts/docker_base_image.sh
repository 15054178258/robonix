#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Prepare a local Docker base-image alias so BuildKit does not query remote
# registry metadata on every rebuild. First use pulls through configured
# domestic mirrors; later rebuilds use the local alias directly.

robonix_ensure_local_base_image() {
    local dest="$1"
    local upstream="$2"
    local mirrors_raw="${ROBONIX_DOCKER_MIRRORS:-${ROBONIX_DOCKER_MIRROR:-docker.m.daocloud.io,dockerproxy.net}}"
    local image_path="$upstream"
    local candidates=()

    if docker image inspect "$dest" >/dev/null 2>&1; then
        return 0
    fi

    if docker image inspect "$upstream" >/dev/null 2>&1; then
        echo "[docker-base] tagging cached $upstream as $dest"
        docker tag "$upstream" "$dest"
        return 0
    fi

    image_path="${image_path#docker.io/}"
    image_path="${image_path#registry-1.docker.io/}"
    if [[ "$image_path" != */* ]]; then
        image_path="library/$image_path"
    fi

    IFS=',' read -r -a _mirror_items <<< "$mirrors_raw"
    for mirror in "${_mirror_items[@]}"; do
        mirror="${mirror#${mirror%%[![:space:]]*}}"
        mirror="${mirror%${mirror##*[![:space:]]}}"
        [[ -z "$mirror" ]] && continue
        mirror="${mirror%/}"
        candidates+=("${mirror}/${image_path}")
    done
    candidates+=("$upstream")

    for candidate in "${candidates[@]}"; do
        echo "[docker-base] pulling base candidate: $candidate"
        if docker pull "$candidate"; then
            docker tag "$candidate" "$dest"
            return 0
        fi
        echo "[docker-base] warning: failed to pull $candidate" >&2
    done

    echo "[docker-base] error: failed to prepare local base image $dest from $upstream" >&2
    echo "[docker-base]        set ROBONIX_DOCKER_MIRROR, ROBONIX_DOCKER_MIRRORS, or an explicit package base-image env" >&2
    return 1
}
