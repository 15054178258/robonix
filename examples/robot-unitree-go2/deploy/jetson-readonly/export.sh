#!/usr/bin/env bash
set -euo pipefail

readonly image="${JETSON_READONLY_IMAGE:-robonix-go2-jetson-readonly:local}"
if [[ "$#" -ne 1 || -z "$1" ]]; then
  echo "usage: $0 OUTPUT.tar" >&2
  exit 2
fi
if [[ ! "$image" =~ ^[a-zA-Z0-9][a-zA-Z0-9._/:@-]*$ ]]; then
  echo "invalid image reference" >&2
  exit 42
fi
if [[ "$(docker image inspect --format '{{.Architecture}}' "$image")" != "arm64" ]]; then
  echo "refusing to export a non-ARM64 image" >&2
  exit 43
fi

output="$(realpath -m -- "$1")"
case "$output" in
  *.tar) ;;
  *) echo "output must end in .tar" >&2; exit 2 ;;
esac
[[ ! -e "$output" ]] || { echo "refusing to overwrite: ${output}" >&2; exit 44; }
mkdir -p "$(dirname -- "$output")"
umask 077
temporary="${output}.partial.$$"
trap 'rm -f -- "$temporary"' EXIT
docker save --output "$temporary" "$image"
mv -- "$temporary" "$output"
trap - EXIT
sha256sum "$output" > "${output}.sha256"
echo "exported ${output}"
