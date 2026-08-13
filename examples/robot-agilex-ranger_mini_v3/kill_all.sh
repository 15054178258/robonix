#!/usr/bin/env bash
set -euo pipefail

# Stop only the Robonix stack rooted at this deployment directory. Persistent
# map and scene data are intentionally left untouched.
DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="$DEPLOY_DIR/robonix_manifest.yaml"
CACHE_DIR="$DEPLOY_DIR/rbnx-boot/cache"
export PATH="$HOME/.cargo/bin:$PATH"

if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi

collect_pids() {
  local -a pids=()
  local pid port cmd

  while IFS= read -r pid; do
    [[ -n "$pid" ]] && pids+=("$pid")
  done < <(pgrep -f "$CACHE_DIR" || true)

  # The Piper packages currently live outside this deploy's cache directory.
  # Include them in the same graceful TERM -> KILL lifecycle instead of using
  # an unconditional kill -9 after the normal shutdown path.
  while IFS= read -r pid; do
    [[ -n "$pid" ]] && pids+=("$pid")
  done < <(pgrep -f "piper" || true)

  # These are the fixed control-plane ports of this deploy. A second stack
  # cannot legitimately own them at the same time, so this also recovers from
  # a missing rbnx-boot/state.json after an interrupted boot.
  for port in 50051 50061 50071 50081 50091 7447; do
    while IFS= read -r pid; do
      [[ -n "$pid" ]] || continue
      cmd="$(ps -p "$pid" -o command= 2>/dev/null || true)"
      if [[ "$cmd" =~ (robonix-|rmw_zenohd) ]]; then
        pids+=("$pid")
      fi
    done < <(lsof -t -nP -iTCP:"$port" -sTCP:LISTEN 2>/dev/null || true)
  done

  printf '%s\n' "${pids[@]:-}" | awk 'NF && !seen[$0]++'
}

stop_pids() {
  local signal="$1"
  shift
  local pid
  for pid in "$@"; do
    kill -0 "$pid" 2>/dev/null || continue
    echo "[$signal] pid=$pid $(ps -p "$pid" -o comm= 2>/dev/null || true)"
    kill "-$signal" "$pid" 2>/dev/null || true
  done
}

echo "[kill_all] deployment: $DEPLOY_DIR"
if command -v rbnx >/dev/null 2>&1; then
  rbnx shutdown -f "$MANIFEST" || true
fi

mapfile -t pids < <(collect_pids)
if ((${#pids[@]} == 0)); then
  echo "[kill_all] no matching processes remain"
  exit 0
fi

stop_pids TERM "${pids[@]}"
sleep 8

mapfile -t pids < <(collect_pids)
if ((${#pids[@]} > 0)); then
  echo "[kill_all] escalating remaining deployment processes"
  stop_pids KILL "${pids[@]}"
fi

echo "[kill_all] done"
