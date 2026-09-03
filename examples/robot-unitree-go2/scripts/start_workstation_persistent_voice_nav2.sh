#!/usr/bin/env bash
set -euo pipefail
umask 077

# Long-running full Robonix stack.  Startup remains DISARMED and dispatches no
# goal.  Physical motion still requires a separate, current operator action.

readonly ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly BASE_LAUNCHER="$ROOT/scripts/start_workstation_staged_nav2_corrected.sh"
readonly START_ACK=I_AM_ON_SITE_SITE_CLEAR_REMOTE_STOP_READY

die() {
  local status="$1"
  shift
  printf '%s\n' "$*" >&2
  exit "$status"
}

[[ "$#" -eq 0 ]] || die 2 "this launcher accepts no arguments"
[[ -f "$BASE_LAUNCHER" && ! -L "$BASE_LAUNCHER" ]] \
  || die 2 "persistent base launcher is missing or is a symlink"

# Motion-scope values must come from this exact invocation.  Preserve them
# across the optional general deployment .env load instead of letting a stale
# no-motion interface/map value replace the operator's current confirmation.
INHERITED_START_ACK="${GO2_PERSISTENT_NAV2_START_ACK:-}"
INHERITED_INTERFACE="${GO2_NETWORK_INTERFACE:-}"
INHERITED_MAP_ID="${GO2_PERSISTENT_NAV2_MAP_ID:-}"
INHERITED_MAP_GENERATION="${GO2_PERSISTENT_NAV2_MAP_GENERATION:-}"
INHERITED_ALLOWED_MODE="${GO2_PERSISTENT_NAV2_ALLOWED_MODE:-}"
INHERITED_ALLOWED_MARKER="${GO2_PERSISTENT_NAV2_ALLOWED_STATE_MARKER:-}"
if [[ -f "$ROOT/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT/.env"
  set +a
fi
export GO2_PERSISTENT_NAV2_START_ACK="$INHERITED_START_ACK"
export GO2_NETWORK_INTERFACE="$INHERITED_INTERFACE"
export GO2_PERSISTENT_NAV2_MAP_ID="$INHERITED_MAP_ID"
export GO2_PERSISTENT_NAV2_MAP_GENERATION="$INHERITED_MAP_GENERATION"
export GO2_PERSISTENT_NAV2_ALLOWED_MODE="$INHERITED_ALLOWED_MODE"
export GO2_PERSISTENT_NAV2_ALLOWED_STATE_MARKER="$INHERITED_ALLOWED_MARKER"

export MODELSCOPE_CACHE="${MODELSCOPE_CACHE:-$ROOT/.cache/modelscope}"
export GO2_FUNASR_MODEL_PATH="${GO2_FUNASR_MODEL_PATH:-$MODELSCOPE_CACHE/models/iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online}"
export SPEECH_BACKEND="${SPEECH_BACKEND:-local}"
export SEMANTIC_LANDMARKS_FILE="${SEMANTIC_LANDMARKS_FILE:-config/semantic_landmarks.yaml}"
export GO2_DASHBOARD_PORT="${GO2_DASHBOARD_PORT:-8092}"
export SEMANTIC_INTENT_PORT="${SEMANTIC_INTENT_PORT:-18080}"
export VLM_BASE_URL="http://127.0.0.1:${SEMANTIC_INTENT_PORT}/v1"
export VLM_API_KEY=local-no-secret
export VLM_MODEL=go2-semantic-router

[[ "${GO2_PERSISTENT_NAV2_START_ACK:-}" == "$START_ACK" ]] \
  || die 2 "GO2_PERSISTENT_NAV2_START_ACK must exactly equal $START_ACK"
[[ "${GO2_NETWORK_INTERFACE:-}" == wlx500ff54809b8 ]] \
  || die 2 "persistent wireless Nav2 must use wlx500ff54809b8"
[[ -n "${GO2_PERSISTENT_NAV2_MAP_ID:-}" \
  && "$GO2_PERSISTENT_NAV2_MAP_ID" =~ ^[A-Za-z0-9._-]+$ ]] \
  || die 2 "GO2_PERSISTENT_NAV2_MAP_ID must be an explicit safe map id"
[[ "${GO2_PERSISTENT_NAV2_MAP_GENERATION:-}" =~ ^[0-9]+$ ]] \
  || die 2 "GO2_PERSISTENT_NAV2_MAP_GENERATION must be an explicit uint64"
[[ "${GO2_PERSISTENT_NAV2_ALLOWED_MODE:-}" =~ ^[0-9]+$ \
  && "$GO2_PERSISTENT_NAV2_ALLOWED_MODE" -le 254 ]] \
  || die 2 "GO2_PERSISTENT_NAV2_ALLOWED_MODE must come from the current read-only audit"
[[ "${GO2_PERSISTENT_NAV2_ALLOWED_STATE_MARKER:-}" =~ ^[0-9]+$ \
  && "$GO2_PERSISTENT_NAV2_ALLOWED_STATE_MARKER" -le 4294967295 ]] \
  || die 2 "GO2_PERSISTENT_NAV2_ALLOWED_STATE_MARKER must come from the current read-only audit"

readonly MAP_DIR="$ROOT/rbnx-build/data/maps/$GO2_PERSISTENT_NAV2_MAP_ID"
[[ -d "$MAP_DIR" && -f "$MAP_DIR/rtabmap.db" \
  && -f "$MAP_DIR/generation" && ! -L "$MAP_DIR/generation" ]] \
  || die 2 "the exact saved map artifact or generation file is missing"
IFS= read -r disk_generation < "$MAP_DIR/generation" \
  || die 2 "the saved map generation is unreadable"
[[ "$disk_generation" == "$GO2_PERSISTENT_NAV2_MAP_GENERATION" ]] \
  || die 2 "requested map generation does not match the saved artifact"

for variable in SPEECH_BACKEND GO2_FUNASR_MODEL_PATH SEMANTIC_LANDMARKS_FILE; do
  [[ -n "${!variable:-}" ]] \
    || die 2 "$variable must be configured for the full voice flow"
done
[[ "$GO2_FUNASR_MODEL_PATH" == /* \
  && -s "$GO2_FUNASR_MODEL_PATH/model.pt" \
  && -s "$GO2_FUNASR_MODEL_PATH/config.yaml" ]] \
  || die 2 "the audited local FunASR model is incomplete"

export GO2_PERSISTENT_NAV2_MODE=true
export GO2_STAGED_NAV2_STANDARD_MODE=true
export GO2_STAGED_NAV2_RUN_ACK=I_APPROVE_GO2_STAGED_NAV2_MOTION
export GO2_STAGED_NAV2_ALLOWED_MODE="$GO2_PERSISTENT_NAV2_ALLOWED_MODE"
export GO2_STAGED_NAV2_ALLOWED_STATE_MARKER="$GO2_PERSISTENT_NAV2_ALLOWED_STATE_MARKER"
export GO2_NAV2_MAP_ID="$GO2_PERSISTENT_NAV2_MAP_ID"
export GO2_NAV2_MAP_GENERATION="$GO2_PERSISTENT_NAV2_MAP_GENERATION"
export GO2_DASHBOARD_BROWSER_VOICE_ENABLED=1
export GO2_D435I_PREVIEW_ENABLED=true

echo "Persistent full stack accepted: startup is DISARMED and sends no goal."
echo "Map identity: $GO2_PERSISTENT_NAV2_MAP_ID generation $GO2_PERSISTENT_NAV2_MAP_GENERATION"
exec bash "$BASE_LAUNCHER"
