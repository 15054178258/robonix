#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HOST="${SEMANTIC_INTENT_HOST:-127.0.0.1}"
PORT="${SEMANTIC_INTENT_PORT:-18080}"
LANDMARKS="${SEMANTIC_LANDMARKS_FILE:-config/semantic_landmarks.yaml}"
EXECUTION_MODE="${SEMANTIC_INTENT_EXECUTION_MODE:-preview}"

case "$EXECUTION_MODE" in
  preview|live)
    ;;
  *)
    echo "SEMANTIC_INTENT_EXECUTION_MODE must be preview or live" >&2
    exit 64
    ;;
esac

if [[ "$LANDMARKS" != /* ]]; then
  LANDMARKS="$ROOT/$LANDMARKS"
fi

export PYTHONPATH="$ROOT/packages/semantic_intent_router:$ROOT/packages/semantic_navigation${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m semantic_intent_router.server \
  --host "$HOST" \
  --port "$PORT" \
  --landmarks "$LANDMARKS" \
  --execution-mode "$EXECUTION_MODE"
