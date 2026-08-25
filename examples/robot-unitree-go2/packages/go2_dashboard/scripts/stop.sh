#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="${ROOT_DIR}/rbnx-build/run/provider.pid"

if [[ ! -r "${PID_FILE}" ]]; then
  echo "[go2-dashboard] not running (no PID file)"
  exit 0
fi
read -r PID < "${PID_FILE}" || true
if [[ ! "${PID:-}" =~ ^[0-9]+$ ]]; then
  echo "[go2-dashboard] invalid PID file; refusing to signal any process" >&2
  exit 2
fi
if ! kill -0 "${PID}" 2>/dev/null; then
  rm -f -- "${PID_FILE}"
  echo "[go2-dashboard] stale PID file removed"
  exit 0
fi

COMMAND_LINE="$(tr '\0' ' ' < "/proc/${PID}/cmdline" 2>/dev/null || true)"
if [[ "${COMMAND_LINE}" != *"go2_dashboard.service"* ]]; then
  echo "[go2-dashboard] PID ${PID} does not belong to this provider; refusing" >&2
  exit 3
fi

kill -TERM "${PID}"
for _ in $(seq 1 50); do
  if ! kill -0 "${PID}" 2>/dev/null; then
    rm -f -- "${PID_FILE}"
    echo "[go2-dashboard] provider stopped; its lifecycle hook stopped its child"
    exit 0
  fi
  sleep 0.2
done
echo "[go2-dashboard] process did not stop within 10 seconds" >&2
exit 4
