#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/rbnx-build"
VENV_DIR="${BUILD_DIR}/venv"
RBNX_BIN="${RBNX_BIN:-rbnx}"

echo "[go2-dashboard] building read-only telemetry + optional Liaison voice UI"
PYTHONPATH="${ROOT_DIR}${PYTHONPATH:+:${PYTHONPATH}}" \
  PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s "${ROOT_DIR}/tests" -v
"${RBNX_BIN}" codegen -p "${ROOT_DIR}" --mcp
python3 -m venv --system-site-packages "${VENV_DIR}"

if [[ "${GO2_DASHBOARD_SKIP_PIP:-0}" != "1" ]]; then
  "${VENV_DIR}/bin/python" -m pip install \
    --disable-pip-version-check \
    --requirement "${ROOT_DIR}/requirements.txt"
fi

if [[ -n "${ROBONIX_SOURCE_PATH:-}" && -d "${ROBONIX_SOURCE_PATH}/pylib/robonix-api/robonix_api" ]]; then
  ROBONIX_API_ROOT="${ROBONIX_SOURCE_PATH}/pylib/robonix-api"
else
  ROBONIX_API_ROOT="$("${RBNX_BIN}" path robonix-api)"
fi
[[ -d "${ROBONIX_API_ROOT}/robonix_api" ]] || {
  echo "[go2-dashboard] invalid robonix-api path: ${ROBONIX_API_ROOT}" >&2
  exit 2
}
CODEGEN_PATH="${ROOT_DIR}/rbnx-build/codegen/proto_gen:${ROOT_DIR}/rbnx-build/codegen/robonix_mcp_types"

PYTHONPATH="${ROOT_DIR}:${CODEGEN_PATH}:${ROBONIX_API_ROOT}${PYTHONPATH:+:${PYTHONPATH}}" \
  "${VENV_DIR}/bin/python" -m compileall -q "${ROOT_DIR}/go2_dashboard"

PYTHONPATH="${ROOT_DIR}:${CODEGEN_PATH}:${ROBONIX_API_ROOT}${PYTHONPATH:+:${PYTHONPATH}}" \
  "${VENV_DIR}/bin/python" -c \
  'import fastapi, PIL, uvicorn; import robonix_api, go2_dashboard_pb2, go2_dashboard_mcp; import go2_dashboard.service; print("[go2-dashboard] provider dependencies OK")'

# The first unittest pass intentionally runs before PyPI dependencies exist.
# Re-run the HTTP boundary suite inside the package venv so FastAPI exercises
# disabled, loopback, nonce, MIME, Content-Length, and byte-limit behavior.
PYTHONPATH="${ROOT_DIR}:${CODEGEN_PATH}:${ROBONIX_API_ROOT}${PYTHONPATH:+:${PYTHONPATH}}" \
  PYTHONDONTWRITEBYTECODE=1 \
  "${VENV_DIR}/bin/python" -m unittest discover \
    -s "${ROOT_DIR}/tests" -p 'test_web_voice.py' -v

echo "[go2-dashboard] build complete"
