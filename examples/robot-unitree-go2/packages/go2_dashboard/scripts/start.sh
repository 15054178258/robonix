#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${ROOT_DIR}/rbnx-build/venv"
PID_FILE="${ROOT_DIR}/rbnx-build/run/provider.pid"
ROS_SETUP_FILE="${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"

echo "================================================================"
echo "  GO2 TELEMETRY + OPTIONAL LIAISON VOICE SERVICE"
echo "  Provider id: go2_dashboard"
echo "  Telemetry is read-only; browser voice is disabled by default."
echo "  Voice delegates only to Liaison and NEVER authorizes robot motion."
echo "================================================================"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "[go2-dashboard] missing build environment; run: bash scripts/build.sh" >&2
  exit 2
fi
if [[ ! -r "${ROS_SETUP_FILE}" ]]; then
  echo "[go2-dashboard] ROS setup not found: ${ROS_SETUP_FILE}" >&2
  exit 2
fi
if [[ -r "${PID_FILE}" ]]; then
  read -r EXISTING_PID < "${PID_FILE}" || true
  if [[ "${EXISTING_PID:-}" =~ ^[0-9]+$ ]] && kill -0 "${EXISTING_PID}" 2>/dev/null; then
    echo "[go2-dashboard] already running as PID ${EXISTING_PID}" >&2
    exit 3
  fi
fi

set +u
source "${ROS_SETUP_FILE}"
if [[ -n "${GO2_ROS_OVERLAY_SETUP:-}" ]]; then
  source "${GO2_ROS_OVERLAY_SETUP}"
fi
set -u

mkdir -p "${ROOT_DIR}/rbnx-build/run"
CODEGEN_PATH="${ROOT_DIR}/rbnx-build/codegen/proto_gen:${ROOT_DIR}/rbnx-build/codegen/robonix_mcp_types"
if [[ -n "${ROBONIX_SOURCE_PATH:-}" && -d "${ROBONIX_SOURCE_PATH}/pylib/robonix-api/robonix_api" ]]; then
  ROBONIX_API_ROOT="${ROBONIX_SOURCE_PATH}/pylib/robonix-api"
else
  command -v rbnx >/dev/null 2>&1 || {
    echo "[go2-dashboard] rbnx is required to locate robonix-api" >&2
    exit 2
  }
  ROBONIX_API_ROOT="$(rbnx path robonix-api)"
fi
[[ -d "${ROBONIX_API_ROOT}/robonix_api" ]] || {
  echo "[go2-dashboard] invalid robonix-api path: ${ROBONIX_API_ROOT}" >&2
  exit 2
}
export PYTHONPATH="${ROOT_DIR}:${CODEGEN_PATH}:${ROBONIX_API_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
export GO2_DASHBOARD_PROVIDER_PID_FILE="${PID_FILE}"

exec "${VENV_DIR}/bin/python" -m go2_dashboard.service
