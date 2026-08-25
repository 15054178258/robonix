#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEPLOY_ROOT="$(CDPATH= cd -- "${ROOT}/../.." && pwd)"
BUILD_ROOT="${GO2_CAMERA_DAEMON_BUILD_ROOT:-${ROOT}/.build/camera-daemon}"
INSTALL_ROOT="${BUILD_ROOT}/install"
SDK2_ROOT="${UNITREE_SDK2_ROOT:-${UNITREE_SDK2_DIR:-${DEPLOY_ROOT}/third_party/unitree_sdk2}}"

if [[ ! -f "${SDK2_ROOT}/CMakeLists.txt" ]]; then
  echo "No official unitree_sdk2 checkout at: ${SDK2_ROOT}" >&2
  echo "Set UNITREE_SDK2_ROOT or UNITREE_SDK2_DIR explicitly." >&2
  exit 1
fi
if ! command -v cmake >/dev/null 2>&1; then
  echo "cmake is not available." >&2
  exit 1
fi

cmake \
  -S "${ROOT}/camera_daemon" \
  -B "${BUILD_ROOT}" \
  -DUNITREE_SDK2_ROOT="${SDK2_ROOT}" \
  -DBUILD_EXAMPLES=OFF \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_ROOT}" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "${BUILD_ROOT}" --parallel --target go2_camera_daemon
cmake --install "${BUILD_ROOT}" --component go2_camera_runtime

for artifact in \
  "${INSTALL_ROOT}/bin/go2_camera_daemon" \
  "${INSTALL_ROOT}/lib/libddsc.so" \
  "${INSTALL_ROOT}/lib/libddsc.so.0" \
  "${INSTALL_ROOT}/lib/libddscxx.so" \
  "${INSTALL_ROOT}/lib/libddscxx.so.0" \
  "${INSTALL_ROOT}/share/licenses/unitree_sdk2/LICENSE" \
  "${INSTALL_ROOT}/share/licenses/unitree_sdk2/cyclonedds/LICENSE" \
  "${INSTALL_ROOT}/share/licenses/unitree_sdk2/cyclonedds-cxx/LICENSE" \
  "${INSTALL_ROOT}/share/licenses/unitree_sdk2/iceoryx/LICENSE" \
  "${INSTALL_ROOT}/share/licenses/unitree_sdk2/rapidjson/LICENSE"
do
  if [[ ! -e "${artifact}" ]]; then
    echo "Camera daemon private runtime output is missing: ${artifact}" >&2
    exit 1
  fi
done
