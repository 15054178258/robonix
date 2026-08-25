#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TEMP_DIR="$(mktemp -d)"
trap 'rm -rf -- "${TEMP_DIR}"' EXIT

PYTHONDONTWRITEBYTECODE=1 python3 "${ROOT}/tests/static_safety_test.py"
PYTHONDONTWRITEBYTECODE=1 python3 "${ROOT}/tests/provider_contract_test.py"

# Configure only: this validates the standalone CMake graph and the forced
# example-off gate without compiling or running any Unitree code.
DEPLOY_ROOT="$(CDPATH= cd -- "${ROOT}/../.." && pwd)"
SDK2_ROOT="${UNITREE_SDK2_ROOT:-${UNITREE_SDK2_DIR:-${DEPLOY_ROOT}/third_party/unitree_sdk2}}"
if command -v cmake >/dev/null 2>&1; then
  if [[ ! -f "${SDK2_ROOT}/CMakeLists.txt" ]]; then
    echo "FAILED: official unitree_sdk2 checkout is missing: ${SDK2_ROOT}" >&2
    exit 1
  fi
  cmake \
    -S "${ROOT}/camera_daemon" \
    -B "${TEMP_DIR}/camera-cmake" \
    -DUNITREE_SDK2_ROOT="${SDK2_ROOT}" \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_INSTALL_PREFIX="${TEMP_DIR}/camera-install" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
  if ! grep -q '^BUILD_EXAMPLES:BOOL=OFF$' "${TEMP_DIR}/camera-cmake/CMakeCache.txt"; then
    echo "FAILED: camera daemon CMake did not force Unitree examples off" >&2
    exit 1
  fi
  echo "camera daemon CMake configure test passed (examples OFF; no build)"
else
  echo "camera daemon CMake configure skipped: cmake is not installed"
fi

g++ \
  -std=c++17 \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -pthread \
  -I"${ROOT}/include" \
  "${ROOT}/tests/test_camera_ipc.cpp" \
  -o "${TEMP_DIR}/test_camera_ipc"
"${TEMP_DIR}/test_camera_ipc"

g++ \
  -std=c++17 \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -pthread \
  -I"${ROOT}/include" \
  "${ROOT}/tests/test_latest_frame_mailbox.cpp" \
  -o "${TEMP_DIR}/test_latest_frame_mailbox"
"${TEMP_DIR}/test_latest_frame_mailbox"

g++ \
  -std=c++17 \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I"${ROOT}/include" \
  "${ROOT}/tests/test_camera_error_watermark.cpp" \
  -o "${TEMP_DIR}/test_camera_error_watermark"
"${TEMP_DIR}/test_camera_error_watermark"

g++ \
  -std=c++17 \
  -DGO2_SENSORS_STRICT_JPEG_TESTING \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I"${ROOT}/include" \
  "${ROOT}/src/strict_jpeg_decoder.cpp" \
  "${ROOT}/tests/test_strict_jpeg_decoder.cpp" \
  -ljpeg \
  -o "${TEMP_DIR}/test_strict_jpeg_decoder"
"${TEMP_DIR}/test_strict_jpeg_decoder"

g++ \
  -std=c++17 \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I"${ROOT}/include" \
  "${ROOT}/tests/test_stamp_guard.cpp" \
  -o "${TEMP_DIR}/test_stamp_guard"
"${TEMP_DIR}/test_stamp_guard"
