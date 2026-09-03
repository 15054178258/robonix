#!/usr/bin/env bash
# Build the guarded G1 adapter, SDK daemon, and offline tests.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
BUILD_ROOT="$PKG/rbnx-build"
DEPLOY_ROOT="${ROBONIX_DEPLOY_DIR:-$(cd "$PKG/../.." && pwd)}"
SDK2_DIR="${UNITREE_SDK2_DIR:-$DEPLOY_ROOT/third_party/unitree_sdk2}"
# shellcheck disable=SC1091
source "$DEPLOY_ROOT/scripts/build_robonix_ros2_overlay.sh"

echo "[g1/build] generating canonical ROS 2 IDL overlay"
rbnx codegen -p "$PKG" --ros2

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[g1/build] /opt/ros/humble/setup.bash is required" >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
robonix_build_ros2_overlay "$BUILD_ROOT/codegen/ros2_idl" \
  --packages-select lifecycle \
  --merge-install
if [ ! -f "$BUILD_ROOT/codegen/ros2_idl/install/setup.bash" ]; then
  echo "[g1/build] generated ROS 2 overlay is incomplete" >&2
  exit 2
fi

set +u
source "$BUILD_ROOT/codegen/ros2_idl/install/setup.bash"
set -u

echo "[g1/build] building offline protocol tests"
cmake -S "$PKG" -B "$BUILD_ROOT/offline" \
  -DG1_CHASSIS_BUILD_OFFLINE_TESTS=ON
cmake --build "$BUILD_ROOT/offline" --parallel
ctest --test-dir "$BUILD_ROOT/offline" --output-on-failure

echo "[g1/build] building ROS-only adapter (no SDK2 linkage)"
colcon --log-base "$BUILD_ROOT/ros/log" build \
  --base-paths "$PKG/ros2_ws/src" \
  --build-base "$BUILD_ROOT/ros/build" \
  --install-base "$BUILD_ROOT/ros/install" \
  --merge-install
if [ ! -x "$BUILD_ROOT/ros/install/lib/g1_chassis_adapter/g1_chassis_adapter_node" ]; then
  echo "[g1/build] ROS adapter output is missing" >&2
  exit 2
fi

if [ ! -f "$SDK2_DIR/CMakeLists.txt" ]; then
  echo "[g1/build] UNITREE_SDK2_DIR is not an official SDK2 checkout" >&2
  exit 2
fi
echo "[g1/build] building isolated SDK-only daemon; Unitree examples are OFF"
cmake -S "$PKG/sdk_daemon" -B "$BUILD_ROOT/sdk" \
  -DUNITREE_SDK2_DIR="$SDK2_DIR" \
  -DBUILD_EXAMPLES=OFF \
  -DCMAKE_INSTALL_PREFIX="$BUILD_ROOT/sdk/install"
cmake --build "$BUILD_ROOT/sdk" --parallel --target g1_loco_daemon
cmake --install "$BUILD_ROOT/sdk"
for artifact in \
  "$BUILD_ROOT/sdk/install/bin/g1_loco_daemon" \
  "$BUILD_ROOT/sdk/install/lib/libddsc.so" \
  "$BUILD_ROOT/sdk/install/lib/libddsc.so.0" \
  "$BUILD_ROOT/sdk/install/lib/libddscxx.so" \
  "$BUILD_ROOT/sdk/install/lib/libddscxx.so.0" \
  "$BUILD_ROOT/sdk/install/share/licenses/unitree_sdk2/LICENSE" \
  "$BUILD_ROOT/sdk/install/share/licenses/unitree_sdk2/cyclonedds/LICENSE" \
  "$BUILD_ROOT/sdk/install/share/licenses/unitree_sdk2/cyclonedds-cxx/LICENSE" \
  "$BUILD_ROOT/sdk/install/share/licenses/unitree_sdk2/iceoryx/LICENSE" \
  "$BUILD_ROOT/sdk/install/share/licenses/unitree_sdk2/rapidjson/LICENSE"
do
  if [ ! -e "$artifact" ]; then
    echo "[g1/build] SDK daemon runtime output is missing: $artifact" >&2
    exit 2
  fi
done
