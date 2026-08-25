#!/usr/bin/env bash
set -euo pipefail

readonly overlay="/opt/robonix/overlay"
readonly camera="/opt/robonix/camera"
readonly profile="/opt/robonix/profile"

for package in unitree_api unitree_go go2_chassis_adapter go2_sensors go2_description; do
  [[ -d "${overlay}/share/${package}" ]] || {
    echo "required package missing from runtime: ${package}" >&2
    exit 30
  }
done

installed_packages="$({
  find "${overlay}/share/ament_index/resource_index/packages" \
    -mindepth 1 -maxdepth 1 -type f -printf '%f\n'
} | LC_ALL=C sort)"
expected_packages="$(printf '%s\n' \
  go2_chassis_adapter \
  go2_description \
  go2_sensors \
  unitree_api \
  unitree_go)"
if [[ "$installed_packages" != "$expected_packages" ]]; then
  echo "overlay package allowlist mismatch" >&2
  printf 'found:\n%s\n' "$installed_packages" >&2
  exit 30
fi

for forbidden_package in \
  nav2_bringup \
  navigation2 \
  slam_toolbox \
  rtabmap_ros \
  mapping \
  speech \
  scene \
  semantic_navigation; do
  [[ ! -e "${overlay}/share/${forbidden_package}" ]] || {
    echo "forbidden package present: ${forbidden_package}" >&2
    exit 31
  }
done

readonly adapter="${overlay}/lib/go2_chassis_adapter/go2_chassis_adapter_node"
readonly relay="${overlay}/lib/go2_sensors/go2_sensor_relay"
readonly bridge="${overlay}/lib/go2_sensors/go2_camera_bridge"
readonly camera_daemon="${camera}/bin/go2_camera_daemon"
for executable in "$adapter" "$relay" "$bridge" "$camera_daemon"; do
  [[ -x "$executable" ]] || {
    echo "required read-only executable missing: ${executable}" >&2
    exit 32
  }
done

overlay_executables="$({
  find "${overlay}/lib/go2_chassis_adapter" "${overlay}/lib/go2_sensors" \
    -mindepth 1 -maxdepth 1 -type f -perm /0111 -printf '%f\n'
} | LC_ALL=C sort)"
expected_overlay_executables="$(printf '%s\n' \
  go2_camera_bridge \
  go2_chassis_adapter_node \
  go2_sensor_relay)"
if [[ "$overlay_executables" != "$expected_overlay_executables" ]]; then
  echo "ROS executable allowlist mismatch" >&2
  printf 'found:\n%s\n' "$overlay_executables" >&2
  exit 32
fi
camera_executables="$(find "${camera}/bin" -mindepth 1 -maxdepth 1 \
  -type f -perm /0111 -printf '%f\n' | LC_ALL=C sort)"
if [[ "$camera_executables" != "go2_camera_daemon" ]]; then
  echo "camera executable allowlist mismatch" >&2
  printf 'found:\n%s\n' "$camera_executables" >&2
  exit 32
fi
camera_dependencies="$(LD_LIBRARY_PATH="${camera}/lib" ldd "$camera_daemon")"
if grep -Eiq '(not found|libcuda|libnvidia)' <<< "$camera_dependencies"; then
  echo "camera runtime has missing or GPU dependencies" >&2
  printf '%s\n' "$camera_dependencies" >&2
  exit 32
fi
# ldd does not read ament environment hooks. Load the same ROS/overlay search
# path as entrypoint before checking the ROS bridge, otherwise normal Humble
# libraries are incorrectly reported as missing during the image build.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${overlay}/setup.bash"
set -u
bridge_dependencies="$(ldd "$bridge")"
if grep -Eiq '(not found|libcuda|libnvidia)' <<< "$bridge_dependencies"; then
  echo "camera bridge has missing or GPU dependencies" >&2
  printf '%s\n' "$bridge_dependencies" >&2
  exit 32
fi
if ! grep -Eq 'libjpeg\.so\.[0-9]+[[:space:]]*=>' <<< "$bridge_dependencies"; then
  echo "camera bridge does not resolve an explicit JPEG runtime" >&2
  printf '%s\n' "$bridge_dependencies" >&2
  exit 32
fi

for forbidden_executable in \
  "${overlay}/lib/go2_chassis_adapter/go2_sport_daemon" \
  "${overlay}/bin/go2_sport_daemon" \
  "${camera}/bin/go2_sport_daemon" \
  "${overlay}/lib/go2_chassis_adapter/go2_chassis_sdk_daemon" \
  "${overlay}/bin/go2_chassis_sdk_daemon" \
  "${camera}/bin/go2_chassis_sdk_daemon" \
  "${camera}/bin/go2_sport_client" \
  "${camera}/bin/go2_stand_example" \
  "${camera}/bin/low_level_ctrl"; do
  [[ ! -e "$forbidden_executable" ]] || {
    echo "forbidden control executable present: ${forbidden_executable}" >&2
    exit 33
  }
done

grep -Eq '^[[:space:]]*allow_motion:[[:space:]]*false[[:space:]]*$' \
  "${profile}/config/chassis-passive.yaml"
if grep -E \
  '(/api/sport/request|(^|/)lowcmd($|[^[:alnum:]_])|ros2[[:space:]]+topic[[:space:]]+pub)' \
  "${profile}/entrypoint.sh" \
  "${profile}/healthcheck.sh" \
  "${profile}/camera-quality-healthcheck.py" \
  "${profile}/check-runtime-ownership.sh" \
  "${profile}/validate-network.sh" \
  "${profile}/config/chassis-passive.yaml" \
  "${profile}/config/sensors-readonly.yaml"; then
  echo "forbidden publisher or motion endpoint found in runtime profile" >&2
  exit 34
fi
