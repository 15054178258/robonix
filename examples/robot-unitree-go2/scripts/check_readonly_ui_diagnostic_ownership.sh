#!/usr/bin/env bash
# Graph-only ownership gate for the raw-hardware diagnostic UI. Every ROS CLI
# query is bounded and uses --no-daemon. This script never creates a publisher,
# calls a service/action, or changes the network configuration.

set -euo pipefail

if [[ "$#" -ne 1 || ( "$1" != pre && "$1" != post ) ]]; then
  echo "usage: $0 <pre|post>" >&2
  exit 2
fi

readonly phase="$1"
readonly discovery_timeout_s="${GO2_DIAGNOSTIC_DISCOVERY_TIMEOUT_S:-20}"
readonly stability_samples="${GO2_DIAGNOSTIC_STABILITY_SAMPLES:-1}"
readonly query_timeout_s="${GO2_DIAGNOSTIC_QUERY_TIMEOUT_S:-4}"

for value_name in discovery_timeout_s stability_samples query_timeout_s; do
  value="${!value_name}"
  if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
    echo "${value_name} must be a positive integer" >&2
    exit 2
  fi
done
if (( discovery_timeout_s > 60 || stability_samples > 10 || query_timeout_s > 15 )); then
  echo "diagnostic ownership timing bounds exceed their audited maxima" >&2
  exit 2
fi

for command in grep ros2 sed timeout; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "missing read-only ownership command: $command" >&2
    exit 3
  }
done

# The first three topics are vendor-owned raw hardware streams. The only
# publishers created by this profile are the camera bridge's image, camera
# info, and diagnostic outputs. Everything used by mapping/navigation remains
# absent, so this profile cannot resemble a navigation-ready graph.
declare -a topics=(
  /frontvideostream
  /utlidar/cloud
  /utlidar/robot_odom
  /camera/color/image_raw
  /camera/color/camera_info
  /go2/sensors/status
  /scanner/cloud
  /scanner/imu
  /odom
  /tf
  /tf_static
  /map
  /navigate_to_pose/_action/status
)
declare -a expected_pre=(1 1 1 0 0 0 0 0 0 0 0 0 0)
declare -a expected_post=(1 1 1 1 1 1 0 0 0 0 0 0 0)
declare -a expected
if [[ "$phase" == pre ]]; then
  expected=("${expected_pre[@]}")
else
  expected=("${expected_post[@]}")
fi

publisher_count() {
  local topic="$1"
  local output status count
  set +e
  output="$(
    LC_ALL=C timeout --signal=INT --kill-after=1s "${query_timeout_s}s" \
      ros2 topic info --no-daemon -v "$topic" 2>&1
  )"
  status=$?
  set -e
  if [[ "$status" -ne 0 ]]; then
    if grep -Fq "Unknown topic" <<<"$output"; then
      printf '0\n'
      return 0
    fi
    echo "could not query publisher ownership for ${topic}: ${output}" >&2
    return 4
  fi
  count="$(
    sed -n 's/^[[:space:]]*Publisher count:[[:space:]]*\([0-9][0-9]*\)[[:space:]]*$/\1/p' \
      <<<"$output"
  )"
  if [[ ! "$count" =~ ^[0-9]+$ ]]; then
    echo "ros2 topic info returned no unambiguous publisher count for ${topic}" >&2
    return 4
  fi
  printf '%s\n' "$count"
}

echo "================================================================"
echo " READ-ONLY DIAGNOSTIC PUBLISHER OWNERSHIP ${phase^^}CHECK"
echo " NOT NAVIGATION READY: map, TF, Nav2 and standardized odom stay absent."
echo " No topic is published and no service, action or motion API is called."
echo "================================================================"

deadline=$((SECONDS + discovery_timeout_s))
stable=0
declare -a last_counts
while (( SECONDS <= deadline )); do
  mismatch=false
  for index in "${!topics[@]}"; do
    count="$(publisher_count "${topics[$index]}")"
    last_counts[$index]="$count"
    if (( count > expected[index] )); then
      echo "publisher ownership violation: ${topics[$index]} expected ${expected[$index]}, found ${count}" >&2
      exit 5
    fi
    if (( count < expected[index] )); then
      mismatch=true
    fi
  done
  if [[ "$mismatch" == false ]]; then
    stable=$((stable + 1))
    if (( stable >= stability_samples )); then
      for index in "${!topics[@]}"; do
        printf '  %s publishers=%s\n' "${topics[$index]}" "${last_counts[$index]}"
      done
      echo "READ-ONLY diagnostic publisher ownership ${phase}check passed"
      exit 0
    fi
  else
    stable=0
  fi
  sleep 1
done

for index in "${!topics[@]}"; do
  printf '  %s expected=%s observed=%s\n' \
    "${topics[$index]}" "${expected[$index]}" "${last_counts[$index]:-unknown}" >&2
done
echo "publisher ownership discovery deadline expired for read-only diagnostic profile" >&2
exit 6
