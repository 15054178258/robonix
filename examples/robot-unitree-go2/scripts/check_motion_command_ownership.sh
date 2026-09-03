#!/usr/bin/env bash
set -euo pipefail

# Read-only graph gate for the dedicated first-motion commissioning path.
# This script never publishes, calls a service/action, or changes networking.

readonly topic=/go2/commissioning/cmd_vel
readonly canonical_topic=/cmd_vel
readonly expected_type=geometry_msgs/msg/Twist
readonly expected_publisher=go2_first_motion_probe
readonly expected_subscriber=go2_chassis_adapter
readonly discovery_timeout_s="${GO2_MOTION_OWNERSHIP_TIMEOUT_S:-15}"
readonly stability_samples="${GO2_MOTION_OWNERSHIP_STABILITY_SAMPLES:-3}"
readonly query_timeout_s="${GO2_MOTION_OWNERSHIP_QUERY_TIMEOUT_S:-3}"

if [[ "$#" -ne 0 ]]; then
  echo "usage: $0" >&2
  exit 2
fi

for value_name in discovery_timeout_s stability_samples query_timeout_s; do
  value="${!value_name}"
  if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
    echo "${value_name} must be a positive integer" >&2
    exit 2
  fi
done
if (( discovery_timeout_s > 30 || stability_samples > 10 || query_timeout_s > 10 )); then
  echo "motion ownership timing exceeds its audited maximum" >&2
  exit 2
fi

for command in grep ros2 sed sleep timeout; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "missing read-only ownership command: $command" >&2
    exit 3
  }
done

query_graph() {
  local output status type publisher_count subscription_count
  local publisher_section subscription_section publisher_names subscriber_names
  local publisher_namespaces subscriber_namespaces publisher_gid subscriber_gid
  local publisher_reliable subscriber_reliable

  set +e
  output="$(
    LC_ALL=C timeout --signal=INT --kill-after=1s "${query_timeout_s}s" \
      ros2 topic info --no-daemon -v "$topic" 2>&1
  )"
  status=$?
  set -e
  if [[ "$status" -ne 0 ]]; then
    if grep -Fq "Unknown topic" <<<"$output"; then
      return 1
    fi
    echo "could not query dedicated motion topic: $output" >&2
    return 4
  fi

  type="$(sed -n 's/^Type:[[:space:]]*//p' <<<"$output")"
  publisher_count="$(
    sed -n 's/^Publisher count:[[:space:]]*\([0-9][0-9]*\)[[:space:]]*$/\1/p' \
      <<<"$output"
  )"
  subscription_count="$(
    sed -n 's/^Subscription count:[[:space:]]*\([0-9][0-9]*\)[[:space:]]*$/\1/p' \
      <<<"$output"
  )"
  if [[ "$type" != "$expected_type" || ! "$publisher_count" =~ ^[0-9]+$ \
      || ! "$subscription_count" =~ ^[0-9]+$ ]]; then
    echo "motion topic type/count output is ambiguous" >&2
    return 4
  fi
  if (( publisher_count != 1 || subscription_count != 1 )); then
    echo "motion ownership requires exactly one publisher and one subscription; found publishers=${publisher_count} subscriptions=${subscription_count}" >&2
    return 5
  fi

  publisher_section="$(
    sed -n '/^Publisher count:/,/^Subscription count:/p' <<<"$output"
  )"
  subscription_section="$(sed -n '/^Subscription count:/,$p' <<<"$output")"
  publisher_names="$(
    sed -n 's/^[[:space:]]*Node name:[[:space:]]*//p' <<<"$publisher_section"
  )"
  subscriber_names="$(
    sed -n 's/^[[:space:]]*Node name:[[:space:]]*//p' <<<"$subscription_section"
  )"
  publisher_namespaces="$(
    sed -n 's/^[[:space:]]*Node namespace:[[:space:]]*//p' <<<"$publisher_section"
  )"
  subscriber_namespaces="$(
    sed -n 's/^[[:space:]]*Node namespace:[[:space:]]*//p' <<<"$subscription_section"
  )"
  publisher_gid="$(
    sed -n 's/^[[:space:]]*GID:[[:space:]]*//p' <<<"$publisher_section"
  )"
  subscriber_gid="$(
    sed -n 's/^[[:space:]]*GID:[[:space:]]*//p' <<<"$subscription_section"
  )"
  publisher_reliable="$(
    sed -n 's/^[[:space:]]*Reliability:[[:space:]]*//p' <<<"$publisher_section"
  )"
  subscriber_reliable="$(
    sed -n 's/^[[:space:]]*Reliability:[[:space:]]*//p' <<<"$subscription_section"
  )"

  if [[ "$publisher_names" != "$expected_publisher" \
      || "$subscriber_names" != "$expected_subscriber" \
      || "$publisher_namespaces" != / || "$subscriber_namespaces" != / \
      || -z "$publisher_gid" || -z "$subscriber_gid" \
      || "$publisher_reliable" != RELIABLE \
      || "$subscriber_reliable" != RELIABLE ]]; then
    echo "motion endpoint identity/QoS does not match the dedicated commissioning path" >&2
    return 5
  fi
  printf '%s|%s\n' "$publisher_gid" "$subscriber_gid"
}

canonical_publisher_count() {
  local output status count
  set +e
  output="$(
    LC_ALL=C timeout --signal=INT --kill-after=1s "${query_timeout_s}s" \
      ros2 topic info --no-daemon -v "$canonical_topic" 2>&1
  )"
  status=$?
  set -e
  if [[ "$status" -ne 0 ]]; then
    if grep -Fq "Unknown topic" <<<"$output"; then
      printf '0\n'
      return 0
    fi
    echo "could not query canonical motion topic: $output" >&2
    return 4
  fi
  count="$(
    sed -n 's/^Publisher count:[[:space:]]*\([0-9][0-9]*\)[[:space:]]*$/\1/p' \
      <<<"$output"
  )"
  if [[ ! "$count" =~ ^[0-9]+$ ]]; then
    echo "canonical motion publisher count is ambiguous" >&2
    return 4
  fi
  printf '%s\n' "$count"
}

echo "================================================================"
echo " READ-ONLY FIRST-MOTION COMMAND OWNERSHIP CHECK"
echo " topic: $topic"
echo " No command is published and no arm/motion API is called."
echo "================================================================"

deadline=$((SECONDS + discovery_timeout_s))
stable=0
previous_identity=""
while (( SECONDS <= deadline )); do
  set +e
  identity="$(query_graph)"
  status=$?
  set -e
  if [[ "$status" -ge 4 ]]; then
    exit "$status"
  fi
  set +e
  canonical_count="$(canonical_publisher_count)"
  canonical_status=$?
  set -e
  if [[ "$canonical_status" -ne 0 ]]; then
    exit "$canonical_status"
  fi
  if (( canonical_count != 0 )); then
    echo "canonical /cmd_vel still has ${canonical_count} publisher(s); Nav2 must be motion-isolated" >&2
    exit 5
  fi
  if [[ "$status" -eq 0 && "$identity" == "$previous_identity" ]]; then
    stable=$((stable + 1))
  elif [[ "$status" -eq 0 ]]; then
    previous_identity="$identity"
    stable=1
  else
    previous_identity=""
    stable=0
  fi
  if (( stable >= stability_samples )); then
    echo "PASS: one stable ${expected_publisher} publisher and one ${expected_subscriber} subscriber"
    exit 0
  fi
  sleep 1
done

echo "dedicated motion endpoint ownership did not become stable before deadline" >&2
exit 6
