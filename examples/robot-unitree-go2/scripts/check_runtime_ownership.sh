#!/usr/bin/env bash
set -euo pipefail

# Do not invoke Humble's high-level topic-info CLI here: it constructs a
# high-level rclpy Node and therefore an implicit /parameter_events publisher.
# The Python checker uses only a low-level graph node with zero communication
# endpoints and applies the same bounded exact-count policy.

readonly script_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${script_dir}/check_runtime_ownership.py" "$@"
