#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

docker rm -f "${ROBONIX_EXPLORE_CONTAINER:-robonix_explore}" >/dev/null 2>&1 || true
