#!/usr/bin/env bash
set -euo pipefail

# Robonix owns the provider process group and sends its normal lifecycle
# shutdown. There is deliberately no broad pkill fallback.
echo "semantic_navigation stop is lifecycle-owned by Robonix"
