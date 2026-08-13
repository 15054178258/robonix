#!/usr/bin/env bash
set -euo pipefail
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_zenoh_cpp" && -n "${ROBONIX_ZENOH_ROUTER:-}" ]]; then
    src="/opt/ros/${ROS_DISTRO:-humble}/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"
    dst="/tmp/robonix_zenoh_session.json5"
    cp "$src" "$dst"
    sed -i \
        -e "s#\"mode\": \"peer\"#\"mode\": \"${ROBONIX_ZENOH_MODE:-client}\"#" \
        -e "s#\"tcp/localhost:7447\"#\"${ROBONIX_ZENOH_ROUTER}\"#g" \
        "$dst"
    if [[ -n "${ROBONIX_ZENOH_LISTEN:-}" ]]; then
        sed -i "s#\"tcp/localhost:0\"#\"${ROBONIX_ZENOH_LISTEN}\"#g" "$dst"
    fi
    export ZENOH_SESSION_CONFIG_URI="$dst"
fi

exec "$@"

