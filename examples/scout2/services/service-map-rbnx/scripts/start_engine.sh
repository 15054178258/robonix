#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# In-container SLAM engine launcher. Picks the algo from MAPPING_ALGO
# (set by atlas_bridge after parsing config) and execs the
# corresponding ros2 launch.
#
# Supported algos:
#   rtabmap   — Sensor-agnostic graph SLAM. Subscribes to whatever
#               sensors.* the deploy enabled (lidar2d / lidar3d / rgb /
#               depth / odom). Webots tiago = lidar2d + rgb + depth; real
#               robot (mid360) = lidar3d + rgb + depth.
#   dlio      — Direct LiDAR-Inertial Odometry (3D Livox; real robot)
#   fastlio2  — [BROKEN: drift] kept for repro/debug only
#
# Pre-conditions: atlas_bridge has already written
# /tmp/<algo>_resolved.yaml with discovered topic names.
set -eo pipefail

ALGO="${MAPPING_ALGO:-rtabmap}"
RESOLVED="/tmp/${ALGO}_resolved.yaml"

source /opt/ros/humble/setup.bash

read_y() {
    # Never fail under `set -e`: grep returns 1 on no match, which
    # would otherwise abort the script before we even get to the
    # case branch. Default-empty is what every caller wants anyway.
    { grep -E "^$1:" "$RESOLVED" 2>/dev/null || true; } | head -1 | awk '{print $2}' || true
}

case "$ALGO" in
    rtabmap)
        # Sensor inputs are deploy-driven. Every topic comes from
        # /tmp/<algo>_resolved.yaml, written by atlas_bridge after it
        # resolved the contracts enabled by `sensors.*` in the deploy
        # manifest. Keys (see atlas_bridge:_SENSOR_CONTRACTS):
        #   scan_topic       ← robonix/primitive/lidar/lidar    (lidar2d)
        #   lidar_topic      ← robonix/primitive/lidar/lidar3d  (lidar3d)
        #   odom_topic       ← robonix/primitive/chassis/odom
        #   rgb_topic        ← robonix/primitive/camera/rgb
        #   depth_topic      ← robonix/primitive/camera/depth
        # Webots tiago = lidar2d + rgb + depth + odom.
        # Real robot   = lidar3d + rgb + depth + odom (+ imu, unused by rtabmap).
        # Anything not present in resolved.yaml passes through as the
        # `<none>` sentinel and the launch file disables that subscription.
        SCAN_TOPIC=$(read_y scan_topic)
        SCAN_CLOUD_TOPIC=$(read_y lidar_topic)
        ODOM_TOPIC=$(read_y odom_topic)
        RGB_TOPIC=$(read_y rgb_topic)
        DEPTH_TOPIC=$(read_y depth_topic)
        IMU_TOPIC=$(read_y imu_topic)
        DESKEW_LIDAR=$(read_y deskew_lidar)
        SCAN_TOPIC="${SCAN_TOPIC:-<none>}"
        SCAN_CLOUD_TOPIC="${SCAN_CLOUD_TOPIC:-<none>}"
        ODOM_TOPIC="${ODOM_TOPIC:-<none>}"
        RGB_TOPIC="${RGB_TOPIC:-<none>}"
        DEPTH_TOPIC="${DEPTH_TOPIC:-<none>}"
        IMU_TOPIC="${IMU_TOPIC:-<none>}"
        DESKEW_LIDAR="${DESKEW_LIDAR:-false}"
        # tf + time-source from atlas_bridge's resolved.yaml (cfg-driven
        # in the deploy manifest); fall back to legacy env / defaults.
        # Real-robot bring-ups without a chassis TF override base_frame
        # to the lidar's own frame so rtabmap doesn't block on base_link.
        BASE_FRAME=$(read_y base_frame)
        ODOM_FRAME=$(read_y odom_frame)
        NAV_ODOM_BRIDGE=$(read_y navigation_odom_bridge)
        NAV_ODOM_TOPIC=$(read_y navigation_odom_topic)
        NAV_ODOM_FRAME=$(read_y navigation_odom_frame)
        USE_SIM_TIME_R=$(read_y use_sim_time)
        BASE_FRAME="${BASE_FRAME:-base_link}"
        ODOM_FRAME="${ODOM_FRAME:-odom}"
        NAV_ODOM_BRIDGE="${NAV_ODOM_BRIDGE:-false}"
        NAV_ODOM_TOPIC="${NAV_ODOM_TOPIC:-/odom}"
        NAV_ODOM_FRAME="${NAV_ODOM_FRAME:-odom}"
        USE_SIM_TIME="${USE_SIM_TIME_R:-${MAPPING_USE_SIM_TIME:-true}}"
        ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-false}"
        # Map persistence (atlas_bridge wrote these from the deploy's
        # map_id / map_mode config). Empty database_path = ephemeral.
        DATABASE_PATH=$(read_y database_path)
        MAP_MODE=$(read_y map_mode)
        RESET_MAP=$(read_y reset_map)
        RTABMAP_OVERRIDES_FILE=$(read_y rtabmap_overrides_file)
        MAP_MODE="${MAP_MODE:-mapping}"
        RESET_MAP="${RESET_MAP:-false}"
        echo "[start_engine] rtabmap persistence: db=${DATABASE_PATH:-<ephemeral>} mode=$MAP_MODE reset=$RESET_MAP"
        # Only pass database_path when it is set. ros2 launch rejects an
        # empty `database_path:=` ("malformed launch argument"), so the
        # ephemeral case (no map_id) must omit the argument entirely and
        # let the launch file's default ("" → temp db) take over.
        DB_ARG=()
        [ -n "$DATABASE_PATH" ] && DB_ARG=(database_path:="$DATABASE_PATH")
        OVERRIDES_ARG=()
        [ -n "$RTABMAP_OVERRIDES_FILE" ] && OVERRIDES_ARG=(rtabmap_overrides_file:="$RTABMAP_OVERRIDES_FILE")
        echo "[start_engine] rtabmap scan2d=$SCAN_TOPIC scan3d=$SCAN_CLOUD_TOPIC odom=$ODOM_TOPIC rgb=$RGB_TOPIC depth=$DEPTH_TOPIC imu=$IMU_TOPIC deskew=$DESKEW_LIDAR base=$BASE_FRAME odomf=$ODOM_FRAME use_sim_time=$USE_SIM_TIME viz=$ENABLE_VIZ"
        # Run launch in the background so a sidecar can scrape
        # rtabmap_slam's actual --params-file path (the temp file ros2
        # launch generated, e.g. /tmp/launch_params_xxxxxx) plus its
        # contents and log them into mapping.log. Without this the only
        # way to find that file is `ps -ef | grep rtabmap`, which is
        # hostile to debugging parameter resolution.
        ros2 launch "${MAPPING_LAUNCH_DIR:-/mapping/launch}/rtabmap_2d.launch.py" \
            scan_topic:="$SCAN_TOPIC" \
            scan_cloud_topic:="$SCAN_CLOUD_TOPIC" \
            odom_topic:="$ODOM_TOPIC" \
            rgb_topic:="$RGB_TOPIC" \
            depth_topic:="$DEPTH_TOPIC" \
            imu_topic:="$IMU_TOPIC" \
            deskew_lidar:="$DESKEW_LIDAR" \
            base_frame:="$BASE_FRAME" \
            odom_frame:="$ODOM_FRAME" \
            navigation_odom_bridge:="$NAV_ODOM_BRIDGE" \
            navigation_odom_topic:="$NAV_ODOM_TOPIC" \
            navigation_odom_frame:="$NAV_ODOM_FRAME" \
            use_sim_time:="$USE_SIM_TIME" \
            enable_viz:="$ENABLE_VIZ" \
            map_mode:="$MAP_MODE" \
            reset_map:="$RESET_MAP" \
            "${OVERRIDES_ARG[@]}" \
            "${DB_ARG[@]}" &
        LAUNCH_PID=$!
        (
            python3 - <<'PYEOF'
import time, subprocess
deadline = time.time() + 10
seen = set()
while time.time() < deadline:
    try:
        out = subprocess.check_output(["pgrep", "-af", "rtabmap_slam/rtabmap"], text=True)
    except subprocess.CalledProcessError:
        time.sleep(0.4); continue
    for ln in out.splitlines():
        if not ln:
            continue
        pid = ln.split()[0]
        if pid in seen:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                argv = f.read().split(b"\x00")
        except FileNotFoundError:
            continue
        seen.add(pid)
        params = None
        for i, a in enumerate(argv):
            if a == b"--params-file" and i + 1 < len(argv):
                params = argv[i + 1].decode("utf-8", "replace")
                break
        if params:
            print(f"[start_engine] >>> rtabmap pid={pid} params-file={params}", flush=True)
            print(f"[start_engine] >>> ----- begin {params} -----", flush=True)
            try:
                with open(params) as f:
                    for line in f.read().splitlines():
                        print(f"[start_engine]    {line}", flush=True)
            except OSError as e:
                print(f"[start_engine] >>> ERR reading {params}: {e}", flush=True)
            print(f"[start_engine] >>> ----- end {params} -----", flush=True)
        else:
            print(f"[start_engine] >>> rtabmap pid={pid} has no --params-file in argv", flush=True)
    if seen:
        break
    time.sleep(0.4)
else:
    print("[start_engine] >>> WARN: no rtabmap_slam process found within 10s", flush=True)
PYEOF
        ) &
        wait $LAUNCH_PID
        ;;

    dlio)
        # Direct LiDAR-Inertial Odometry — real-robot 3D livox path.
        # Requires the `dlio` ros2 package mounted/installed in the
        # image; for desktop deploys bind-mount the slam_alt_ws colcon
        # build at /ws/install.
        if [ -f /ws/install/setup.bash ]; then
            source /ws/install/setup.bash
        else
            echo "[start_engine] ERR: dlio needs /ws/install (slam_alt_ws colcon build)" >&2
            exit 2
        fi
        echo "[start_engine] dlio (3D Livox + IMU)"
        exec ros2 launch dlio dlio.launch.py
        ;;

    fastlio2)
        # [BROKEN: drift] kept only so the repro path stays reachable.
        # Image must include /ws/install built from FASTLIO2_ROS2
        # (the heavier Dockerfile.fastlio2_full path). Do NOT pick
        # fastlio2 for any production deploy until the global drift
        # issue is fixed.
        if [ -f /ws/install/setup.bash ]; then
            source /ws/install/setup.bash
        fi
        echo "[start_engine] WARN fastlio2 has known drift; use for repro only"
        exec ros2 launch /mapping/launch/slam_mapping_native.launch.py
        ;;

    *)
        echo "[start_engine] unknown algo: $ALGO (supported: rtabmap | dlio | fastlio2)" >&2
        exit 2
        ;;
esac
