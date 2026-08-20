# SPDX-License-Identifier: Apache-2.0
# nav2_wrapper_rbnx — Nav2 navigation service image.
#
# Why a container: nav2_wrapper drives a system-installed Nav2 stack, but
# many hosts (x86 dev boxes, the webots CI machine) have no host ROS2 at
# all. This image bundles ROS2 Humble + nav2_bringup so the wrapper runs
# anywhere docker does — the same portability story as mapping_rbnx. The
# native (host-ROS2) path stays available via scripts/start_native.sh for
# robots whose host already runs ROS2.
#
# --network host + --ipc=host at runtime so DDS/SHM line up with whatever
# sim/robot container publishes /map, /odom, /scan and consumes /cmd_vel.
ARG ROS_BASE_IMAGE=robonix-ros:humble-ros-base
FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# ── TUNA mirrors ───────────────────────────────────────────────────
RUN set -eux; \
    find /etc/apt/sources.list.d/ -maxdepth 1 \( -name '*.list' -o -name '*.sources' \) -print0 \
      | xargs -0 -r sed -i --follow-symlinks \
          -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
          -e 's|^Types: deb deb-src$|Types: deb|'; \
    if [ -f /etc/apt/sources.list ]; then \
        sed -i --follow-symlinks \
            -e 's|http://archive\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://security\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://ports\.ubuntu\.com/ubuntu-ports/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/|g' \
            /etc/apt/sources.list; \
    fi
ENV PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple

# Nav2 stack + tf + the pointcloud→laserscan adapter (for 3D-lidar-only
# robots whose costmap wants a LaserScan).
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-yaml \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-tf2-ros \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-rtabmap-util \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rmw-zenoh-cpp \
 && rm -rf /var/lib/apt/lists/*

# grpcio-tools 1.83+ needs setuptools>=77, while ROS colcon-core requires <80.
# Upgrade pip first to avoid dependency resolution bugs.
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir 'setuptools>=77,<80' && \
    pip install --no-cache-dir grpcio>=1.60.0 grpcio-tools>=1.60.0 && \
    pip install --no-cache-dir protobuf>=4.25.0 pyyaml>=6.0 && \
    pip install --no-cache-dir 'mcp>=1.28.1,<2' fastmcp>=3

# ── Build robonix_nav2_terminal plugins (PersistentRotateToGoal, etc.) ──
COPY terminal_controller /tmp/terminal_controller
RUN bash -c 'set -ex; \
    source /opt/ros/humble/setup.bash; \
    cd /tmp/terminal_controller; \
    colcon build --packages-select robonix_nav2_terminal --install-base /opt/robonix_nav2_terminal; \
    rm -rf /tmp/terminal_controller'

WORKDIR /nav2
COPY entrypoint.sh /entrypoint.sh
COPY no_shm_profile.xml /etc/fastrtps_no_shm.xml
RUN chmod +x /entrypoint.sh

# Left for operators who explicitly override RMW back to FastRTPS.
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastrtps_no_shm.xml \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp

ENTRYPOINT ["/entrypoint.sh"]
