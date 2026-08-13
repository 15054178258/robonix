# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx — RTAB-Map SLAM service image.
#
# Same packaging pattern as system/scene: own image so the package can
# pick its RMW + SLAM toolchain without polluting the sim image.
# --network host + --ipc=host at runtime so DDS/SHM line up with
# whatever sim/robot container is publishing the sensor topics.
#
# Algorithms shipped (selected via manifest config.algo):
#   rtabmap   — default. 2D lidar + RGBD fusion. Fully apt-installable
#               (ros-humble-rtabmap-ros), no extra colcon build.
#   dlio      — Direct LiDAR-Inertial Odometry. Real-robot 3D Livox
#               path; expects the dlio package mounted at /ws/install
#               (separate slam_alt_ws colcon build), so this image
#               stays light.
#   fastlio2  — [BROKEN: drift] same /ws/install path as dlio.
#
# slam_toolbox + cartographer were dropped after rtabmap consistently
# beat them in head-to-head tests on webots tiago (curve-motion
# stability, RGBD fusion, loop closure).
ARG ROS_BASE_IMAGE=robonix-ros:humble-ros-base
FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# ── TUNA mirrors ───────────────────────────────────────────────────
# Swap apt sources (Ubuntu/ports + ROS 2) to TUNA. Use `find` + sed
# --follow-symlinks because /etc/apt/sources.list.d/ros2.sources is
# a symlink into /usr/share/ros-apt-source/ in the upstream ros image.
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
ENV PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    UV_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    HF_ENDPOINT=https://hf-mirror.com

# Runtime deps:
#   rtabmap-ros: SLAM core + rtabmap_viz GUI for dev inspection
#   tf2 / tf-transformations: pose lookups in atlas_bridge
#   pointcloud_to_laserscan: depth → 2D scan adapter (used by dlio
#     fallback paths and any future depth-only deploys)
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-numpy \
        python3-yaml \
        ros-humble-rtabmap-ros \
        ros-humble-imu-filter-madgwick \
        ros-humble-tf2-ros \
        ros-humble-tf-transformations \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rmw-zenoh-cpp \
 && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
        grpcio==1.80.0 \
        grpcio-tools==1.76.0 \
        protobuf==6.33.6 \
        pyyaml>=6.0 \
        numpy \
        pillow \
        'mcp>=1.28.1,<2'

WORKDIR /mapping
COPY entrypoint.sh /entrypoint.sh
COPY no_shm_profile.xml /etc/fastrtps_no_shm.xml
RUN chmod +x /entrypoint.sh

# Left for operators who explicitly override RMW back to FastRTPS.
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastrtps_no_shm.xml

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

ENTRYPOINT ["/entrypoint.sh"]
