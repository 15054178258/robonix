ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod 0755 /entrypoint.sh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENTRYPOINT ["/entrypoint.sh"]

