# syntax=docker/dockerfile:1.6

# build.sh accepts only a locally loaded, content-addressed image that has
# already passed the jetson-readonly ARM64 checks.  The build never resolves a
# mutable registry tag or pulls a base image implicitly.
ARG JETSON_READONLY_IMAGE

FROM --platform=linux/arm64 ${JETSON_READONLY_IMAGE} AS verified_base

ARG TARGETARCH
USER root

RUN test "${TARGETARCH}" = "arm64" \
    && test "$(uname -m)" = "aarch64" \
    && grep -Fqx 'VERSION_ID="22.04"' /etc/os-release \
    && test -r /opt/ros/humble/setup.bash \
    && test -x /opt/robonix/overlay/lib/go2_chassis_adapter/go2_chassis_adapter_node \
    && test ! -e /opt/robonix/overlay/bin/go2_sport_daemon

# Build only the Nav2 terminal-controller plugins.  The Unitree SDK checkout,
# Go2 SDK daemon, and all vendor examples are deliberately absent from this
# Dockerfile's allowlisted context.
FROM verified_base AS navigation_builder

ARG TARGETARCH
ENV DEBIAN_FRONTEND=noninteractive

RUN test "${TARGETARCH}" = "arm64"
RUN --mount=type=cache,id=robonix-go2-full-nomotion-apt-archives-arm64,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=robonix-go2-full-nomotion-apt-state-arm64,target=/var/lib/apt,sharing=locked \
    test -f /etc/apt/apt.conf.d/docker-clean \
    && mv /etc/apt/apt.conf.d/docker-clean /tmp/docker-clean \
    && apt-get \
        -o Acquire::Retries=5 \
        -o Acquire::http::Timeout=30 \
        -o Acquire::https::Timeout=30 \
        -o APT::Update::Error-Mode=any \
        update \
    && apt-get \
        -o Acquire::Retries=5 \
        -o Acquire::http::Timeout=30 \
        -o Acquire::https::Timeout=30 \
        install --yes --no-install-recommends \
        build-essential \
        cmake \
        python3-colcon-common-extensions \
        ros-humble-nav2-core \
        ros-humble-nav2-dwb-controller \
        ros-humble-pluginlib \
    && mv /tmp/docker-clean /etc/apt/apt.conf.d/docker-clean

WORKDIR /work/navigation
COPY third_party/service-navigation-rbnx/terminal_controller ./terminal_controller

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN source /opt/ros/humble/setup.bash \
    && colcon --log-base /work/navigation/log build \
        --base-paths /work/navigation/terminal_controller \
        --build-base /work/navigation/build \
        --install-base /opt/robonix/navigation-overlay \
        --cmake-args -DBUILD_TESTING=OFF

FROM verified_base AS runtime_blueprint

ARG TARGETARCH
ARG VCS_REF=unknown
ARG BASE_IMAGE_ID=unknown
ARG MANIFEST_SHA256=unknown
ENV DEBIAN_FRONTEND=noninteractive

USER root
RUN test "${TARGETARCH}" = "arm64"
RUN --mount=type=cache,id=robonix-go2-full-nomotion-apt-archives-arm64,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=robonix-go2-full-nomotion-apt-state-arm64,target=/var/lib/apt,sharing=locked \
    test -f /etc/apt/apt.conf.d/docker-clean \
    && mv /etc/apt/apt.conf.d/docker-clean /tmp/docker-clean \
    && apt-get \
        -o Acquire::Retries=5 \
        -o Acquire::http::Timeout=30 \
        -o Acquire::https::Timeout=30 \
        -o APT::Update::Error-Mode=any \
        update \
    && apt-get \
        -o Acquire::Retries=5 \
        -o Acquire::http::Timeout=30 \
        -o Acquire::https::Timeout=30 \
        install --yes --no-install-recommends \
        iproute2 \
        python3-yaml \
        ros-humble-imu-filter-madgwick \
        ros-humble-nav2-bringup \
        ros-humble-navigation2 \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-rtabmap-ros \
    && mv /tmp/docker-clean /etc/apt/apt.conf.d/docker-clean

COPY --from=navigation_builder \
  /opt/robonix/navigation-overlay /opt/robonix/navigation-overlay

# These are source snapshots for a later, pinned ARM64 Robonix build.  No
# package build or boot happens in this image because the Robonix ARM64 CLI,
# generated contracts, speech assets, and runtime supervisor are not yet
# present and verified.
COPY third_party/service-map-rbnx /opt/robonix/deploy/third_party/service-map-rbnx
COPY third_party/service-navigation-rbnx /opt/robonix/deploy/third_party/service-navigation-rbnx
COPY packages/go2_chassis /opt/robonix/deploy/packages/go2_chassis
COPY packages/go2_sensors /opt/robonix/deploy/packages/go2_sensors
COPY packages/go2_description /opt/robonix/deploy/packages/go2_description
COPY packages/go2_dashboard /opt/robonix/deploy/packages/go2_dashboard
COPY packages/semantic_navigation /opt/robonix/deploy/packages/semantic_navigation
COPY packages/semantic_intent_router /opt/robonix/deploy/packages/semantic_intent_router

COPY config /opt/robonix/deploy/config
COPY soma.yaml /opt/robonix/deploy/soma.yaml
COPY deploy/jetson-full-nomotion/robonix_manifest.yaml \
  /opt/robonix/deploy/robonix_manifest.yaml
COPY deploy/jetson-full-nomotion/profile.yaml /opt/robonix/profile/profile.yaml
COPY deploy/jetson-full-nomotion/entrypoint.sh /opt/robonix/profile/entrypoint.sh
COPY deploy/jetson-full-nomotion/healthcheck.sh /opt/robonix/profile/healthcheck.sh
COPY deploy/jetson-full-nomotion/validate-network.sh /opt/robonix/profile/validate-network.sh
COPY deploy/jetson-full-nomotion/verify-image-rootfs.sh \
  /opt/robonix/profile/verify-image-rootfs.sh

RUN chmod 0555 \
      /opt/robonix/profile/entrypoint.sh \
      /opt/robonix/profile/healthcheck.sh \
      /opt/robonix/profile/validate-network.sh \
      /opt/robonix/profile/verify-image-rootfs.sh \
    && chmod 0444 \
      /opt/robonix/profile/profile.yaml \
      /opt/robonix/deploy/soma.yaml \
      /opt/robonix/deploy/robonix_manifest.yaml \
    && /opt/robonix/profile/verify-image-rootfs.sh \
    && rm /opt/robonix/profile/verify-image-rootfs.sh

LABEL org.opencontainers.image.title="Robonix Go2 Jetson full-stack no-motion blueprint" \
      org.opencontainers.image.description="ARM64 ROS Humble/Nav2/RTAB-Map blueprint; intentionally not runtime-complete" \
      org.opencontainers.image.revision="${VCS_REF}" \
      org.opencontainers.image.source="https://github.com/syswonder/robot-unitree-go2" \
      io.robonix.go2.base.image-id="${BASE_IMAGE_ID}" \
      io.robonix.go2.manifest.sha256="${MANIFEST_SHA256}" \
      io.robonix.go2.profile="jetson-full-nomotion" \
      io.robonix.go2.motion="false" \
      io.robonix.go2.velocity-output="/robonix/nomotion/cmd_vel" \
      io.robonix.go2.runtime-complete="false"

ENV HOME=/tmp/home \
    ROS_DISTRO=humble \
    ROS_DOMAIN_ID=0 \
    ROS_LOCALHOST_ONLY=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROBONIX_DEPLOY_DIR=/opt/robonix/deploy \
    ROBONIX_MOTION_ENABLED=false \
    GO2_CHASSIS_ALLOW_MOTION=false \
    ROBONIX_RUNTIME_COMPLETE=0 \
    ROBONIX_VELOCITY_OUTPUT_TOPIC=/robonix/nomotion/cmd_vel \
    NVIDIA_VISIBLE_DEVICES=void \
    NVIDIA_DRIVER_CAPABILITIES=

USER 10001:10001
WORKDIR /tmp

HEALTHCHECK --interval=10s --timeout=8s --start-period=10s --retries=2 \
  CMD ["/opt/robonix/profile/healthcheck.sh"]

ENTRYPOINT ["/opt/robonix/profile/entrypoint.sh"]
