ARG JETSON_READONLY_IMAGE

# The reviewed local ARM64 read-only image already contains ROS 2 Humble and
# the generated unitree_go interfaces.  This derivative starts none of its
# sensor/chassis entrypoints and adds only chrony plus the subscription tools.
FROM ${JETSON_READONLY_IMAGE}

ARG TARGETARCH
ARG VCS_REF=unknown
USER root

RUN test "${TARGETARCH}" = "arm64"
RUN --mount=type=cache,id=robonix-go2-time-apt-archives-arm64,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=robonix-go2-time-apt-state-arm64,target=/var/lib/apt,sharing=locked \
    apt-get \
      -o Acquire::Retries=5 \
      -o Acquire::http::Timeout=30 \
      -o Acquire::https::Timeout=30 \
      -o APT::Update::Error-Mode=any \
      update \
    && apt-get \
      -o Acquire::Retries=5 \
      -o Acquire::http::Timeout=30 \
      -o Acquire::https::Timeout=30 \
      install --yes --no-install-recommends chrony \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid 10002 robonix-time \
    && useradd \
       --uid 10002 \
       --gid 10002 \
       --no-create-home \
       --home-dir /nonexistent \
       --shell /usr/sbin/nologin \
       robonix-time \
    && test "$(id -u robonix)" = "10001" \
    && test "$(id -g robonix)" = "10001" \
    && test "$(id -u robonix-time)" = "10002" \
    && test "$(id -g robonix-time)" = "10002" \
    && install -d -o root -g root -m 0755 \
       /opt/robonix/deploy/time-sync \
       /opt/robonix/scripts \
       /opt/robonix/time-sync/config \
    && install -d -o root -g root -m 0711 /run/robonix-go2-time \
    && install -d -o 10002 -g 10001 -m 2770 \
       /run/robonix-go2-time/refclock \
    && install -d -o 10002 -g 10002 -m 0700 \
       /run/robonix-go2-time/control \
       /run/robonix-go2-time/state

COPY deploy/time-sync/go2_time_core.py /opt/robonix/deploy/time-sync/go2_time_core.py
COPY deploy/time-sync/go2_clock_ref.py /opt/robonix/deploy/time-sync/go2_clock_ref.py
COPY deploy/time-sync/post_bootstrap_quality_gate.py /opt/robonix/deploy/time-sync/post_bootstrap_quality_gate.py
COPY scripts/probe_go2_time_readonly.py /opt/robonix/scripts/probe_go2_time_readonly.py
COPY deploy/jetson-time-sync/config /opt/robonix/time-sync/config
COPY deploy/jetson-time-sync/entrypoint.sh /opt/robonix/time-sync/entrypoint.sh
COPY deploy/jetson-time-sync/healthcheck.sh /opt/robonix/time-sync/healthcheck.sh
COPY deploy/jetson-time-sync/validate-network.sh /opt/robonix/time-sync/validate-network.sh

RUN chmod 0444 \
      /opt/robonix/deploy/time-sync/go2_time_core.py \
      /opt/robonix/deploy/time-sync/go2_clock_ref.py \
      /opt/robonix/deploy/time-sync/post_bootstrap_quality_gate.py \
      /opt/robonix/scripts/probe_go2_time_readonly.py \
      /opt/robonix/time-sync/config/*.conf \
    && chmod 0555 \
      /opt/robonix/time-sync/entrypoint.sh \
      /opt/robonix/time-sync/healthcheck.sh \
      /opt/robonix/time-sync/validate-network.sh \
    && ! grep -ERn \
      'create_publisher|/cmd_vel|/lowcmd|/api/sport/request|SportClient|clock_settime|settimeofday' \
      /opt/robonix/deploy/time-sync/*.py \
      /opt/robonix/scripts/probe_go2_time_readonly.py

LABEL org.opencontainers.image.title="Robonix Go2 Jetson time components" \
      org.opencontainers.image.description="Separated zero-capability DDS feeder and networkless chronyd" \
      org.opencontainers.image.revision="${VCS_REF}" \
      org.opencontainers.image.source="https://github.com/syswonder/robot-unitree-go2" \
      io.robonix.go2.profile="jetson-time-sync" \
      io.robonix.go2.motion="absent" \
      io.robonix.go2.clock-default="observe-only" \
      io.robonix.go2.time-topology="dual-container"

ENV HOME=/dev/shm/robonix-time/home \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=0 \
    ROS_LOCALHOST_ONLY=0 \
    GO2_TIME_PROFILE=probe

USER 10001:10001
WORKDIR /dev/shm

HEALTHCHECK --interval=10s --timeout=3s --start-period=20s --retries=3 \
  CMD ["/opt/robonix/time-sync/healthcheck.sh"]

ENTRYPOINT ["/opt/robonix/time-sync/entrypoint.sh"]
