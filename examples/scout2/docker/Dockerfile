ARG BASE=ubuntu:22.04
FROM ${BASE}

RUN apt-get update && apt-get install -y --no-install-recommends \
      curl lsb-release gnupg locales software-properties-common \
      python3-pip python3-dev python3-venv \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-* \
      ros-humble-geometry-msgs \
      ros-humble-nav-msgs \
      ros-humble-sensor-msgs \
      ros-humble-tf2-* \
      ros-humble-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/* && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Python deps
RUN pip3 install --no-cache-dir numpy Pillow onnxruntime

# Install robonix-api (host directory mounted at runtime; just install it
# here for build-time codegen).  The actual package source is mounted from
# the host via docker compose volumes.
WORKDIR /robonix_pkgs
COPY pylib/robonix-api /robonix_pkgs/pylib/robonix-api
RUN cd /robonix_pkgs/pylib/robonix-api && pip3 install -e .

ENV PYTHONPATH="/robonix_pkgs/pylib/robonix-api"
ENV RMW_IMPLEMENTATION="rmw_zenoh_cpp"
ENV ROS_DOMAIN_ID="0"

CMD ["sleep", "infinity"]
