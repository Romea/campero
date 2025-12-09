FROM ros:humble

RUN apt-get update \
    && apt-get install -yq --no-install-recommends \
        curl \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# add ros one repositories
RUN curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg \
    && . /etc/os-release \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $VERSION_CODENAME main" \
        | tee /etc/apt/sources.list.d/ros1.list

# install ros one
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-one-ros-core \
    && rm -rf /var/lib/apt/lists/*

# install dependencies
RUN --mount=type=bind,source=package.xml,target=/tmp/bridge/package.xml \
    apt-get update \
    && rosdep update \
    && rosdep install -iyr --from-paths /tmp/bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /bridge_ws

# compile this package (the bridge)
RUN --mount=type=bind,source=.,target=/bridge_ws/src/bridge \
    ROS2_DISTRO=${ROS_DISTRO} \
    && . /opt/ros/one/setup.sh \
    && . /opt/ros/${ROS2_DISTRO}/local_setup.sh \
    && ROS_VERSION=1 colcon --log-base /dev/null build \
    && rm -rf /bridge_ws/build

# override entrypoint
COPY docker/entrypoint.sh /ros_entrypoint.sh
