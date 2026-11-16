# ARCoS-Lab - Franka Panda Simulator
# Dell Precision 5860 Tower / ROS 2 Humble

# Pinned by digest (ros:humble-ros-base as of 2026-10-27), so a rebuild
# starts from the same base. To update: pull the tag, put its digest here
# ('docker image inspect ros:humble-ros-base'), then run 'make test-sim'.
FROM ros:humble-ros-base@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855 AS sim

ENV DEBIAN_FRONTEND=noninteractive

# System and ROS 2 dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        pkg-config \
        libeigen3-dev \
        python3-pip \
        python3-numpy \
        python3-scipy \
        python3-colcon-common-extensions \
        python3-numba \
        python3-serial \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-tf2-ros \
        ros-humble-ruckig \
        ros-humble-rosidl-default-generators && \
    rm -rf /var/lib/apt/lists/*

# Python dependencies for Robot Body Coupler and GELLO (Dynamixel).
# --no-deps: quaternionic's own dependencies (numpy, scipy, numba) and
# dynamixel-sdk's (pyserial) come from apt above.
RUN python3 -m pip install --no-cache-dir --no-deps \
    quaternionic==1.0.17 \
    dynamixel-sdk==3.7.31

# ROS 2 environment
COPY ./ros_entrypoint.sh /sbin/ros_entrypoint.sh

RUN chmod +x /sbin/ros_entrypoint.sh

WORKDIR /ros2_ws

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]

# Real robot image ('make .build-real'): the simulator image plus libfranka.
# The lab's Panda runs System Version 4.0.4; libfranka 0.9.x needs >= 4.2.1,
# 0.8.0 needs >= 4.0.0 and is the one the lab has used with this robot.
# The examples (echo_robot_state, communication_test) stay in
# /opt/libfranka/build/examples for the first checks at the lab.
FROM sim AS real

ARG LIBFRANKA_VERSION=0.8.0

RUN apt-get update && \
    apt-get install -y --no-install-recommends libpoco-dev && \
    rm -rf /var/lib/apt/lists/*

# 0.8.0 is from 2020: Ubuntu 22.04's GCC 11 needs two missing #includes
# (patches/libfranka_0.8.0_gcc11.patch).
COPY patches/libfranka_0.8.0_gcc11.patch /tmp/libfranka.patch

RUN git clone --recursive --depth 1 --shallow-submodules \
        --branch ${LIBFRANKA_VERSION} \
        https://github.com/frankaemika/libfranka.git /opt/libfranka && \
    git -C /opt/libfranka apply /tmp/libfranka.patch && \
    cmake -S /opt/libfranka -B /opt/libfranka/build \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF && \
    cmake --build /opt/libfranka/build -j"$(nproc)" && \
    cmake --install /opt/libfranka/build && \
    ldconfig

