# ARCoS-Lab - Franka Panda Simulator
# Dell Precision 5860 Tower / ROS 2 Humble

# Pinned by digest (ros:humble-ros-base as of 2026-10-27), so a rebuild
# starts from the same base. To update: pull the tag, put its digest here
# ('docker image inspect ros:humble-ros-base'), then run 'make test-sim'.
FROM ros:humble-ros-base@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855

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