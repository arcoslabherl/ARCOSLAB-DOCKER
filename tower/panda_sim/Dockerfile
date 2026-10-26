# ARCoS-Lab - Franka Panda Simulator
# Dell Precision 5860 Tower / ROS 2 Humble

FROM ros:humble-ros-base

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
        python3-rosdep \
        python3-vcstool \
        python3-numba \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-tf2-ros \
        ros-humble-ruckig \
        ros-humble-rosidl-default-generators && \
    rm -rf /var/lib/apt/lists/*

# Python dependencies for Robot Body Coupler
RUN python3 -m pip install --no-cache-dir --no-deps \
    quaternionic==1.0.17

# ROS 2 environment
COPY ./ros_entrypoint.sh /sbin/ros_entrypoint.sh

RUN chmod +x /sbin/ros_entrypoint.sh

WORKDIR /ros2_ws

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]