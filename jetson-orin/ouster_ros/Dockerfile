# ARCoS-Lab - Ouster OS1-128
# Jetson AGX Orin / JetPack 6.1 / ROS 2 Humble

ARG ZED_SDK=5.2
ARG JETPACK=6.1.0
ARG ROS_INSTALL_TYPE=base

FROM autonomousrobotscognitivesystemslab/arcoslab:zedsdk-${ZED_SDK}-jetson-jp${JETPACK}-ros2-humble-${ROS_INSTALL_TYPE}

ENV DEBIAN_FRONTEND=noninteractive

# Ouster ROS 2 dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        python3-colcon-common-extensions \
        libeigen3-dev \
        libjsoncpp-dev \
        libspdlog-dev \
        libcurl4-openssl-dev \
        libssl-dev \
        libtins-dev \
        libzip-dev \
        libpcl-dev \
        libopencv-dev \
        ros-humble-pcl-ros \
        ros-humble-tf2-eigen \
        ros-humble-cv-bridge \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rviz2 && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN chmod +x /sbin/ros_entrypoint.sh

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]

WORKDIR /ros2_ws
