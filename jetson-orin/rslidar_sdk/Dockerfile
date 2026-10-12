# ARCOSLAB - Universidad de Costa Rica, 2026
# Author: Rodrigo Chacon Quesada (rodrigo.chacon@ucr.ac.cr)
#Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#(https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)

ARG ZED_SDK=5.2
ARG JETPACK=6.1.0
ARG ROS_INSTALL_TYPE=base
ARG BASE_IMAGE_TAG=latest

FROM autonomousrobotscognitivesystemslab/arcoslab:zedsdk-${ZED_SDK}-jetson-jp${JETPACK}-ros2-humble-${ROS_INSTALL_TYPE}

ENV DEBIAN_FRONTEND=noninteractive

RUN <<-EOF
        set -e
        apt-get update
        apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        libyaml-cpp-dev \
        libpcap-dev
        rm -rf /var/lib/apt/lists/*
        apt-get clean
EOF

RUN <<-EOF
        set -e
        apt-get update
        apt-get install -y --no-install-recommends \
        ros-humble-point-cloud-transport \
        ros-humble-xacro \
        ros-humble-grid-map-rviz-plugin \
        ros-humble-tf-transformations \
        ros-humble-rqt-tf-tree \
        ros-humble-rviz2
        rm -rf /var/lib/apt/lists/*
        apt-get clean
EOF

COPY ./ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN chmod +x /sbin/ros_entrypoint.sh
ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]

WORKDIR /ros2_ws