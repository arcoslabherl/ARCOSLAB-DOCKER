# ARCOSLAB - Universidad de Costa Rica, 2026
# Author: Rodrigo Chacon Quesada (rodrigo.chacon@ucr.ac.cr)
#Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#(https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)

ARG ZED_SDK=5.2
ARG JETPACK=6.1.0

FROM stereolabs/zed:${ZED_SDK}-tools-devel-jetson-jp${JETPACK}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y --no-install-recommends wget
RUN apt-get install -y --no-install-recommends iputils-ping
RUN apt-get install -y --no-install-recommends git
RUN apt-get install -y --no-install-recommends curl
RUN apt-get install -y --no-install-recommends ca-certificates
RUN apt-get install -y --no-install-recommends software-properties-common
RUN rm -rf /var/lib/apt/lists/* 
RUN apt-get clean

# Install ROS 2 Humble  
RUN apt-get update
RUN add-apt-repository universe -y
RUN apt-get update && apt-get install -y curl gnupg \
    && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
       | grep -F 'tag_name' | awk -F'"' '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb \
       "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
RUN rm -rf /var/lib/apt/lists/*
RUN apt-get clean

RUN apt-get update
RUN apt install -y ros-humble-ros-base
RUN apt install -y ros-humble-rmw-cyclonedds-cpp
RUN rm -rf /var/lib/apt/lists/*
RUN apt-get clean

# Setup environment variables
COPY ./ros_entrypoint_jetson.sh /sbin/ros_entrypoint.sh
RUN chmod +x /sbin/ros_entrypoint.sh
ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]

WORKDIR /