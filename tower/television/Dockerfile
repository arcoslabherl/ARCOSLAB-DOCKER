# ARCoS-Lab - TeleVision: ZED stereo video to a WebXR headset
# ZED SDK + ROS 2 Humble, Ubuntu 22.04

# Stereolabs' image: ZED SDK, CUDA runtime, pyzed, and numpy 2 and OpenCV
# from pip (so no python3-opencv/scipy from apt: built for numpy 1).
# Pinned by digest (as of 2026-11-17). CUDA 12.8 needs an NVIDIA driver
# >= 570 on the host ('nvidia-smi'). To update: pull the tag, put its digest here
# ('docker image inspect stereolabs/zed:<tag>'), then test 'make stream'.
FROM stereolabs/zed:5.6.0-runtime-cuda12.8-ubuntu22.04@sha256:f1898d9b2b25561fa566126b491fccf10b96d6ed59058bd96d30fa8699f0cb77

ENV DEBIAN_FRONTEND=noninteractive

# ROS 2 Humble apt repository (ros2-apt-source holds the signing key)
ARG ROS_APT_SOURCE_VERSION=1.3.0

RUN apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl && \
    curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb" && \
    apt-get install -y /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    rm -rf /var/lib/apt/lists/*

# ROS 2 and system dependencies. mkcert: the headset's HTTPS certificate.
# libgl1, libglib2.0-0: the base image's OpenCV needs them.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        libgl1 \
        libglib2.0-0 \
        python3-colcon-common-extensions \
        python3-pip \
        python3-pytest \
        mkcert && \
    rm -rf /var/lib/apt/lists/*

# Vuer (the headset's page, WebXR) at the version the lab tested with
# the Apple Vision Pro, and its dependencies, pinned. scipy: for numpy 2.
# chmod: some packages are root-only, and the container runs as the user.
RUN python3 -m pip install --no-cache-dir \
    vuer==0.0.32rc7 \
    params-proto==2.12.1 \
    aiohttp==3.9.5 \
    aiohttp-cors==0.7.0 \
    websockets==12.0 \
    msgpack==1.0.8 \
    pillow==10.3.0 \
    scipy==1.15.3 && \
    chmod -R a+rX /usr/local/lib/python3.10/dist-packages

# ROS 2 environment
COPY ./ros_entrypoint.sh /sbin/ros_entrypoint.sh

RUN chmod +x /sbin/ros_entrypoint.sh

WORKDIR /ros2_ws

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]
