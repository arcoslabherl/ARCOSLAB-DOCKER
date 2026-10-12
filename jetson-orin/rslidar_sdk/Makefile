# ARCOSLAB - Universidad de Costa Rica, 2026
# Author: Rodrigo Chacon Quesada (rodrigo.chacon@ucr.ac.cr)
#Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#(https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)


THIS_FILE := $(lastword $(MAKEFILE_LIST))

NPROC := $(shell nproc)
REMOTE = true

ZED_SDK = 5.2
JETPACK = 6.1.0
ROS_INSTALL_TYPE = base

CONTAINER_NAME = rslidar-e1r
TAG = arcoslab-rslidar:e1r

.PHONY: help defualt .build run debug stop

help:			## Shows this help
	@echo 'Usage: make [target]'
	@echo
	@echo 'Targets:'
	@echo
	@fgrep -h "##" $(MAKEFILE_LIST) | fgrep -v fgrep | sed -e 's/\\$$//' | sed -e 's/##//'

.pull: 			## Download dependencies and apply local configurations
	mkdir -p "$(CURDIR)/ros2_ws/src"

	@if [ ! -d ros2_ws/src/rslidar_sdk/.git ]; then \
	git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git ros2_ws/src/rslidar_sdk; \
	git -C ros2_ws/src/rslidar_sdk checkout --detach 7c4ea25; \
	fi

	@if [ ! -d ros2_ws/src/rslidar_msg/.git ]; then \
	git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git ros2_ws/src/rslidar_msg; \
	git -C ros2_ws/src/rslidar_msg checkout --detach fe8a95c; \
	fi

	@git -C ros2_ws/src/rslidar_sdk rev-parse HEAD | grep -q '^7c4ea25' || \
	{ echo "ERROR: Unexpected rslidar_sdk version"; exit 1; }

	@git -C ros2_ws/src/rslidar_msg rev-parse HEAD | grep -q '^fe8a95c' || \
	{ echo "ERROR: Unexpected rslidar_msg version"; exit 1; }

	git -C ros2_ws/src/rslidar_sdk submodule update --init --recursive

	cp config/rse1.yaml ros2_ws/src/rslidar_sdk/config/config.yaml
	cp config/rviz2.rviz ros2_ws/src/rslidar_sdk/rviz/rviz2.rviz

.build:			## Builds Docker image
	docker build \
	--tag=${TAG} \
	--network=host \
	--build-arg ZED_SDK=${ZED_SDK} \
	--build-arg JETPACK=${JETPACK} \
	--build-arg ROS_INSTALL_TYPE=${ROS_INSTALL_TYPE} \
	.

.colcon_build:		## Build ROS2 workspace
	@docker run \
			--detach \
			--rm \
			--runtime nvidia \
			--privileged \
			-v ${PWD}/ros2_ws:/ros2_ws:rw \
			--name ${CONTAINER_NAME} \
			${TAG} bash -c \
			"tail -f /dev/null"
	@docker exec \
			-it \
			${CONTAINER_NAME} bash -c \
			"source /opt/ros/humble/setup.bash && \
			cd /ros2_ws && \
			colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(NPROC)"

.start_if_not_running:
	@if ! docker ps -a | grep -w ${CONTAINER_NAME}; then if [ ${REMOTE} = true ]; then $(MAKE) run-remote; else $(MAKE) run; fi fi

run:			## Starts Docker image
	@xhost +si:localuser:root >> /dev/null
	@docker run \
			--detach \
			--rm \
			--net=host \
			--runtime nvidia \
			--privileged \
			-e DISPLAY=${DISPLAY} \
			-v /dev/:/dev/ \
			-v /tmp/:/tmp/ \
			-v /var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/ \
			-v /etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service \
			-v ${PWD}/resources/:/usr/local/zed/resources/ \
			-v ${PWD}/settings/:/usr/local/zed/settings:rw \
			-v ${PWD}/ros2_ws:/ros2_ws:rw \
			--name ${CONTAINER_NAME} \
			${TAG} bash -c \
			"tail -f /dev/null"

run-remote:		## Starts Docker image without xhost
run-remote:
	@docker run \
			--detach \
			--rm \
			--net=host \
			--runtime nvidia \
			--privileged \
			-e DISPLAY=${DISPLAY} \
			-v /dev/:/dev/ \
			-v /tmp/:/tmp/ \
			-v /var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/ \
			-v /etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service \
			-v /tmp/.X11-unix/:/tmp/.X11-unix \
			-v ${PWD}/resources/:/usr/local/zed/resources/ \
			-v ${PWD}/settings/:/usr/local/zed/settings:rw \
			-v ${PWD}/ros2_ws:/ros2_ws:rw \
			--name ${CONTAINER_NAME} \
			${TAG} bash -c \
			"tail -f /dev/null"

debug: 			## Starts a cli inside the container
debug:.start_if_not_running
	@docker exec -it ${CONTAINER_NAME} bash -c "source /sbin/ros_entrypoint.sh && exec bash"

stop:			## Stops Docker container
	@docker stop -t 1 ${CONTAINER_NAME} >> /dev/null