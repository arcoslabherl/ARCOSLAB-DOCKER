# ARCOSLAB - Universidad de Costa Rica, 2026
# Author: Rodrigo Chacon Quesada (rodrigo.chacon@ucr.ac.cr)
#Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#(https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)

THIS_FILE := $(lastword $(MAKEFILE_LIST))

ZED_SDK = 5.2
JETPACK = 6.1.0
ROS_INSTALL_TYPE = base

CONTAINER_NAME = zedsdk-${ZED_SDK}-jetson-jp${JETPACK}-ros2-humble-${ROS_INSTALL_TYPE}
TAG = autonomousrobotscognitivesystemslab/arcoslab:${CONTAINER_NAME}

.PHONY: help defualt .build run debug stop

help:			## Shows this help
	@echo 'Usage: make [target]'
	@echo
	@echo 'Targets:'
	@echo
	@fgrep -h "##" $(MAKEFILE_LIST) | fgrep -v fgrep | sed -e 's/\\$$//' | sed -e 's/##//'

.build:			## Builds Docker image
	docker build \
	--tag=${TAG} \
	--build-arg ZED_SDK=${ZED_SDK} \
	--build-arg JETPACK=${JETPACK} \
	.

.start_if_not_running:
	@if ! docker ps -a | grep -w ${CONTAINER_NAME}; then $(MAKE) run; fi


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
			--name ${CONTAINER_NAME} \
			${TAG} bash -c \
			"tail -f /dev/null"


debug: 			## Starts a cli inside the container
debug:.start_if_not_running
	@docker exec -it ${CONTAINER_NAME} bash

explorer:		## Starts ZED_Explorer from Zed tools
explorer:.start_if_not_running
	docker exec -it ${CONTAINER_NAME} bash -c "cd /usr/local/zed/tools && ./ZED_Explorer"

depth:			## Starts ZED_Depth_Viewer from Zed tools
depth:.start_if_not_running
	docker exec -it ${CONTAINER_NAME} bash -c "cd /usr/local/zed/tools && ./ZED_Depth_Viewer"

360:			## Starts ZED360 from Zed tools
360:.start_if_not_running
	docker exec -it ${CONTAINER_NAME} bash -c "cd /usr/local/zed/tools && ./ZED360"

calibration:	## Starts ZED_Calibration from Zed tools
calibration:.start_if_not_running
	docker exec -it ${CONTAINER_NAME} bash -c "cd /usr/local/zed/tools && ./ZED_Calibration"

fu:				## Starts ZEDfu from Zed tools
fu:.start_if_not_running
	docker exec -it ${CONTAINER_NAME} bash -c "cd /usr/local/zed/tools && ./ZEDfu"


stop:			## Stops Docker container
	@docker stop -t 1 ${CONTAINER_NAME} >> /dev/null