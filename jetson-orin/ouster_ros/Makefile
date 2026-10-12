# ARCoS-Lab - Ouster OS1-128
# Jetson AGX Orin / ROS 2 Humble

ZED_SDK = 5.2
JETPACK = 6.1.0
ROS_INSTALL_TYPE = base

CONTAINER_NAME = ouster-os1-128
TAG = arcoslab-ouster:os1-128

OUSTER_DIR = ros2_ws/src/ouster-ros
OUSTER_COMMIT = bbe9a890012226f7d9354250bbe212852b2443f2

.PHONY: help .pull .build .colcon_build run debug stop

help:
	@echo "Available commands:"
	@echo "  make .pull          Download Ouster dependencies"
	@echo "  make .build         Build Docker image"
	@echo "  make run            Start Docker container"
	@echo "  make debug          Open container terminal"
	@echo "  make .colcon_build  Build ROS 2 workspace"
	@echo "  make stop           Stop Docker container"

.pull:
	mkdir -p ros2_ws/src
	@if [ ! -d "$(OUSTER_DIR)/.git" ]; then \
	git clone -b ros2 --recurse-submodules \
	https://github.com/ouster-lidar/ouster-ros.git \
	$(OUSTER_DIR); \
	git -C $(OUSTER_DIR) checkout --detach $(OUSTER_COMMIT); \
	fi
	@test "$$(git -C $(OUSTER_DIR) rev-parse HEAD)" = "$(OUSTER_COMMIT)" || \
	{ echo "ERROR: Unexpected Ouster driver version"; exit 1; }
	git -C $(OUSTER_DIR) submodule update --init --recursive
	@if git -C $(OUSTER_DIR) apply --reverse --check "$(CURDIR)/patches/ouster-window-compat.patch" >/dev/null 2>&1; then echo "Ouster WINDOW patch already applied."; elif git -C $(OUSTER_DIR) apply --check "$(CURDIR)/patches/ouster-window-compat.patch" >/dev/null 2>&1; then git -C $(OUSTER_DIR) apply "$(CURDIR)/patches/ouster-window-compat.patch" && echo "Ouster WINDOW patch applied."; else echo "ERROR: Cannot apply Ouster WINDOW patch."; exit 1; fi

.build:
	docker build \
	--tag=$(TAG) \
	--network=host \
	--build-arg ZED_SDK=$(ZED_SDK) \
	--build-arg JETPACK=$(JETPACK) \
	--build-arg ROS_INSTALL_TYPE=$(ROS_INSTALL_TYPE) \
	.

run:
	@xhost +si:localuser:root > /dev/null
	docker run \
	--detach \
	--rm \
	--net=host \
	--runtime nvidia \
	-e DISPLAY=$(DISPLAY) \
	-e QT_X11_NO_MITSHM=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v $(CURDIR)/config:/ouster_config:ro \
	-v $(CURDIR)/ros2_ws:/ros2_ws:rw \
	--name $(CONTAINER_NAME) \
	$(TAG) bash -c "tail -f /dev/null"

debug:
	@docker exec -it $(CONTAINER_NAME) bash -c \
	"source /sbin/ros_entrypoint.sh && exec bash"

.colcon_build:
	@docker exec -it $(CONTAINER_NAME) bash -c \
	"source /ros2_ws/build_ws.sh"

stop:
	@docker stop -t 1 $(CONTAINER_NAME)
