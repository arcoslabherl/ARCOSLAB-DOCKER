# ARCoS-Lab - Franka Panda Simulator
# Dell Precision 5860 Tower / ROS 2 Humble

CONTAINER_NAME = panda-sim-tower
TAG = arcoslab-panda:sim-humble

ROS_WS = $(CURDIR)/ros2_ws
ROS_SRC = $(ROS_WS)/src

.PHONY: help .pull .build run debug .colcon_build sim jr-init test-j1 stop 

help:
	@echo "Available commands:"
	@echo "  make .pull          Download simulator repositories"
	@echo "  make .build         Build Docker image"
	@echo "  make run            Start Docker container"
	@echo "  make debug          Open container terminal"
	@echo "  make .colcon_build  Build ROS 2 workspace"
	@echo "  make sim            Launch Franka Panda simulator"
	@echo "  make jr-init        Initialize Panda Joint Reacher"
	@echo "  make test-j1        Test Panda J1 movement"
	@echo "  make stop           Stop Docker container"

.pull:
	mkdir -p $(ROS_SRC)
	@echo "Downloading simulator dependencies..."

	@if [ ! -d "$(ROS_SRC)/virtual-robot/.git" ]; then \
	git clone --branch main \
	https://gitlab.com/arcoslab/arcos-lab-cognitive-architecture/ros2-generic-multi-joint-impedance-control-virtual-robot.git \
	$(ROS_SRC)/virtual-robot; \
	fi

	@if [ ! -d "$(ROS_SRC)/robot-body-coupler/.git" ]; then \
	git clone --branch main \
	https://gitlab.com/arcoslab/arcos-lab-cognitive-architecture/robot-body-coupler/robot-body-coupler.git \
	$(ROS_SRC)/robot-body-coupler; \
	fi

	@if [ ! -d "$(ROS_SRC)/robot-descriptions/.git" ]; then \
	git clone --branch main \
	https://gitlab.com/arcoslab/robot-hardware-firmware/collaborative-robot/arcos-lab-asimilar-platform-robot-descriptions.git \
	$(ROS_SRC)/robot-descriptions; \
	fi

	@if [ ! -d "$(ROS_SRC)/robot-startup/.git" ]; then \
	git clone --branch main \
	https://gitlab.com/arcoslab/robot-hardware-firmware/collaborative-robot/arcos-lab-collaborative-robot-startup.git \
	$(ROS_SRC)/robot-startup; \
	fi

	@if [ ! -d "$(ROS_SRC)/panda-driver-topics/.git" ]; then \
	git clone --branch main \
	https://gitlab.com/arcoslab/robot-hardware-firmware/drivers/ros2-panda-driver-topics.git \
	$(ROS_SRC)/panda-driver-topics; \
	fi

	@echo "Checking repository versions..."

	git -C $(ROS_SRC)/virtual-robot checkout --detach de0db87781810f7ed8304ebc713f5391c1c74097

	git -C $(ROS_SRC)/robot-body-coupler checkout --detach 55a76c2d6687737d63a1d809bc54576e00863699

	git -C $(ROS_SRC)/robot-descriptions checkout --detach 5a5fecd2f6be4b77cbd23efd9cf1e9193f785395

	git -C $(ROS_SRC)/robot-startup checkout --detach 93c9fe6b578cc4e631a3a8501fd7207abefc6540

	git -C $(ROS_SRC)/panda-driver-topics checkout --detach ee14a1e6f9666bd2b9a391fa30da79fd00ee5e42


	@echo "Checking platform mesh compatibility patch..."

	@if git -C $(ROS_SRC)/robot-descriptions apply --reverse --check "$(CURDIR)/patches/platform_mesh_package.patch" >/dev/null 2>&1; then \
		echo "Platform mesh patch already applied."; \
	elif git -C $(ROS_SRC)/robot-descriptions apply --check "$(CURDIR)/patches/platform_mesh_package.patch" >/dev/null 2>&1; then \
		git -C $(ROS_SRC)/robot-descriptions apply "$(CURDIR)/patches/platform_mesh_package.patch" && \
		echo "Platform mesh patch applied."; \
	else \
		echo "ERROR: Cannot apply platform mesh patch."; \
		exit 1; \
	fi

	@echo "Checking RViz configuration patch..."

	@if git -C $(ROS_SRC)/robot-startup apply --reverse --check "$(CURDIR)/patches/rviz_config.patch" >/dev/null 2>&1; then \
		echo "RViz configuration patch already applied."; \
	elif git -C $(ROS_SRC)/robot-startup apply --check "$(CURDIR)/patches/rviz_config.patch" >/dev/null 2>&1; then \
		git -C $(ROS_SRC)/robot-startup apply "$(CURDIR)/patches/rviz_config.patch" && \
		echo "RViz configuration patch applied."; \
	else \
		echo "ERROR: Cannot apply RViz configuration patch."; \
		exit 1; \
	fi

	@echo "Checking Joint Reacher compatibility patch..."

	@if git -C $(ROS_SRC)/robot-body-coupler apply --reverse --check "$(CURDIR)/patches/joint_reacher_cycle_freq.patch" >/dev/null 2>&1; then \
		echo "Joint Reacher patch already applied."; \
	elif git -C $(ROS_SRC)/robot-body-coupler apply --check "$(CURDIR)/patches/joint_reacher_cycle_freq.patch" >/dev/null 2>&1; then \
		git -C $(ROS_SRC)/robot-body-coupler apply "$(CURDIR)/patches/joint_reacher_cycle_freq.patch" && \
		echo "Joint Reacher patch applied."; \
	else \
		echo "ERROR: Cannot apply Joint Reacher patch."; \
		exit 1; \
	fi

	@echo "Simulator repositories downloaded."

.build:
	docker build \
	--tag=$(TAG) \
	--network=host \
	.

run:
	@xhost +si:localuser:root > /dev/null
	docker run \
	--detach \
	--rm \
	--network=host \
	-e DISPLAY=$(DISPLAY) \
	-e QT_X11_NO_MITSHM=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v $(ROS_WS):/ros2_ws:rw \
	--name $(CONTAINER_NAME) \
	$(TAG) \
	bash -c "tail -f /dev/null"

debug:
	docker exec -it $(CONTAINER_NAME) bash -c \
	  "source /opt/ros/humble/setup.bash && \
	  source /ros2_ws/install/setup.bash && \
	  exec bash"

.colcon_build:
	docker exec -it $(CONTAINER_NAME) bash \
	/ros2_ws/build_ws.sh

sim:
	docker exec -it $(CONTAINER_NAME) bash \
	/ros2_ws/start_sim.sh

jr-init:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/setup_jr.sh

test-j1:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/test_joint.sh

stop:
	docker stop -t 1 $(CONTAINER_NAME)