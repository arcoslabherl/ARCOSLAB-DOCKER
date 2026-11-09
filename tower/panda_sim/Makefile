# ARCoS-Lab - Franka Panda Simulator
# Dell Precision 5860 Tower / ROS 2 Humble

CONTAINER_NAME = panda-sim-tower
TAG = arcoslab-panda:sim-humble

ROS_WS = $(CURDIR)/ros2_ws
ROS_SRC = $(ROS_WS)/src

# GELLO serial device (a /dev/serial/by-id/... path also works).
# It is passed to the container by 'make run' only if it exists then.
GELLO_PORT ?= /dev/ttyUSB0
GELLO_DEVICE = $(if $(wildcard $(GELLO_PORT)),--device=$(GELLO_PORT):$(GELLO_PORT),)

# Arm limits of 'make sim-teleop' (rad/s, rad/s^2). The defaults match the
# real robot's driver, so the simulator shows the response it would have.
ARM_VEL ?= 0.8
ARM_ACC ?= 1.0

ROS_SOURCE = source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash

# Run a long-lived script (sim, GELLO) in the container. 'docker exec'
# does not stop it when the terminal is closed, so stop its process
# group whenever this command ends (Ctrl+C, closed terminal, error).
define exec_session
	@bash -c 'trap "docker exec $(CONTAINER_NAME) bash /ros2_ws/stop_session.sh $$$$ >/dev/null 2>&1" EXIT; \
		trap "exit 129" HUP; trap "exit 130" INT; trap "exit 143" TERM; \
		docker exec -it -e SESSION_ID=$$$$ $(CONTAINER_NAME) bash $(1)'
endef

.PHONY: help .pull .build run debug .colcon_build sim sim-teleop jr-init test-j1 test-all-joints test-gripper test-panda gello gello-fake gello-enable gello-disable gello-offsets test-gello stop

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
	@echo "  make test-all-joints  Test all 7 Panda joints"
	@echo "  make test-gripper     Test Panda gripper"
	@echo "  make test-panda       Test all joints and gripper"
	@echo "  make sim-teleop     Start the simulator for GELLO control (no RBC)"
	@echo "  make gello          Start GELLO teleoperation (GELLO_PORT=$(GELLO_PORT))"
	@echo "  make gello-fake     Start GELLO with a fake device (no hardware)"
	@echo "  make gello-enable   Enable GELLO control"
	@echo "  make gello-disable  Disable GELLO control"
	@echo "  make gello-offsets GELLO_SIGNS=\"1 -1 ...\"  Calibrate a GELLO unit"
	@echo "  make test-gello     Test the GELLO pipeline (needs sim + gello)"
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

	@echo "Checking RBC process cleanup patch..."

	@if git -C $(ROS_SRC)/robot-body-coupler apply --reverse --check "$(CURDIR)/patches/rbc_process_cleanup.patch" >/dev/null 2>&1; then \
		echo "RBC process cleanup patch already applied."; \
	elif git -C $(ROS_SRC)/robot-body-coupler apply --check "$(CURDIR)/patches/rbc_process_cleanup.patch" >/dev/null 2>&1; then \
		git -C $(ROS_SRC)/robot-body-coupler apply "$(CURDIR)/patches/rbc_process_cleanup.patch" && \
		echo "RBC process cleanup patch applied."; \
	else \
		echo "ERROR: Cannot apply RBC process cleanup patch."; \
		exit 1; \
	fi

	@echo "Checking optional RBC launch patch..."

	@# The arm limits patch builds on this one and changes its context, so if
	@# that patch is applied, this one is too.
	@if git -C $(ROS_SRC)/robot-startup apply --reverse --check "$(CURDIR)/patches/sim_launch_use_rbc.patch" >/dev/null 2>&1 || \
	   git -C $(ROS_SRC)/robot-startup apply --reverse --check "$(CURDIR)/patches/sim_launch_ruckig_limits.patch" >/dev/null 2>&1; then \
		echo "Optional RBC launch patch already applied."; \
	elif git -C $(ROS_SRC)/robot-startup apply --check "$(CURDIR)/patches/sim_launch_use_rbc.patch" >/dev/null 2>&1; then \
		git -C $(ROS_SRC)/robot-startup apply "$(CURDIR)/patches/sim_launch_use_rbc.patch" && \
		echo "Optional RBC launch patch applied."; \
	else \
		echo "ERROR: Cannot apply optional RBC launch patch."; \
		exit 1; \
	fi

	@echo "Checking arm limits launch patch..."

	@if git -C $(ROS_SRC)/robot-startup apply --reverse --check "$(CURDIR)/patches/sim_launch_ruckig_limits.patch" >/dev/null 2>&1; then \
		echo "Arm limits launch patch already applied."; \
	elif git -C $(ROS_SRC)/robot-startup apply --check "$(CURDIR)/patches/sim_launch_ruckig_limits.patch" >/dev/null 2>&1; then \
		git -C $(ROS_SRC)/robot-startup apply "$(CURDIR)/patches/sim_launch_ruckig_limits.patch" && \
		echo "Arm limits launch patch applied."; \
	else \
		echo "ERROR: Cannot apply arm limits launch patch."; \
		exit 1; \
	fi

	@echo "Checking arm joint limits launch patch..."

	@if git -C $(ROS_SRC)/robot-startup apply --reverse --check "$(CURDIR)/patches/sim_launch_joint_limits.patch" >/dev/null 2>&1; then \
		echo "Arm joint limits launch patch already applied."; \
	elif git -C $(ROS_SRC)/robot-startup apply --check "$(CURDIR)/patches/sim_launch_joint_limits.patch" >/dev/null 2>&1; then \
		git -C $(ROS_SRC)/robot-startup apply "$(CURDIR)/patches/sim_launch_joint_limits.patch" && \
		echo "Arm joint limits launch patch applied."; \
	else \
		echo "ERROR: Cannot apply arm joint limits launch patch."; \
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
	@$(if $(GELLO_DEVICE),echo "GELLO found at $(GELLO_PORT).",echo "Note: no GELLO at $(GELLO_PORT). Plug it in and re-run 'make run' to use it.")
	docker run \
	--detach \
	--rm \
	--init \
	--network=host \
	-e DISPLAY=$(DISPLAY) \
	-e QT_X11_NO_MITSHM=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v $(ROS_WS):/ros2_ws:rw \
	$(GELLO_DEVICE) \
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
	$(call exec_session,/ros2_ws/start_sim.sh)

sim-teleop:
	$(call exec_session,/ros2_ws/start_sim.sh teleop $(ARM_VEL) $(ARM_ACC))

jr-init:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/setup_jr.sh

test-j1:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/test_joint.sh

test-all-joints:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/test_all_joints.sh

test-gripper:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/test_gripper.sh

test-panda:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/test_panda.sh

gello:
	$(call exec_session,/ros2_ws/start_gello.sh dynamixel $(GELLO_PORT))

gello-fake:
	$(call exec_session,/ros2_ws/start_gello.sh fake)

gello-enable:
	docker exec -it $(CONTAINER_NAME) bash -c \
		"$(ROS_SOURCE) && ros2 service call /gello/enable std_srvs/srv/SetBool '{data: true}'"

gello-disable:
	docker exec -it $(CONTAINER_NAME) bash -c \
		"$(ROS_SOURCE) && ros2 service call /gello/enable std_srvs/srv/SetBool '{data: false}'"

gello-offsets:
	@if [ -z "$(GELLO_SIGNS)" ]; then \
		echo 'Usage: make gello-offsets GELLO_SIGNS="1 -1 1 1 1 -1 1" [GELLO_START="0 0 0 -1.57 0 1.57 0"]'; \
		exit 1; \
	fi
	docker exec -it $(CONTAINER_NAME) bash -c \
		"$(ROS_SOURCE) && ros2 run gello_teleop gello_get_offset --port $(GELLO_PORT) --joint-signs $(GELLO_SIGNS) $(if $(GELLO_START),--start-joints $(GELLO_START),)"

test-gello:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/test_gello.sh

stop:
	-docker stop -t 1 $(CONTAINER_NAME)
	@# --rm removal finishes after 'docker stop' returns; wait for it
	@# so an immediate 'make run' does not hit a name conflict.
	@while docker container inspect $(CONTAINER_NAME) >/dev/null 2>&1; do \
		sleep 0.2; \
	done