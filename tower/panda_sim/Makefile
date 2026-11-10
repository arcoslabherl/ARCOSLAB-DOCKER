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

# Arm limits of 'make sim-teleop' (rad/s, rad/s^2, rad/s^3). The defaults
# match the real robot's driver, so the simulator shows the response it
# would have.
ARM_VEL ?= 0.8
ARM_ACC ?= 1.0
ARM_JERK ?= 50

ROS_SOURCE = source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash

GITLAB = https://gitlab.com/arcoslab

# Run a long-lived script (sim, GELLO) in the container. 'docker exec'
# does not stop it when the terminal is closed, so stop its process
# group whenever this command ends (Ctrl+C, closed terminal, error).
define exec_session
	@bash -c 'trap "docker exec $(CONTAINER_NAME) bash /ros2_ws/scripts/stop_session.sh $$$$ >/dev/null 2>&1" EXIT; \
		trap "exit 129" HUP; trap "exit 130" INT; trap "exit 143" TERM; \
		docker exec -it -e SESSION_ID=$$$$ $(CONTAINER_NAME) bash $(1)'
endef

# Clone an external repository if missing and check out its pinned commit.
# $(call pull_repo,<dir in src/>,<url>,<commit>)
define pull_repo
	@if [ ! -d "$(ROS_SRC)/$(1)/.git" ]; then \
		git clone --branch main $(2) $(ROS_SRC)/$(1); \
	fi
	git -C $(ROS_SRC)/$(1) checkout --detach $(3)
endef

# Apply patches/<name>.patch to an external repository, unless it is
# already applied. $(call apply_patch,<dir in src/>,<name>)
define apply_patch
	@if git -C $(ROS_SRC)/$(1) apply --reverse --check "$(CURDIR)/patches/$(2).patch" >/dev/null 2>&1; then \
		echo "Patch $(2) already applied."; \
	elif git -C $(ROS_SRC)/$(1) apply --check "$(CURDIR)/patches/$(2).patch" >/dev/null 2>&1; then \
		git -C $(ROS_SRC)/$(1) apply "$(CURDIR)/patches/$(2).patch" && \
		echo "Patch $(2) applied."; \
	else \
		echo "ERROR: Cannot apply patch $(2) to src/$(1)."; \
		echo "If an older version of it is applied, reset the repository and run 'make .pull' again:"; \
		echo "    git -C $(ROS_SRC)/$(1) checkout -- ."; \
		exit 1; \
	fi
endef

.PHONY: help .pull .build run debug .colcon_build sim sim-teleop test-all-joints test-gripper test-panda gello gello-fake gello-enable gello-disable gello-offsets test-gello latency-record latency-replay latency-analyze stop

help:
	@echo "Available commands:"
	@echo "  make .pull          Download simulator repositories"
	@echo "  make .build         Build Docker image"
	@echo "  make run            Start Docker container"
	@echo "  make debug          Open container terminal"
	@echo "  make .colcon_build  Build ROS 2 workspace"
	@echo "  make sim            Launch Franka Panda simulator (with RBC)"
	@echo "  make test-all-joints  Test all 7 Panda joints"
	@echo "  make test-gripper     Test Panda gripper"
	@echo "  make test-panda       Test all joints and gripper"
	@echo "  make sim-teleop     Start the simulator for GELLO control (no RBC)"
	@echo "  make gello          Start GELLO teleoperation (GELLO_PORT=$(GELLO_PORT))"
	@echo "  make gello-fake     Start GELLO with a fake device (no hardware)"
	@echo "  make gello-enable   Enable GELLO control"
	@echo "  make gello-disable  Disable GELLO control"
	@echo "  make gello-offsets GELLO_SIGNS=\"1 -1 ...\"  Calibrate a GELLO unit"
	@echo "  make test-gello     Test the GELLO pipeline (needs sim-teleop + gello)"
	@echo "  make latency-record REC=name [DURATION=60]  Record a GELLO session"
	@echo "  make latency-replay REC=name RUN=name       Replay it to the sim"
	@echo "  make latency-analyze FILES=\"name ...\"      Delay, lag and overshoot"
	@echo "  make stop           Stop Docker container"

.pull:
	mkdir -p $(ROS_SRC)
	@echo "Downloading simulator repositories at their pinned commits..."
	$(call pull_repo,virtual-robot,$(GITLAB)/arcos-lab-cognitive-architecture/ros2-generic-multi-joint-impedance-control-virtual-robot.git,de0db87781810f7ed8304ebc713f5391c1c74097)
	$(call pull_repo,robot-body-coupler,$(GITLAB)/arcos-lab-cognitive-architecture/robot-body-coupler/robot-body-coupler.git,55a76c2d6687737d63a1d809bc54576e00863699)
	$(call pull_repo,robot-descriptions,$(GITLAB)/robot-hardware-firmware/collaborative-robot/arcos-lab-asimilar-platform-robot-descriptions.git,5a5fecd2f6be4b77cbd23efd9cf1e9193f785395)
	$(call pull_repo,robot-startup,$(GITLAB)/robot-hardware-firmware/collaborative-robot/arcos-lab-collaborative-robot-startup.git,93c9fe6b578cc4e631a3a8501fd7207abefc6540)
	$(call pull_repo,panda-driver-topics,$(GITLAB)/robot-hardware-firmware/drivers/ros2-panda-driver-topics.git,ee14a1e6f9666bd2b9a391fa30da79fd00ee5e42)
	@echo "Applying patches..."
	$(call apply_patch,robot-descriptions,platform_mesh_package)
	$(call apply_patch,robot-startup,rviz_config)
	$(call apply_patch,robot-body-coupler,joint_reacher_cycle_freq)
	$(call apply_patch,robot-body-coupler,rbc_process_cleanup)
	$(call apply_patch,robot-startup,sim_launch)
	$(call apply_patch,virtual-robot,sim_gripper_move_tolerance)
	@echo "Simulator repositories ready."

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
	/ros2_ws/scripts/build_ws.sh

sim:
	$(call exec_session,/ros2_ws/scripts/start_sim.sh)

sim-teleop:
	$(call exec_session,/ros2_ws/scripts/start_sim.sh teleop $(ARM_VEL) $(ARM_ACC) $(ARM_JERK))

test-all-joints:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/scripts/test_all_joints.sh

test-gripper:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/scripts/test_gripper.sh

test-panda: test-all-joints test-gripper
	@echo "Full Panda test SUCCEEDED"

gello:
	$(call exec_session,/ros2_ws/scripts/start_gello.sh dynamixel $(GELLO_PORT))

gello-fake:
	$(call exec_session,/ros2_ws/scripts/start_gello.sh fake)

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
		/ros2_ws/scripts/test_gello.sh

# Latency measurements (tools/latency.py). Files go to ros2_ws/tools/data/.
LATENCY = docker exec -it -e HOST_UID=$(shell id -u) -e HOST_GID=$(shell id -g) \
	$(CONTAINER_NAME) bash -c "$(ROS_SOURCE) && python3 /ros2_ws/tools/latency.py
DURATION ?= 60

latency-record:
	@test -n "$(REC)" || { echo 'Usage: make latency-record REC=name [DURATION=60]'; exit 1; }
	$(LATENCY) record $(REC) --duration $(DURATION)"

latency-replay:
	@test -n "$(REC)" -a -n "$(RUN)" || { echo 'Usage: make latency-replay REC=name RUN=name'; exit 1; }
	$(LATENCY) replay $(REC) $(RUN)"

latency-analyze:
	@test -n "$(FILES)" || { echo 'Usage: make latency-analyze FILES="name ..."'; exit 1; }
	$(LATENCY) analyze $(FILES)"

stop:
	-docker stop -t 1 $(CONTAINER_NAME)
	@# --rm removal finishes after 'docker stop' returns; wait for it
	@# so an immediate 'make run' does not hit a name conflict.
	@while docker container inspect $(CONTAINER_NAME) >/dev/null 2>&1; do \
		sleep 0.2; \
	done
