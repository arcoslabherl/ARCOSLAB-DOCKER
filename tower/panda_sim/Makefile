# ARCoS-Lab - Franka Panda Simulator
# Dell Precision 5860 Tower / ROS 2 Humble

CONTAINER_NAME = panda-sim-tower
SIM_TAG = arcoslab-panda:sim-humble
REAL_TAG = arcoslab-panda:real-humble

# REAL=1 ('make run REAL=1'): the real robot image (libfranka) and the
# real-time limits libfranka's control thread needs (SCHED_FIFO 99, locked
# memory). The host also needs a PREEMPT_RT kernel.
REAL ?=
TAG = $(if $(filter 1,$(REAL)),$(REAL_TAG),$(SIM_TAG))
RT_LIMITS = $(if $(filter 1,$(REAL)),--ulimit rtprio=99 --ulimit memlock=-1,)

ROS_WS = $(CURDIR)/ros2_ws
ROS_SRC = $(ROS_WS)/src

# GELLO serial port. Empty: 'make gello' finds the GELLO by itself
# (gello_find_port). A path (/dev/ttyUSB0, /dev/serial/by-id/...) forces it.
GELLO_PORT ?=
GELLO_PORT_ARG = $(or $(GELLO_PORT),auto)
# 'make run' shows the host's /dev at /hostdev, read-only, so a GELLO
# plugged in later (or unplugged and plugged back) appears in the
# container too. The cgroup rule lets it open only USB-serial adapters
# (major 188, ttyUSB*: the GELLO's U2D2 is an FTDI).
SERIAL_DEVICES = -v /dev:/hostdev:ro --device-cgroup-rule='c 188:* rmw'
# The container runs as the host user, so it needs the serial ports'
# group (dialout on Ubuntu, uucp on Arch). By name, since the GELLO may
# not be plugged in yet; plus the group of any ttyUSB present now.
SERIAL_GROUPS = $(foreach gid,$(sort $(shell getent group dialout uucp | cut -d: -f3) \
	$(shell stat -Lc %g /dev/ttyUSB* 2>/dev/null)),--group-add $(gid))

# The container runs as the host user, so the files it writes in the repo
# (build/, install/, log/, __pycache__, tools/data/) are the user's, not root's.
HOST_USER = --user $(shell id -u):$(shell id -g) -e HOME=/tmp

# ROS 2 domain of the container. Not the default 0, so this setup doesn't
# see (or command) other ROS computers on the lab's network, nor they it.
# Takes effect on the next 'make run'.
ROS_DOMAIN_ID ?= 47

# Which Panda 'make gello' commands: sim, or real (the lab driver,
# 'make real-driver'; config/panda_real.yaml on top of gello_panda.yaml)
ROBOT ?= sim
# GRIPPER=off: the GELLO leaves the Panda's gripper alone (first real tests)
GRIPPER ?= on
# Joint mask (L4, first GELLO tests on the real Panda): JOINTS=7 or JOINTS=6,7
# follow the GELLO within RANGE degrees of the pose at enable; the rest hold.
JOINTS ?= all
RANGE ?= 10

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

.PHONY: help help-all setup update test test-fake-robot .pull .build .build-real run fix-owner debug .colcon_build sim sim-teleop test-all-joints test-gripper test-panda gello gello-fake gello-enable gello-disable gello-offsets test-gello test-sim test-unit fake-driver-build test-driver test-reflex test-robot-mode test-nudge test-joint-mask real-driver-build rt-check robot-echo robot-comm-test real-driver robot-enable robot-disable robot-nudge latency-record latency-replay latency-analyze stop

# The everyday commands, by situation. Everything else: 'make help-all'.
help:
	@echo "Simulator with the GELLO (one terminal each):"
	@echo "  make run            Start the container (plug the GELLO in any time)"
	@echo "  make sim-teleop     Simulator for GELLO control"
	@echo "  make gello          GELLO teleoperation (make gello-fake: no hardware)"
	@echo "  make gello-enable   The Panda follows the GELLO (match its pose first)"
	@echo "  make gello-disable  The Panda stops where it is"
	@echo "  make stop           Stop the container"
	@echo ""
	@echo "Real Panda, at the lab (in this order; tools/real_robot/README.md):"
	@echo "  make run REAL=1     Container with libfranka (needs a PREEMPT_RT kernel)"
	@echo "  make real-driver-build  Once per machine: the driver, rt_check, nudge"
	@echo "  make rt-check       Real-time checks, no robot needed"
	@echo "  make robot-echo     The robot's state, no motion"
	@echo "  make robot-nudge JOINT=7 DEG=5  One joint a few degrees and back (driver stopped)"
	@echo "  make real-driver    The lab's driver (own terminal)"
	@echo "  make robot-enable   The driver holds the arm's pose"
	@echo "  make robot-nudge VIA=driver JOINT=7 DEG=5  The same, through the driver"
	@echo "  make gello ROBOT=real, then gello-enable / gello-disable as above"
	@echo "  make robot-disable  Only with the GELLO disabled and the arm still"
	@echo ""
	@echo "Setup and tests:"
	@echo "  make setup [REAL=1] First time on a machine: download, build, start, compile"
	@echo "  make update         After 'git pull': update the repositories and compile"
	@echo "  make test           Unit tests + simulator tests, unattended"
	@echo ""
	@echo "Step by step, start to end: TELEOP.md. All commands: make help-all"

help-all:
	@echo "Shortcuts:"
	@echo "  make setup [REAL=1] .pull, .build (+ .build-real), run if stopped, .colcon_build"
	@echo "  make update         .pull and .colcon_build (the container must be running)"
	@echo "  make test [ONLY=rbc|teleop]  test-unit, then test-sim"
	@echo "  make test-fake-robot [PATCHED=1]  fake-driver-build + the five tests (original driver: test-driver fails, known)"
	@echo ""
	@echo "Available commands:"
	@echo "  make .pull          Download simulator repositories"
	@echo "  make .build         Build Docker image"
	@echo "  make .build-real    Build the real robot image (sim + libfranka 0.8.0)"
	@echo "  make run [REAL=1]   Start Docker container (REAL=1: real robot image)"
	@echo "  make fix-owner      Give root-owned files in ros2_ws/ back to the host user"
	@echo "  make debug          Open container terminal"
	@echo "  make .colcon_build  Build ROS 2 workspace"
	@echo "  make sim            Launch Franka Panda simulator (with RBC)"
	@echo "  make test-all-joints  Test all 7 Panda joints"
	@echo "  make test-gripper     Test Panda gripper"
	@echo "  make test-panda       Test all joints and gripper"
	@echo "  make sim-teleop     Start the simulator for GELLO control (no RBC)"
	@echo "  make gello [ROBOT=real] [GRIPPER=off] [JOINTS=7 RANGE=10]  Start GELLO teleoperation (finds the GELLO; GELLO_PORT=... forces a port)"
	@echo "  make gello-fake     Start GELLO with a fake device (no hardware)"
	@echo "  make gello-enable   Enable GELLO control"
	@echo "  make gello-disable  Disable GELLO control"
	@echo "  make gello-offsets GELLO_SIGNS=\"1 -1 ...\"  Calibrate a GELLO unit"
	@echo "  make test-gello     Test the GELLO pipeline (needs sim-teleop + gello)"
	@echo "  make test-sim [ONLY=rbc|teleop]  Start the simulator, run its tests, stop it"
	@echo "  make test-unit      Unit tests of gello_teleop (no simulator needed)"
	@echo "  make latency-record REC=name [DURATION=60]  Record a GELLO session"
	@echo "  make latency-replay REC=name RUN=name       Replay it to the sim"
	@echo "  make latency-analyze FILES=\"name ...\"      Delay, lag and overshoot"
	@echo "  make fake-driver-build  Build the real Panda driver on a fake libfranka"
	@echo "  make test-driver [PATCHED=1]  Test the real driver on the fake libfranka"
	@echo "  make test-reflex [PATCHED=1]  Test a reflex during GELLO teleop (fake robot)"
	@echo "  make test-joint-mask [PATCHED=1]  Test the GELLO joint mask, JOINTS=7 (fake robot)"
	@echo "  make test-robot-mode [PATCHED=1]  Test robot-enable/robot-disable (fake robot)"
	@echo "  make test-nudge [PATCHED=1]  Test robot-nudge, both ways (fake robot)"
	@echo "  make real-driver-build  Build the real driver (plain and patched), rt_check and nudge (REAL=1 container)"
	@echo "  make rt-check       Check libfranka's real-time requirements (REAL=1 container)"
	@echo "  make robot-echo     Print the real Panda's state 100 times (no motion)"
	@echo "  make robot-comm-test  libfranka's communication test (MOVES the arm)"
	@echo "  make robot-nudge JOINT=1-7 DEG=+-10 [VIA=driver | STIFF=1 NOLIMIT=1]  One joint a few degrees and back"
	@echo "  make real-driver [PATCHED=1]  Run the real driver on the real Panda"
	@echo "  make robot-enable   Enable the driver (it holds the arm's pose)"
	@echo "  make robot-disable  Disable the driver (refused while GELLO controls or the arm moves)"
	@echo "  make stop           Stop Docker container"

# No "Entering directory" lines from the shortcuts' sub-makes
MAKEFLAGS += --no-print-directory

# Shortcuts: the commands that always go together, in order. Each step is
# still its own target. REAL=1, ONLY and PATCHED reach the sub-makes.
setup:
	$(MAKE) .pull
	$(MAKE) .build $(if $(filter 1,$(REAL)),.build-real)
	@# A running container keeps its image and options: restart it by hand
	@if docker container inspect $(CONTAINER_NAME) >/dev/null 2>&1; then \
		echo "The container is already running: kept as it is."; \
	else \
		$(MAKE) run; \
	fi
	$(MAKE) .colcon_build

update:
	$(MAKE) .pull
	$(MAKE) .colcon_build

test:
	$(MAKE) test-unit
	$(MAKE) test-sim

# Runs them all even if one fails: the original driver fails test-driver
# (known bugs, tools/fake_franka/README.md); PATCHED=1 should pass them all.
test-fake-robot:
	$(MAKE) fake-driver-build
	@failed=; for t in test-driver test-reflex test-robot-mode test-nudge test-joint-mask; do \
		$(MAKE) $$t || failed="$$failed $$t"; \
	done; \
	if [ -n "$$failed" ]; then echo "test-fake-robot: FAILED:$$failed"; exit 1; fi; \
	echo "test-fake-robot: all passed"

.pull:
	mkdir -p $(ROS_SRC)
	@echo "Downloading simulator repositories at their pinned commits..."
	$(call pull_repo,virtual-robot,$(GITLAB)/arcos-lab-cognitive-architecture/ros2-generic-multi-joint-impedance-control-virtual-robot.git,de0db87781810f7ed8304ebc713f5391c1c74097)
	$(call pull_repo,robot-body-coupler,$(GITLAB)/arcos-lab-cognitive-architecture/robot-body-coupler/robot-body-coupler.git,55a76c2d6687737d63a1d809bc54576e00863699)
	$(call pull_repo,robot-descriptions,$(GITLAB)/robot-hardware-firmware/collaborative-robot/arcos-lab-asimilar-platform-robot-descriptions.git,5a5fecd2f6be4b77cbd23efd9cf1e9193f785395)
	$(call pull_repo,robot-startup,$(GITLAB)/robot-hardware-firmware/collaborative-robot/arcos-lab-collaborative-robot-startup.git,93c9fe6b578cc4e631a3a8501fd7207abefc6540)
	$(call pull_repo,panda-driver-topics,$(GITLAB)/robot-hardware-firmware/drivers/ros2-panda-driver-topics.git,ee14a1e6f9666bd2b9a391fa30da79fd00ee5e42)
	$(call pull_repo,mobile-platform,$(GITLAB)/robot-hardware-firmware/arcos-lab-mobile-platform.git,90fdcf24c4df884e1554db456c3ac1ca00f91df8)
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
	--target=sim \
	--tag=$(SIM_TAG) \
	--network=host \
	.

.build-real:
	docker build \
	--target=real \
	--tag=$(REAL_TAG) \
	--network=host \
	.

run:
	@xhost +si:localuser:$(shell id -un) > /dev/null
	docker run \
	--detach \
	--rm \
	--init \
	--network=host \
	$(HOST_USER) \
	-e DISPLAY=$(DISPLAY) \
	-e QT_X11_NO_MITSHM=1 \
	-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v $(ROS_WS):/ros2_ws:rw \
	$(SERIAL_DEVICES) \
	$(SERIAL_GROUPS) \
	$(RT_LIMITS) \
	--name $(CONTAINER_NAME) \
	$(TAG) \
	bash -c "tail -f /dev/null"

# Once, for files created when the container still ran as root
fix-owner:
	docker run --rm -v $(ROS_WS):/ros2_ws $(TAG) \
		chown -R $(shell id -u):$(shell id -g) /ros2_ws

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
	$(call exec_session,/ros2_ws/scripts/start_gello.sh dynamixel $(GELLO_PORT_ARG) $(ROBOT) $(GRIPPER) $(JOINTS) $(RANGE))

gello-fake:
	$(call exec_session,/ros2_ws/scripts/start_gello.sh fake /dev/null $(ROBOT) $(GRIPPER) $(JOINTS) $(RANGE))

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
		"$(ROS_SOURCE) && port=\$$(ros2 run gello_teleop gello_find_port $(GELLO_PORT_ARG)) && ros2 run gello_teleop gello_get_offset --port \$$port --joint-signs $(GELLO_SIGNS) $(if $(GELLO_START),--start-joints $(GELLO_START),)"

test-gello:
	docker exec -it $(CONTAINER_NAME) bash \
		/ros2_ws/scripts/test_gello.sh

# Unattended: starts what each test needs and stops it afterwards
test-sim:
	$(call exec_session,/ros2_ws/scripts/test_sim.sh $(or $(ONLY),all))

# No cache or bytecode files left in the repo
test-unit:
	docker exec -it -e PYTHONDONTWRITEBYTECODE=1 $(CONTAINER_NAME) bash -c \
		"cd /ros2_ws/gello_teleop && python3 -m pytest -q -p no:cacheprovider test"

# Latency measurements (tools/latency.py). Files go to ros2_ws/tools/data/.
LATENCY = docker exec -it $(CONTAINER_NAME) bash -c "$(ROS_SOURCE) && python3 /ros2_ws/tools/latency.py
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

# The real Panda driver on a fake libfranka (tools/fake_franka/), no robot
# needed. PATCHED=1 uses the driver with patches/panda_driver_safety.patch.
DRIVER_PATCH = panda_driver_safety.patch
FAKE_FRANKA = /ros2_ws/tools/fake_franka

fake-driver-build:
	docker cp patches/$(DRIVER_PATCH) $(CONTAINER_NAME):/tmp/$(DRIVER_PATCH)
	docker exec $(CONTAINER_NAME) bash $(FAKE_FRANKA)/build.sh /tmp/$(DRIVER_PATCH)

test-driver:
	docker exec -it $(CONTAINER_NAME) bash $(FAKE_FRANKA)/test_driver.sh $(if $(PATCHED),--patched)

test-reflex:
	docker exec -it $(CONTAINER_NAME) bash $(FAKE_FRANKA)/test_reflex.sh $(if $(PATCHED),--patched)

test-robot-mode:
	docker exec -it $(CONTAINER_NAME) bash $(FAKE_FRANKA)/test_robot_mode.sh $(if $(PATCHED),--patched)

test-joint-mask:
	docker exec -it $(CONTAINER_NAME) bash $(FAKE_FRANKA)/test_joint_mask.sh $(if $(PATCHED),--patched)

test-nudge:
	docker exec -it $(CONTAINER_NAME) bash $(FAKE_FRANKA)/test_nudge.sh $(if $(PATCHED),--patched)

# The real Panda (tools/real_robot/). Needs the real image: 'make run REAL=1'.
# PATCHED=1 uses the driver with patches/panda_driver_safety.patch.
REAL_ROBOT = /ros2_ws/tools/real_robot
ROBOT_IP ?= 172.16.0.2
ROBOT_NS ?= /asimilar/panda_arm

real-driver-build:
	docker cp patches/$(DRIVER_PATCH) $(CONTAINER_NAME):/tmp/$(DRIVER_PATCH)
	docker exec $(CONTAINER_NAME) bash $(REAL_ROBOT)/build.sh /tmp/$(DRIVER_PATCH)

rt-check:
	docker exec $(CONTAINER_NAME) /ros2_ws/build/_real_robot/rt_check

robot-echo:
	$(call exec_session,$(REAL_ROBOT)/examples.sh echo $(ROBOT_IP))

robot-comm-test:
	$(call exec_session,$(REAL_ROBOT)/examples.sh comm $(ROBOT_IP))

real-driver:
	$(call exec_session,$(REAL_ROBOT)/run_driver.sh $(if $(PATCHED),--patched))

robot-enable:
	$(call exec_session,$(REAL_ROBOT)/robot_mode.sh enable --ns $(ROBOT_NS))

robot-disable:
	$(call exec_session,$(REAL_ROBOT)/robot_mode.sh disable --ns $(ROBOT_NS))

# One joint a few degrees from where it is, and back. VIA=driver: through the
# running, enabled driver (posref); otherwise libfranka alone (driver stopped).
# STIFF=1 (libfranka alone): libfranka examples' joint impedance, not the driver's.
# NOLIMIT=1 (libfranka alone): libfranka's rate limiter off, a jump stops the robot.
robot-nudge:
	@test -n "$(JOINT)" -a -n "$(DEG)" || { echo 'Usage: make robot-nudge JOINT=1-7 DEG=-10..10 [VIA=driver]'; exit 1; }
	@test -z "$(NOLIMIT)" -o "$(VIA)" != driver || { echo 'NOLIMIT=1 is for libfranka alone: the driver always uses the rate limiter.'; exit 1; }
	$(call exec_session,$(REAL_ROBOT)/nudge.sh $(if $(filter driver,$(VIA)),driver $(JOINT) $(DEG) --ns $(ROBOT_NS),libfranka $(ROBOT_IP) $(JOINT) $(DEG) $(if $(STIFF),--stiff) $(if $(NOLIMIT),--no-limit)))

stop:
	-docker stop -t 1 $(CONTAINER_NAME)
	@# --rm removal finishes after 'docker stop' returns; wait for it
	@# so an immediate 'make run' does not hit a name conflict.
	@while docker container inspect $(CONTAINER_NAME) >/dev/null 2>&1; do \
		sleep 0.2; \
	done
