# ARCoS-Lab - TeleVision: ZED stereo video to a WebXR headset (Apple Vision Pro)
# ROS 2 Humble, ZED SDK. Any Linux machine with an NVIDIA GPU.

CONTAINER_NAME = television
TAG = arcoslab-television:humble

ROS_WS = $(CURDIR)/ros2_ws
# Certificates for HTTPS ('make cert') and the ZED's calibration files
# (downloaded by the SDK the first time a camera is opened). Gitignored.
CERTS = $(CURDIR)/certs
ZED_SETTINGS = $(CURDIR)/zed_settings

# The machine's names and addresses for the certificate: the headset must
# open the page with one of them. HOSTS="..." replaces the list.
HOST_IPS = $(shell ip -o -4 addr show scope global 2>/dev/null | grep -v -E ' (docker|br-|veth)' | awk '{split($$4,a,"/"); print a[1]}')
TAILSCALE_NAME = $(shell tailscale status --json 2>/dev/null | python3 -c "import json,sys; print(json.load(sys.stdin)['Self']['DNSName'].rstrip('.'))" 2>/dev/null)
HOSTS ?= localhost 127.0.0.1 $(shell hostname) $(HOST_IPS) $(TAILSCALE_NAME)

# GPU: needs the NVIDIA Container Toolkit (README). Without it only the
# test pattern works ('make stream-fake').
GPU = $(if $(shell docker info 2>/dev/null | grep -i 'runtimes:.*nvidia'),--gpus all,)
# The image's CUDA 12.8 needs driver >= 570 (Dockerfile)
NVIDIA_DRIVER = $(shell nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)
MIN_DRIVER = 570

# ZED: its video (/dev/video*), sensors (/dev/hidraw*) and USB bus, as
# present at 'make run': plug the ZED in first. Major 189: USB devices.
ZED_DEVICES = $(foreach dev,$(wildcard /dev/video* /dev/hidraw*),--device $(dev)) \
	-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw'
ZED_PLUGGED = $(shell lsusb -d 2b03: 2>/dev/null)
# The container runs as the host user: the video group opens /dev/video*
VIDEO_GROUPS = $(foreach gid,$(sort $(shell getent group video | cut -d: -f3) \
	$(shell stat -Lc %g /dev/video* 2>/dev/null)),--group-add $(gid))
# The camera mount's servos: the host's /dev at /hostdev, read-only, so
# a U2D2 plugged in later shows up; the cgroup rule opens only USB-serial
# adapters (major 188, ttyUSB*). Their group: dialout (Ubuntu), uucp (Arch).
SERIAL_DEVICES = -v /dev:/hostdev:ro --device-cgroup-rule='c 188:* rmw'
SERIAL_GROUPS = $(foreach gid,$(sort $(shell getent group dialout uucp | cut -d: -f3) \
	$(shell stat -Lc %g /dev/ttyUSB* 2>/dev/null)),--group-add $(gid))
# The mount's U2D2 (not the GELLO's, FTBEO7H2)
NECK_PORT ?= /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO5IA-if00-port0
# The zed group (in the image) reads the SDK in /usr/local/zed
HOST_USER = --user $(shell id -u):$(shell id -g) -e HOME=/tmp --group-add zed

ROS_SOURCE = source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash

# Run a long-lived script in the container. 'docker exec' does not stop
# it when the terminal is closed, so stop its process group whenever this
# command ends (Ctrl+C, closed terminal, error).
define exec_session
	@bash -c 'trap "docker exec $(CONTAINER_NAME) bash /ros2_ws/scripts/stop_session.sh $$$$ >/dev/null 2>&1" EXIT; \
		trap "exit 129" HUP; trap "exit 130" INT; trap "exit 143" TERM; \
		docker exec -it -e SESSION_ID=$$$$ $(CONTAINER_NAME) bash $(1)'
endef

.PHONY: help setup update .build run debug .colcon_build cert cert-tailscale udev stream stream-fake head-echo neck-read neck test-neck test-unit test-stream stop

help:
	@echo "Video to the headset (README.md):"
	@echo "  make run            Start the container (plug the ZED in first)"
	@echo "  make stream         ZED video to the headset"
	@echo "  make stream-fake    Test pattern instead of the ZED (no camera, no GPU)"
	@echo "  make head-echo      Head pose in degrees and Hz while a headset is connected"
	@echo "  make neck-read      Read the camera mount's servos (read only, nothing moves)"
	@echo "  make neck [LIVE=1] [RANGE=3]  The mount follows the head (dry run unless LIVE=1)"
	@echo "  make stop           Stop the container"
	@echo ""
	@echo "Setup and tests:"
	@echo "  make setup          First time on a machine: build, start, compile"
	@echo "  make cert-tailscale  HTTPS certificate from Tailscale (every 90 days, sudo)"
	@echo "  make cert [HOSTS=\"...\"]  Or: own CA, installed on the headset (once per machine)"
	@echo "  make udev           Once per machine (sudo): let users open the ZED"
	@echo "  make update         After 'git pull': compile"
	@echo "  make test-unit      Unit tests (no camera)"
	@echo "  make test-stream    Fake headset against a running stream (make stream-fake first)"
	@echo "  make test-neck      The neck against fake servos and a fake head (no hardware)"
	@echo ""
	@echo "Other: make .build, .colcon_build, debug"

# No "Entering directory" lines from the shortcuts' sub-makes
MAKEFLAGS += --no-print-directory

setup:
	$(MAKE) .build
	@# A running container keeps its image and options: restart it by hand
	@if docker container inspect $(CONTAINER_NAME) >/dev/null 2>&1; then \
		echo "The container is already running: kept as it is."; \
	else \
		$(MAKE) run; \
	fi
	$(MAKE) .colcon_build

update:
	$(MAKE) .colcon_build

.build:
	docker build \
	--tag=$(TAG) \
	--network=host \
	.

run:
	@mkdir -p $(CERTS) $(ZED_SETTINGS)
	@driver="$(NVIDIA_DRIVER)"; \
	if [ -n "$$driver" ] && [ "$${driver%%.*}" -lt $(MIN_DRIVER) ]; then \
		echo "WARNING: NVIDIA driver $$driver: the ZED SDK's CUDA 12.8 needs >= $(MIN_DRIVER). Docker will refuse the GPU."; \
	fi
	@if [ -z "$(GPU)" ]; then \
		echo "WARNING: Docker has no NVIDIA runtime: no GPU, so no ZED. Only 'make stream-fake' works."; \
		echo "         Install the NVIDIA Container Toolkit (README.md)."; \
	fi
	@if [ -z "$(ZED_PLUGGED)" ]; then \
		echo "WARNING: no ZED plugged in. To use it: plug it in, then 'make stop && make run'."; \
	fi
	docker run \
	--detach \
	--rm \
	--init \
	--network=host \
	$(HOST_USER) \
	$(GPU) \
	$(ZED_DEVICES) \
	$(VIDEO_GROUPS) \
	$(SERIAL_DEVICES) \
	$(SERIAL_GROUPS) \
	-v $(ROS_WS):/ros2_ws:rw \
	-v $(CERTS):/television/certs:rw \
	-v $(ZED_SETTINGS):/usr/local/zed/settings:rw \
	--name $(CONTAINER_NAME) \
	$(TAG) \
	bash -c "tail -f /dev/null"

debug:
	docker exec -it $(CONTAINER_NAME) bash -c \
	  "$(ROS_SOURCE) && exec bash"

.colcon_build:
	docker exec -it $(CONTAINER_NAME) bash \
	/ros2_ws/scripts/build_ws.sh

# HTTPS certificate for the headset (WebXR needs HTTPS), signed by a CA
# of this machine (certs/ca/). Install certs/ca/rootCA.pem on the headset
# once (README.md). Rerun when the machine's address changes.
cert:
	@mkdir -p $(CERTS)/ca
	docker run --rm $(HOST_USER) -e CAROOT=/certs/ca -v $(CERTS):/certs $(TAG) \
		mkcert -cert-file /certs/cert.pem -key-file /certs/key.pem $(sort $(HOSTS))
	@echo "Install on the headset (once): $(CERTS)/ca/rootCA.pem"

# Or a Let's Encrypt certificate from Tailscale for this machine's
# Tailscale name: nothing to install on the headset, which must be in the
# same tailnet. Needs HTTPS enabled in the tailnet's admin console. It
# expires after 90 days: rerun.
cert-tailscale:
	@if [ -z "$(TAILSCALE_NAME)" ]; then \
		echo "ERROR: Tailscale is not running or not logged in here ('tailscale status')."; \
		exit 1; \
	fi
	@mkdir -p $(CERTS)
	sudo tailscale cert --cert-file $(CERTS)/cert.pem --key-file $(CERTS)/key.pem $(TAILSCALE_NAME)
	sudo chown $(shell id -u):$(shell id -g) $(CERTS)/cert.pem $(CERTS)/key.pem
	@echo "On the headset (Tailscale on): https://$(TAILSCALE_NAME):8012"

# The ZED SDK's udev rules: users (and the container) may open the
# ZED's sensors. From the image, so they match its SDK.
udev:
	docker run --rm $(TAG) cat /etc/udev/rules.d/99-slabs.rules > /tmp/99-slabs.rules
	sudo install -m 644 /tmp/99-slabs.rules /etc/udev/rules.d/99-slabs.rules
	rm -f /tmp/99-slabs.rules
	sudo udevadm control --reload-rules && sudo udevadm trigger
	@echo "Done. Unplug and plug the ZED back in."

stream:
	$(call exec_session,/ros2_ws/scripts/start_stream.sh zed)

stream-fake:
	$(call exec_session,/ros2_ws/scripts/start_stream.sh fake)

head-echo:
	docker exec -it $(CONTAINER_NAME) bash -c \
		"$(ROS_SOURCE) && python3 /ros2_ws/scripts/head_echo.py"

neck-read:
	docker exec -it $(CONTAINER_NAME) python3 /ros2_ws/scripts/neck_read.py $(NECK_PORT)

# The mount follows the head. Dry run unless LIVE=1. RANGE: deg from neutral.
LIVE ?= 0
RANGE ?=
neck:
	$(call exec_session,/ros2_ws/scripts/start_neck.sh $(LIVE) "$(RANGE)" $(NECK_PORT))

test-neck:
	docker exec -it $(CONTAINER_NAME) bash -c \
		"$(ROS_SOURCE) && python3 /ros2_ws/scripts/test_neck.py"

test-unit:
	docker exec -it -e PYTHONDONTWRITEBYTECODE=1 $(CONTAINER_NAME) bash -c \
		"cd /ros2_ws/television_teleop && python3 -m pytest -q -p no:cacheprovider test"

# Like a headset: both eyes' images arrive, the head pose reaches ROS
test-stream:
	docker exec -it $(CONTAINER_NAME) bash -c \
		"$(ROS_SOURCE) && python3 /ros2_ws/scripts/test_stream.py \
		$(if $(wildcard $(CERTS)/cert.pem),wss,ws)://localhost:8012"

stop:
	docker stop $(CONTAINER_NAME)
