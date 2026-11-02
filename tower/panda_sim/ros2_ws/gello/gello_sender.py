#!/usr/bin/env python3

import json
import socket
import sys
import time
from pathlib import Path

GELLO_ROOT = Path.home() / "sim_frankaP/gello_software"
sys.path.insert(0, str(GELLO_ROOT))

from gello.agents.gello_agent import GelloAgent, PORT_CONFIG_MAP


UDP_HOST = "127.0.0.1"
UDP_PORT = 15010
GELLO_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO7H2-if00-port0"


def main():
    config = PORT_CONFIG_MAP["/dev/ttyUSB0"]

    print("Connecting to GELLO...")

    agent = GelloAgent(
        port=GELLO_PORT,
        dynamixel_config=config,
    )

    driver = agent._robot._driver

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    last_sample = driver._joint_angles
    sequence = 0
    start = time.monotonic()

    print("Publishing new GELLO readings over UDP...")

    try:
        while True:
            current_sample = driver._joint_angles

            if current_sample is None or current_sample is last_sample:
                time.sleep(0.001)
                continue

            last_sample = current_sample

            sample_time_ns = time.monotonic_ns()
            positions = agent.act({})

            if len(positions) != 8:
                continue

            payload = {
                "seq": sequence,
                "sample_time_ns": sample_time_ns,
                "joints": [float(value) for value in positions[:7]],
                "gripper": float(positions[7]),
            }

            sock.sendto(
                json.dumps(payload).encode("utf-8"),
                (UDP_HOST, UDP_PORT),
            )

            sequence += 1

            if sequence % 100 == 0:
                elapsed = time.monotonic() - start
                frequency = sequence / elapsed
                print(f"Sent: {sequence} | Average: {frequency:.2f} Hz")

    except KeyboardInterrupt:
        print("\nGELLO sender stopped.")

    finally:
        sock.close()


if __name__ == "__main__":
    main()