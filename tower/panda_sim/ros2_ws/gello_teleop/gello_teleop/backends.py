"""GELLO servo backends.

Every backend returns *raw* servo angles in radians, one per servo id,
exactly as a Dynamixel reports them. Converting raw angles to robot joint
angles (offsets, signs, gripper normalization) happens in gello_driver, so
the fake backend exercises the same conversion as the real hardware.

To add a new backend (e.g. a servo simulator), implement read() and close().
"""

import math
import time


# Dynamixel X-series control table (protocol 2.0)
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
PROTOCOL_VERSION = 2.0

# 4096 ticks per revolution
TICKS_TO_RAD = math.pi / 2048.0


class DynamixelBackend:
    """Reads present positions from a chain of Dynamixel servos."""

    def __init__(self, port, baudrate, servo_ids):
        # Imported here so the fake backend works without the SDK.
        from dynamixel_sdk import (
            COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler,
        )

        self._comm_success = COMM_SUCCESS
        self._ids = list(servo_ids)

        self._port = PortHandler(port)
        self._packet = PacketHandler(PROTOCOL_VERSION)

        if not self._port.openPort():
            raise RuntimeError(f"Cannot open GELLO port {port}")

        if not self._port.setBaudRate(baudrate):
            self._port.closePort()
            raise RuntimeError(f"Cannot set baudrate {baudrate} on {port}")

        self._sync_read = GroupSyncRead(
            self._port, self._packet,
            ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION,
        )

        for servo_id in self._ids:
            if not self._sync_read.addParam(servo_id):
                self.close()
                raise RuntimeError(f"Cannot add servo {servo_id} to sync read")

        # GELLO is moved by hand: make sure every servo is passive.
        for servo_id in self._ids:
            result, error = self._packet.write1ByteTxRx(
                self._port, servo_id, ADDR_TORQUE_ENABLE, 0,
            )
            if result != COMM_SUCCESS or error != 0:
                self.close()
                raise RuntimeError(
                    f"Servo {servo_id} not responding on {port} "
                    f"({self._packet.getTxRxResult(result)})"
                )

    def read(self):
        result = self._sync_read.txRxPacket()

        if result != self._comm_success:
            raise IOError(self._packet.getTxRxResult(result))

        angles = []

        for servo_id in self._ids:
            if not self._sync_read.isAvailable(
                servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION,
            ):
                raise IOError(f"No data from servo {servo_id}")

            ticks = self._sync_read.getData(
                servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION,
            )

            # Present position is a signed 32-bit value
            if ticks > 0x7FFFFFFF:
                ticks -= 0x100000000

            angles.append(ticks * TICKS_TO_RAD)

        return angles

    def close(self):
        self._port.closePort()


class FakeBackend:
    """Generates smooth raw servo angles without hardware.

    Joints oscillate around `home` (robot joint angles) and the gripper
    opens and closes. Values are converted back to raw servo angles using
    the configured offsets and signs, so the driver's conversion is tested.
    """

    def __init__(self, rate, home, amplitude, frequency,
                 joint_offsets, joint_signs, gripper_open, gripper_closed):
        self._period = 1.0 / rate
        self._home = list(home)
        self._amplitude = amplitude
        self._frequency = frequency
        self._offsets = list(joint_offsets)
        self._signs = list(joint_signs)
        self._gripper = (gripper_open, gripper_closed)
        self._start = time.monotonic()
        self._next = self._start

    def read(self):
        # Keep a steady rate, like a serial read would
        self._next += self._period
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self._next = time.monotonic()

        t = time.monotonic() - self._start
        phase = 2.0 * math.pi * self._frequency * t

        raw = []
        for i, (home, offset, sign) in enumerate(
            zip(self._home, self._offsets, self._signs)
        ):
            joint = home + self._amplitude * math.sin(phase + i * 0.7)
            raw.append(offset + sign * joint)

        if self._gripper[0] is not None:
            opening = 0.5 - 0.5 * math.cos(phase)
            gripper_open, gripper_closed = self._gripper
            raw.append(gripper_open + opening * (gripper_closed - gripper_open))

        return raw

    def close(self):
        pass
