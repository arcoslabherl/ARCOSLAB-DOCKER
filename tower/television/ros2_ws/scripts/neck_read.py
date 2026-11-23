#!/usr/bin/env python3
"""Read the camera mount's 2 Dynamixel servos (ID 1 yaw, ID 2 pitch).
READ ONLY: never writes to the servos, so nothing moves and the torque
stays as it is.

Prints each servo's settings, then its position while the mount is moved
by hand (torque off), and at the end (Ctrl+C) the range it went through.

Usage: neck_read.py <port> [baud]   (default 57600)
"""

import glob
import os
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

SERVOS = {1: 'yaw', 2: 'pitch'}
BAUD = 57600
OTHER_BAUDS = [1000000, 115200, 2000000, 9600]
# The GELLO's U2D2: its servos are Dynamixels with IDs 1 and 2 too
GELLO_SERIAL = 'FTBEO7H2'

# X series control table (XL330, XL430, XM430...)
ADDR_DRIVE_MODE = 10        # 1 byte, bit 0: reverse direction
ADDR_OPERATING_MODE = 11    # 1 byte
ADDR_HOMING_OFFSET = 20     # 4 bytes, signed
ADDR_MAX_POSITION = 48      # 4 bytes
ADDR_MIN_POSITION = 52      # 4 bytes
ADDR_TORQUE_ENABLE = 64     # 1 byte
ADDR_HARDWARE_ERROR = 70    # 1 byte
ADDR_PROFILE_ACC = 108      # 4 bytes
ADDR_PROFILE_VEL = 112      # 4 bytes
ADDR_PRESENT_POSITION = 132  # 4 bytes, signed
ADDR_PRESENT_VOLTAGE = 144  # 2 bytes, 0.1 V
ADDR_PRESENT_TEMP = 146     # 1 byte, deg C

MODELS = {1190: 'XL330-M077', 1200: 'XL330-M288', 1060: 'XL430-W250',
          1020: 'XM430-W350', 1030: 'XM430-W210'}
MODES = {0: 'current', 1: 'velocity', 3: 'position', 4: 'extended position',
         5: 'current-based position', 16: 'PWM'}
TICKS_PER_REV = 4096
PRINT_STEP_DEG = 2.0
PRINT_EVERY_S = 2.0


def deg(ticks):
    return ticks * 360.0 / TICKS_PER_REV


def signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


def check_port(path):
    """The port's real device; exits if missing or if it is the GELLO's."""
    if path.startswith('/dev/'):
        path = '/hostdev/' + path[len('/dev/'):]
    if not os.path.exists(path):
        print(f'ERROR: {path} not found. Serial adapters seen:')
        for link in sorted(glob.glob('/hostdev/serial/by-id/*')) or ['  (none)']:
            print('  ' + link)
        print("Plug the mount's U2D2 in, or pass NECK_PORT=... "
              "('make stop && make run' if the container is older than this command).")
        sys.exit(1)
    real = os.path.realpath(path)
    for link in glob.glob(f'/hostdev/serial/by-id/*{GELLO_SERIAL}*'):
        if os.path.realpath(link) == real:
            print(f"ERROR: {path} is the GELLO's U2D2 ({GELLO_SERIAL}), not the camera mount's.")
            sys.exit(1)
    return path, real


class Bus:

    def __init__(self, path, baud):
        self.port = PortHandler(path)
        self.packet = PacketHandler(2.0)
        if not self.port.openPort():
            print(f'ERROR: cannot open {path} (permissions? another program using it?)')
            sys.exit(1)
        self.set_baud(baud)

    def set_baud(self, baud):
        if not self.port.setBaudRate(baud):
            print(f'ERROR: cannot set {baud} baud')
            sys.exit(1)
        self.baud = baud

    def ping(self, dxl_id):
        model, result, error = self.packet.ping(self.port, dxl_id)
        return model if result == COMM_SUCCESS else None

    def read(self, dxl_id, addr, size):
        reader = {1: self.packet.read1ByteTxRx, 2: self.packet.read2ByteTxRx,
                  4: self.packet.read4ByteTxRx}[size]
        value, result, error = reader(self.port, dxl_id, addr)
        if result != COMM_SUCCESS:
            return None
        return value

    def close(self):
        self.port.closePort()


def find_servos(bus):
    """IDs that answer at the bus's baud, else at the other usual bauds."""
    for baud in [bus.baud] + OTHER_BAUDS:
        if baud != bus.baud:
            bus.set_baud(baud)
        found = {i: bus.ping(i) for i in SERVOS}
        if any(m is not None for m in found.values()):
            if baud != BAUD:
                print(f'NOTE: the servos answer at {baud} baud, not {BAUD}.')
            return found
    return {}


def describe(bus, dxl_id, model):
    name = SERVOS[dxl_id]
    if model is None:
        print(f'ID {dxl_id} ({name}): NO ANSWER')
        return
    torque = bus.read(dxl_id, ADDR_TORQUE_ENABLE, 1)
    mode = bus.read(dxl_id, ADDR_OPERATING_MODE, 1)
    drive = bus.read(dxl_id, ADDR_DRIVE_MODE, 1)
    homing = bus.read(dxl_id, ADDR_HOMING_OFFSET, 4)
    lo = bus.read(dxl_id, ADDR_MIN_POSITION, 4)
    hi = bus.read(dxl_id, ADDR_MAX_POSITION, 4)
    vel = bus.read(dxl_id, ADDR_PROFILE_VEL, 4)
    acc = bus.read(dxl_id, ADDR_PROFILE_ACC, 4)
    volt = bus.read(dxl_id, ADDR_PRESENT_VOLTAGE, 2)
    temp = bus.read(dxl_id, ADDR_PRESENT_TEMP, 1)
    err = bus.read(dxl_id, ADDR_HARDWARE_ERROR, 1)
    pos = bus.read(dxl_id, ADDR_PRESENT_POSITION, 4)

    def show(value, fmt=str):
        return '?' if value is None else fmt(value)

    print(f'ID {dxl_id} ({name}): {MODELS.get(model, "model")} ({model})')
    print(f'  torque {show(torque, lambda t: "ON (holding)" if t else "off")}, '
          f'mode {show(mode, lambda m: MODES.get(m, str(m)))}, '
          f'direction {show(drive, lambda d: "reversed" if d & 1 else "normal")}, '
          f'homing offset {show(homing, lambda h: str(signed32(h)))} ticks')
    print(f'  position limits {show(lo)}..{show(hi)} ticks '
          f'({show(lo, lambda v: f"{deg(v):.1f}")}..{show(hi, lambda v: f"{deg(v):.1f}")} deg), '
          f'profile velocity {show(vel)}, acceleration {show(acc)}')
    print(f'  {show(volt, lambda v: f"{v / 10:.1f}")} V, {show(temp)} C, '
          f'hardware error {show(err, lambda e: "none" if e == 0 else f"0x{e:02x}")}, '
          f'position now {show(pos, lambda p: f"{signed32(p)} ticks ({deg(signed32(p)):.1f} deg)")}')


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    path, real = check_port(sys.argv[1])
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD
    print(f'Port {path} (-> {real}), {baud} baud. READ ONLY: nothing is written to the servos.')
    bus = Bus(path, baud)
    try:
        found = find_servos(bus)
        if not found:
            print('ERROR: no servo answers (IDs 1, 2) at any usual baud. '
                  'Is the mount powered? Right U2D2?')
            sys.exit(1)
        print()
        for dxl_id, model in found.items():
            describe(bus, dxl_id, model)
        ids = [i for i, m in found.items() if m is not None]

        print()
        print('Move the mount gently by hand, only if the torque is off, to each end and back. '
              'Ctrl+C to stop.')
        seen = {i: None for i in ids}
        printed, printed_at = None, 0.0
        while True:
            now_pos = {}
            for i in ids:
                p = bus.read(i, ADDR_PRESENT_POSITION, 4)
                if p is not None:
                    p = signed32(p)
                    now_pos[i] = p
                    lo, hi = seen[i] or (p, p)
                    seen[i] = (min(lo, p), max(hi, p))
            moved = printed is None or any(
                abs(deg(now_pos.get(i, 0) - printed.get(i, 0))) >= PRINT_STEP_DEG for i in ids)
            if now_pos and (moved or time.monotonic() - printed_at >= PRINT_EVERY_S):
                print('  '.join(f'{SERVOS[i]} {p:5d} ticks ({deg(p):6.1f} deg)'
                                for i, p in now_pos.items()), flush=True)
                printed, printed_at = now_pos, time.monotonic()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
        print('Range seen:')
        for i, r in seen.items():
            if r is not None:
                print(f'  {SERVOS[i]}: {r[0]}..{r[1]} ticks ({deg(r[0]):.1f}..{deg(r[1]):.1f} deg, '
                      f'{deg(r[1] - r[0]):.1f} deg wide)')
    finally:
        bus.close()


if __name__ == '__main__':
    main()
