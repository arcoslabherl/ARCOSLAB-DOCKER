#!/usr/bin/env python3
"""Turn the camera mount's servos' torque off (IDs 1 and 2) and check it.
The only thing it writes. For when 'make neck' could not.

Usage: neck_off.py <port> [baud]   (default 57600)
"""

import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

from neck_read import ADDR_TORQUE_ENABLE, BAUD, SERVOS, check_port


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    path, real = check_port(sys.argv[1])
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD
    port, packet = PortHandler(path), PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(baud):
        print(f'ERROR: cannot open {path} at {baud} baud (is \'make neck\' still running?)')
        sys.exit(1)
    failed = False
    try:
        for i, name in SERVOS.items():
            for _ in range(5):
                packet.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)
                value, result, _ = packet.read1ByteTxRx(port, i, ADDR_TORQUE_ENABLE)
                if result == COMM_SUCCESS and value == 0:
                    print(f'ID {i} ({name}): torque off')
                    break
                time.sleep(0.05)
            else:
                print(f'ID {i} ({name}): NO ANSWER or still on: unplug the mount\'s power')
                failed = True
    finally:
        port.closePort()
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
