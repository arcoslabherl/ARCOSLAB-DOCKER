"""Fake Dynamixel X servos on a pseudo-terminal, for tests without the
mount: answers ping, read and write (protocol 2.0). With torque on, the
position moves toward the goal at the profile velocity. Records every
write, so a test can check what was sent.
"""

import math
import os
import struct
import threading
import time
import tty

from dynamixel_sdk.protocol2_packet_handler import Protocol2PacketHandler

MODEL_XL330_M288 = 1200
UNIT_DEG_S = 0.229 * 360.0 / 60.0
TICKS_PER_DEG = 4096 / 360.0

_crc = Protocol2PacketHandler().updateCRC


class FakeServos:

    def __init__(self, positions, link='/tmp/fake_u2d2', latency=0.0):
        """positions: {id: ticks}. The port is at 'link'. latency: seconds
        before each answer (a real bus at 57600 takes a few ms)."""
        self.latency = latency
        self.master, slave = os.openpty()
        tty.setraw(slave)
        self.slave = slave
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(os.ttyname(slave), link)
        self.link = link
        self.lock = threading.Lock()
        self.mem = {}
        self.pos = {}
        self.writes = []      # (time, id, addr, value)
        for i, p in positions.items():
            m = bytearray(256)
            m[11] = 3                                    # position mode
            struct.pack_into('<I', m, 48, 4095)
            struct.pack_into('<H', m, 144, 52)
            m[146] = 25
            self.mem[i] = m
            self.pos[i] = float(p)
        self.running = True
        self.last = time.monotonic()
        threading.Thread(target=self._loop, daemon=True).start()

    def torque(self, i):
        with self.lock:
            return self.mem[i][64]

    def position(self, i):
        with self.lock:
            return self.pos[i]

    def push(self, i, ticks):
        """Move a servo from outside (someone holding the camera)."""
        with self.lock:
            self.pos[i] = float(ticks)

    def _advance(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        for i, m in self.mem.items():
            if m[64]:
                goal = struct.unpack_from('<i', m, 116)[0]
                vel = struct.unpack_from('<I', m, 112)[0]
                step = vel * UNIT_DEG_S * TICKS_PER_DEG * dt if vel else math.inf
                diff = goal - self.pos[i]
                self.pos[i] += max(-step, min(step, diff))
            struct.pack_into('<i', m, 132, int(round(self.pos[i])))

    def _reply(self, i, params=b''):
        if self.latency:
            time.sleep(self.latency)
        body = bytes([0xFF, 0xFF, 0xFD, 0x00, i]) + struct.pack('<H', len(params) + 4) + bytes([0x55, 0]) + params
        os.write(self.master, body + struct.pack('<H', _crc(0, body, len(body))))

    def _loop(self):
        buf = b''
        while self.running:
            try:
                buf += os.read(self.master, 256)
            except OSError:
                return
            while True:
                k = buf.find(b'\xff\xff\xfd\x00')
                if k < 0 or len(buf) < k + 7:
                    break
                length = struct.unpack_from('<H', buf, k + 5)[0]
                if len(buf) < k + 7 + length:
                    break
                pkt, buf = buf[k:k + 7 + length], buf[k + 7 + length:]
                i, inst, par = pkt[4], pkt[7], pkt[8:-2]
                if i not in self.mem:
                    continue
                with self.lock:
                    self._advance()
                    m = self.mem[i]
                    if inst == 0x01:
                        self._reply(i, struct.pack('<HB', MODEL_XL330_M288, 52))
                    elif inst == 0x02:
                        addr, n = struct.unpack('<HH', par)
                        self._reply(i, bytes(m[addr:addr + n]))
                    elif inst == 0x03:
                        addr = struct.unpack_from('<H', par)[0]
                        data = par[2:]
                        m[addr:addr + len(data)] = data
                        value = int.from_bytes(data, 'little', signed=(len(data) == 4))
                        self.writes.append((time.monotonic(), i, addr, value))
                        self._reply(i)

    def close(self):
        self.running = False
        os.close(self.slave)
        os.close(self.master)
        if os.path.lexists(self.link):
            os.remove(self.link)
