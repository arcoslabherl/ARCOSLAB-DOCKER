"""Active neck node: the camera mount's servos follow /television/head_pose.

Dry run by default (live: false): reads the servos and prints the goals
it would send, writes nothing. Live: torque on, goals at rate_hz, within
range_deg of neutral and the mount's safe ends, at max_speed_deg_s.

Stops (torque off, node exits) when: no head pose for head_timeout_s, a
servo is more than max_error_deg from its goal, a hardware error, or the
bus fails. At start it refuses unless the camera is at neutral (by hand).
"""

import math
import os
import time

import rclpy
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from television_teleop.neck import Axis, Follower, deg, head_yaw_pitch

# X series control table (XL330)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_PROFILE_ACC = 108
ADDR_PROFILE_VEL = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
MODE_POSITION = 3
PROFILE_VEL_UNIT_DEG_S = 0.229 * 360.0 / 60.0     # 0.229 rev/min
PROFILE_ACC_UNIT_DEG_S2 = 214.577 * 360.0 / 3600.0  # 214.577 rev/min^2
# The GELLO's U2D2: its servos have IDs 1 and 2 too
GELLO_SERIAL = 'FTBEO7H2'


class Stop(Exception):
    """A reason to turn the torque off and exit."""


def signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


class Bus:

    def __init__(self, path, baud):
        self.port = PortHandler(path)
        self.packet = PacketHandler(2.0)
        if not self.port.openPort():
            raise Stop(f'cannot open {path}')
        if not self.port.setBaudRate(baud):
            raise Stop(f'cannot set {baud} baud')

    def _check(self, what, result, error):
        if result != COMM_SUCCESS:
            raise Stop(f'{what}: {self.packet.getTxRxResult(result)}')
        if error:
            raise Stop(f'{what}: {self.packet.getRxPacketError(error)}')

    def read(self, servo_id, addr, size):
        reader = {1: self.packet.read1ByteTxRx, 4: self.packet.read4ByteTxRx}[size]
        value, result, error = reader(self.port, servo_id, addr)
        self._check(f'read {addr} from ID {servo_id}', result, error)
        return value

    def write(self, servo_id, addr, size, value):
        writer = {1: self.packet.write1ByteTxRx, 4: self.packet.write4ByteTxRx}[size]
        result, error = writer(self.port, servo_id, addr, value)
        self._check(f'write {addr} to ID {servo_id}', result, error)

    def position(self, servo_id):
        return signed32(self.read(servo_id, ADDR_PRESENT_POSITION, 4))

    def close(self):
        self.port.closePort()


class NeckNode(Node):

    def __init__(self):
        super().__init__('television_neck')
        p = self.declare_parameter
        self.port_path = p('port', '').value
        self.baud = int(p('baud', 57600).value)
        self.live = bool(p('live', False).value)
        range_deg = float(p('range_deg', 3.0).value)
        self.max_speed = float(p('max_speed_deg_s', 4.0).value)
        self.rate_hz = float(p('rate_hz', 20.0).value)
        self.start_tolerance = float(p('start_tolerance_deg', 5.0).value)
        self.head_timeout = float(p('head_timeout_s', 2.0).value)
        self.max_error = float(p('max_error_deg', 5.0).value)
        self.axes = {}
        for name in ('yaw', 'pitch'):
            self.axes[name] = Axis(
                name, int(p(f'{name}_id', 0).value),
                float(p(f'{name}_neutral_deg', 0.0).value),
                float(p(f'{name}_min_deg', 0.0).value),
                float(p(f'{name}_max_deg', 0.0).value),
                int(p(f'{name}_sign', 1).value), range_deg)
        self.follower = Follower(self.axes['yaw'], self.axes['pitch'], self.max_speed, self.rate_hz)

        self.check_port()
        self.bus = Bus(self.port_path, self.baud)
        self.torque = False
        try:
            self.present = self.check_start()
            self.follower.start(self.present)
            if self.live:
                self.prepare_live()
        except BaseException:
            self.torque_off()
            self.bus.close()
            raise
        for a in self.axes.values():
            self.get_logger().info(
                f'{a.name} (ID {a.id}): {deg(a.low):.1f}..{deg(a.high):.1f} deg, '
                f'neutral {deg(a.neutral):.1f}, now {deg(self.present[a.name]):.1f}')
        self.get_logger().info(
            ('LIVE: torque on, the servos follow the head' if self.live else
             'DRY RUN: nothing is written to the servos') +
            f' (range {range_deg:g} deg, {self.max_speed:g} deg/s). '
            'The head pose at the first message is the reference.')

        self.head = None          # (yaw, pitch, monotonic time)
        self.head_seen = False
        self.started_at = time.monotonic()
        self.over_error_since = {}
        self.last_log = 0.0
        self.last_error_check = 0.0
        self.stopped = None
        self.create_subscription(PoseStamped, 'television/head_pose', self.on_head, 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)

    # Start

    def check_port(self):
        if not os.path.exists(self.port_path):
            raise Stop(f'{self.port_path} not found (U2D2 plugged in? NECK_PORT=...)')
        real = os.path.realpath(self.port_path)
        folder = os.path.dirname(self.port_path) if '/serial/by-id/' in self.port_path \
            else '/hostdev/serial/by-id'
        if GELLO_SERIAL in self.port_path or any(
                GELLO_SERIAL in n and os.path.realpath(os.path.join(folder, n)) == real
                for n in (os.listdir(folder) if os.path.isdir(folder) else [])):
            raise Stop(f"{self.port_path} is the GELLO's U2D2, not the camera mount's")

    def check_start(self):
        """Present positions; refuses unless in position mode and at neutral."""
        present = {}
        for a in self.axes.values():
            mode = self.bus.read(a.id, ADDR_OPERATING_MODE, 1)
            if mode != MODE_POSITION:
                raise Stop(f'{a.name} (ID {a.id}) is in operating mode {mode}, not position (3)')
            if self.bus.read(a.id, ADDR_TORQUE_ENABLE, 1):
                raise Stop(f'{a.name} (ID {a.id}) has its torque on: another program is '
                           'driving it? Restart the mount\'s power if not.')
            error = self.bus.read(a.id, ADDR_HARDWARE_ERROR, 1)
            if error:
                raise Stop(f'{a.name} (ID {a.id}) hardware error 0x{error:02x}: power-cycle it')
            pos = self.bus.position(a.id)
            off = deg(pos - a.neutral)
            if abs(off) > self.start_tolerance:
                raise Stop(f'{a.name} (ID {a.id}) at {deg(pos):.1f} deg, {off:+.1f} from neutral '
                           f'{deg(a.neutral):.1f} (more than {self.start_tolerance:g}). '
                           'Move the camera to neutral by hand (make neck-read shows it).')
            present[a.name] = pos
        return present

    def prepare_live(self):
        """Servo-side speed limit (twice ours, a backstop), hold, torque on."""
        vel = max(1, round(2.0 * self.max_speed / PROFILE_VEL_UNIT_DEG_S))
        acc = max(1, round(2.0 * self.max_speed * 10.0 / PROFILE_ACC_UNIT_DEG_S2))
        for a in self.axes.values():
            self.bus.write(a.id, ADDR_PROFILE_ACC, 4, acc)
            self.bus.write(a.id, ADDR_PROFILE_VEL, 4, vel)
            # Goal = where it is, so torque on does not move it
            self.bus.write(a.id, ADDR_GOAL_POSITION, 4, self.present[a.name] & 0xFFFFFFFF)
        self.torque = True
        for a in self.axes.values():
            self.bus.write(a.id, ADDR_TORQUE_ENABLE, 1, 1)

    # Loop

    def on_head(self, msg):
        q = msg.pose.orientation
        yaw, pitch = head_yaw_pitch(q.x, q.y, q.z, q.w)
        if not (math.isfinite(yaw) and math.isfinite(pitch)):
            return
        self.head = (yaw, pitch, time.monotonic())

    def tick(self):
        if self.stopped:
            return
        try:
            self.step()
        except Stop as e:
            self.stop(str(e))

    def step(self):
        now = time.monotonic()
        if self.head is None:
            if now - self.last_log > 5.0:
                self.get_logger().info('Waiting for /television/head_pose (make stream, headset in VR)...')
                self.last_log = now
            return
        yaw, pitch, stamp = self.head
        if now - stamp > self.head_timeout:
            raise Stop(f'no head pose for {self.head_timeout:g} s')
        self.head_seen = True

        goal = self.follower.update(yaw, pitch)
        for a in self.axes.values():
            if self.live:
                self.bus.write(a.id, ADDR_GOAL_POSITION, 4, goal[a.name])
            self.present[a.name] = self.bus.position(a.id)
            err = abs(deg(self.present[a.name] - goal[a.name]))
            # Dry run: nothing is sent, so the servos never reach the goals
            if self.live and err > self.max_error:
                since = self.over_error_since.setdefault(a.name, now)
                if now - since > 0.5:
                    raise Stop(f'{a.name} is {err:.1f} deg from its goal (stuck or pushed?)')
            else:
                self.over_error_since.pop(a.name, None)

        if now - self.last_error_check > 0.5:
            self.last_error_check = now
            for a in self.axes.values():
                error = self.bus.read(a.id, ADDR_HARDWARE_ERROR, 1)
                if error:
                    raise Stop(f'{a.name} hardware error 0x{error:02x}')

        if now - self.last_log > 0.5:
            self.last_log = now
            ref = self.follower.reference
            print(f'head yaw {yaw - ref[0]:+6.1f} pitch {pitch - ref[1]:+6.1f} deg | ' +
                  '  '.join(f'{a.name} goal {deg(goal[a.name] - a.neutral):+5.1f} '
                            f'now {deg(self.present[a.name] - a.neutral):+5.1f}'
                            for a in self.axes.values()) +
                  ' deg from neutral' + ('' if self.live else '  (dry run)'), flush=True)

    # Stop

    def stop(self, reason):
        self.stopped = reason
        self.get_logger().error(f'STOP: {reason}. Torque off.' if self.live else f'STOP: {reason}.')
        self.torque_off()
        raise SystemExit(1)

    def torque_off(self):
        if not self.torque:
            return
        for a in self.axes.values():
            for _ in range(3):
                try:
                    self.bus.write(a.id, ADDR_TORQUE_ENABLE, 1, 0)
                    break
                except Stop:
                    time.sleep(0.01)
            else:
                self.get_logger().error(f'Could not turn {a.name} torque off: unplug the mount\'s power')
        self.torque = False

    def destroy_node(self):
        self.torque_off()
        self.bus.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    code = 0
    try:
        node = NeckNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Stop as e:
        print(f'ERROR: {e}', flush=True)
        code = 1
    except SystemExit as e:
        code = e.code or 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)
