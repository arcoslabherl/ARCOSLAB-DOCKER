#!/usr/bin/env python3
"""Print the head pose (/television/head_pose) in degrees and meters, 5
times a second, with the rate it arrives at. ROS: X forward, Y left, Z up;
yaw + = head turned left, pitch + = looking down, roll + = tilted right.
"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation

PRINT_PERIOD = 0.2


class HeadEcho(Node):

    def __init__(self):
        super().__init__('television_head_echo')
        self.last = None
        self.count = 0
        self.since = time.monotonic()
        self.create_subscription(PoseStamped, 'television/head_pose', self.on_pose, 10)
        self.create_timer(PRINT_PERIOD, self.show)
        print('Waiting for /television/head_pose (headset in VR on the page)...', flush=True)

    def on_pose(self, msg):
        self.last = msg
        self.count += 1

    def show(self):
        now = time.monotonic()
        rate = self.count / (now - self.since)
        self.count, self.since = 0, now
        if self.last is None:
            return
        q, p = self.last.pose.orientation, self.last.pose.position
        yaw, pitch, roll = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX', degrees=True)
        stale = '  (no new pose)' if rate == 0 else ''
        print(f'yaw {yaw:+7.1f}  pitch {pitch:+7.1f}  roll {roll:+7.1f} deg   '
              f'x {p.x:+.2f}  y {p.y:+.2f}  z {p.z:+.2f} m   {rate:4.0f} Hz{stale}', flush=True)


def main():
    rclpy.init()
    node = HeadEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
