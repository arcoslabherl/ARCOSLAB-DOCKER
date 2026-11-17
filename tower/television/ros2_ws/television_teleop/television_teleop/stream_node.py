"""Stereo video to the headset through Vuer (WebXR), and the headset's
head pose back as a ROS topic.

Three threads: the camera grabs pairs, Vuer serves the page and sends
the latest pair to each connected headset, ROS publishes the head pose.
Based on Open-TeleVision's TeleVision.py (image mode) and the lab's
teleop_active_noservo.py; the viewer layout is the one the lab tested on
the Apple Vision Pro.
"""

import asyncio
import os
import tempfile
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from television_teleop.head import matrix_from_event, quaternion_xyzw, yup_to_zup
from television_teleop.sources import open_source


class Stats:
    """Counts events per second, for the periodic log line."""

    def __init__(self):
        self.lock = threading.Lock()
        self.counts = {}

    def add(self, name):
        with self.lock:
            self.counts[name] = self.counts.get(name, 0) + 1

    def take(self):
        with self.lock:
            counts, self.counts = self.counts, {}
        return counts


class StreamNode(Node):

    def __init__(self):
        super().__init__('television_stream')
        self.source_name = self.declare_parameter('source', 'zed').value
        self.resolution = self.declare_parameter('resolution', 'HD720').value
        self.fps = float(self.declare_parameter('fps', 60.0).value)
        self.stream_hz = float(self.declare_parameter('stream_hz', 30.0).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 80).value)
        self.row_step = int(self.declare_parameter('row_step', 2).value)
        self.port = int(self.declare_parameter('port', 8012).value)
        self.cert = self.declare_parameter('cert', '').value
        self.key = self.declare_parameter('key', '').value
        self.frame_id = self.declare_parameter('frame_id', 'television_world').value

        self.stats = Stats()
        self.frame = None        # latest (left, right, sequence number)
        self.head = None         # latest (matrix, monotonic time)
        self.head_published = None
        self.sessions = 0
        self.running = True

        self.source = open_source(self.source_name, self.resolution, self.fps)
        self.get_logger().info(
            f'Source: {self.source_name}, {self.source.width}x{self.source.height} '
            f'per eye. Streaming at up to {self.stream_hz:g} Hz, '
            f'every {self.row_step} row(s), JPEG {self.jpeg_quality}.')

        self.head_pub = self.create_publisher(PoseStamped, 'television/head_pose', 10)
        self.create_timer(1.0 / 60.0, self.publish_head)
        self.create_timer(5.0, self.log_stats)

        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.camera_thread.start()
        threading.Thread(target=self.vuer_thread, daemon=True).start()

    # Camera

    def camera_loop(self):
        seq = 0
        while self.running:
            try:
                pair = self.source.grab()
            except Exception as e:  # keep the node up; log and retry
                self.get_logger().error(f'Camera grab failed: {e}', throttle_duration_sec=2.0)
                time.sleep(0.1)
                continue
            if pair is None:
                continue
            seq += 1
            left, right = pair
            if self.row_step > 1:
                left, right = left[::self.row_step], right[::self.row_step]
            self.frame = (np.ascontiguousarray(left), np.ascontiguousarray(right), seq)
            self.stats.add('camera')

    # Vuer (own event loop, in this thread)

    def vuer_thread(self):
        asyncio.set_event_loop(asyncio.new_event_loop())
        from television_teleop.vuer_server import TeleVisionVuer  # after the loop exists

        https = bool(self.cert and self.key)
        if https and not (os.path.isfile(self.cert) and os.path.isfile(self.key)):
            self.get_logger().error(
                f'Certificate not found ({self.cert}, {self.key}). '
                "Create it with 'make cert'. Serving plain HTTP: "
                'only a browser on this machine (localhost) will work.')
            https = False
        kwargs = dict(cert=self.cert, key=self.key) if https else {}
        # static_root: Vuer serves it at /static; an empty folder, not the cwd
        app = TeleVisionVuer(host='0.0.0.0', port=self.port, queries=dict(grid=False),
                             queue_len=3, static_root=tempfile.mkdtemp(prefix='vuer_static_'),
                             **kwargs)
        self.app = app

        app.add_handler('CAMERA_MOVE')(self.on_camera_move)
        app.spawn(start=False)(self.vuer_session)

        if https:
            self.get_logger().info(f'Open in the headset: https://<this machine>:{self.port}')
        else:
            self.get_logger().info(f'Open here: http://localhost:{self.port}')
        app.run()

    async def on_camera_move(self, event, session):
        try:
            matrix = matrix_from_event(event.value['camera']['matrix'])
        except (KeyError, TypeError):
            return
        if matrix is None:
            return
        self.head = (matrix, time.monotonic())
        self.stats.add('head')

    async def vuer_session(self, session):
        from vuer.schemas import ImageBackground

        self.sessions += 1
        self.get_logger().info(f'Headset connected ({self.sessions} open).')
        period = 1.0 / self.stream_hz
        last_seq = None
        try:
            # Vuer doesn't end this task when the headset disconnects
            while self.running and session.CURRENT_WS_ID in self.app.ws:
                start = time.monotonic()
                frame = self.frame
                if frame is not None and frame[2] != last_seq:
                    left, right, last_seq = frame
                    # Eye height in pixels before row_step: the image keeps its shape
                    aspect = left.shape[1] / (left.shape[0] * self.row_step)
                    common = dict(format='jpeg', quality=self.jpeg_quality,
                                  interpolate=True, aspect=aspect, height=8,
                                  position=[0, -1, 3])
                    # layers 1 and 2: left and right eye in WebXR
                    session.upsert([
                        ImageBackground(left, key='left-image', layers=1, **common),
                        ImageBackground(right, key='right-image', layers=2, **common),
                    ], to='bgChildren')
                    self.stats.add('stream')
                await asyncio.sleep(max(0.0, period - (time.monotonic() - start)))
        finally:
            self.sessions -= 1
            self.get_logger().info(f'Headset disconnected ({self.sessions} open).')

    # ROS

    def publish_head(self):
        head = self.head
        if head is None or head is self.head_published:
            return
        self.head_published = head
        rot, pos = yup_to_zup(head[0])
        x, y, z, w = quaternion_xyzw(rot)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pos)
        msg.pose.orientation.x = float(x)
        msg.pose.orientation.y = float(y)
        msg.pose.orientation.z = float(z)
        msg.pose.orientation.w = float(w)
        self.head_pub.publish(msg)

    def log_stats(self):
        c = self.stats.take()
        self.get_logger().info(
            f"camera {c.get('camera', 0) / 5:.0f} fps, "
            f"sent {c.get('stream', 0) / 5:.0f} fps to {self.sessions} headset(s), "
            f"head pose {c.get('head', 0) / 5:.0f} Hz")

    def destroy_node(self):
        self.running = False
        self.camera_thread.join(timeout=2.0)  # not inside a grab when closing
        self.source.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = StreamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
