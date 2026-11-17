#!/usr/bin/env python3
"""Test a running stream ('make stream' or 'make stream-fake') without a
headset: connect like the headset's page does, check that both eyes'
images arrive, send a head pose (head turned 30 deg left, looking 20 deg
up) and check it on /television/head_pose.

Usage: test_stream.py [url]   (default ws://localhost:8012; wss:// with 'make cert')
"""

import asyncio
import io
import ssl
import sys
import threading
import time

import msgpack
import numpy as np
import rclpy
import websockets
from geometry_msgs.msg import PoseStamped
from PIL import Image
from scipy.spatial.transform import Rotation

URL = sys.argv[1] if len(sys.argv) > 1 else 'ws://localhost:8012'
DURATION = 5.0
YAW, PITCH = 30.0, 20.0

failures = []


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what)
    if not ok:
        failures.append(what)


def head_event():
    # WebXR (Y up): turn left about Y, then look up about X
    rot = Rotation.from_euler('YX', [YAW, PITCH], degrees=True).as_matrix()
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = [0.0, 1.6, 0.0]
    return {'etype': 'CAMERA_MOVE', 'key': 'defaultCamera', 'ts': time.time() * 1000,
            'value': {'camera': {'matrix': list(m.flatten(order='F')), 'aspect': 1.0}}}


def flatten(nodes):
    # upsert([a, b]) sends the list nested in another one
    for node in nodes:
        if isinstance(node, list):
            yield from flatten(node)
        else:
            yield node


async def headset():
    """Frames received per eye, and the decoded images' sizes."""
    frames = {'left-image': 0, 'right-image': 0}
    sizes = set()
    ctx = ssl._create_unverified_context() if URL.startswith('wss') else None
    async with websockets.connect(URL, ssl=ctx, max_size=2**25) as ws:
        end = time.monotonic() + DURATION
        next_head = 0.0
        while time.monotonic() < end:
            if time.monotonic() > next_head:
                await ws.send(msgpack.packb(head_event()))
                next_head = time.monotonic() + 1 / 30
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=0.05)
            except asyncio.TimeoutError:
                continue
            event = msgpack.unpackb(msg, raw=False)
            if event.get('etype') != 'UPSERT':
                continue
            for node in flatten(event['data']['nodes']):
                if node.get('key') in frames and node.get('tag') == 'ImageBackground':
                    frames[node['key']] += 1
                    img = Image.open(io.BytesIO(node['src']))
                    sizes.add(img.size)  # (width, height)
    return frames, sizes


def main():
    rclpy.init()
    node = rclpy.create_node('television_test_stream')
    poses = []
    node.create_subscription(PoseStamped, 'television/head_pose', poses.append, 10)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print(f'Connecting to {URL} for {DURATION:g} s...')
    try:
        frames, sizes = asyncio.run(headset())
    except (OSError, websockets.WebSocketException) as e:
        print(f'FAIL cannot connect: {e}. Is make stream (or stream-fake) running?')
        sys.exit(1)

    rate = {k: v / DURATION for k, v in frames.items()}
    print(f"Frames/s: left {rate['left-image']:.1f}, right {rate['right-image']:.1f}")
    check(min(rate.values()) >= 10, 'both eyes receive >= 10 frames/s')
    check(abs(frames['left-image'] - frames['right-image']) <= 1, 'eyes in step')
    print(f'Image size (width, height): {sorted(sizes)}')
    check(len(sizes) == 1, 'all images the same size')

    time.sleep(0.5)
    check(len(poses) > 0, f'head pose published ({len(poses)} messages)')
    if poses:
        q = poses[-1].pose.orientation
        yaw, pitch, _ = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX', degrees=True)
        p = poses[-1].pose.position
        print(f'Head: yaw {yaw:.1f}, pitch {pitch:.1f} deg; height {p.z:.2f} m')
        check(abs(yaw - YAW) < 0.5, f'yaw {YAW:g} (head turned left)')
        check(abs(pitch + PITCH) < 0.5, f'pitch {-PITCH:g} (looking up, ROS: nose up is negative)')
        check(abs(p.z - 1.6) < 1e-6, 'height 1.6 m (Z up)')

    node.destroy_node()
    rclpy.try_shutdown()
    print('RESULT: ' + ('PASS' if not failures else f'FAIL ({len(failures)})'))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
