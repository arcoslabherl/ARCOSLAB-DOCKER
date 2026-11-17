"""Stereo image sources: the ZED camera, or a test pattern (no hardware).

Each source returns (left, right): two RGB uint8 images of the same size.
"""

import time

import cv2
import numpy as np

# ZED resolutions (per eye, width x height)
ZED_RESOLUTIONS = {
    'HD2K': (2208, 1242),
    'HD1080': (1920, 1080),
    'HD720': (1280, 720),
    'VGA': (672, 376),
}


class FakeStereo:
    """Test pattern: 'L' in the left eye, 'R' in the right eye, and a bar
    with disparity, so it looks nearer than the background. A swapped
    stereo pair shows the wrong letter in each eye and the bar behind."""

    def __init__(self, resolution='HD720', fps=30.0):
        self.width, self.height = ZED_RESOLUTIONS[resolution]
        self.period = 1.0 / fps
        self.next_time = time.monotonic()
        self.start = self.next_time

    def grab(self):
        # Pace like a camera
        delay = self.next_time - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self.next_time = max(self.next_time + self.period, time.monotonic())
        t = time.monotonic() - self.start
        return self._eye('L', t, 0), self._eye('R', t, -self.width // 20)

    def _eye(self, letter, t, disparity):
        w, h = self.width, self.height
        img = np.full((h, w, 3), 40, dtype=np.uint8)
        # Grid, at "infinity" (no disparity)
        step = h // 8
        img[::step, :] = 90
        img[:, ::step] = 90
        # Bar moving left and right, shifted between the eyes
        x = int((0.5 + 0.35 * np.sin(t)) * w) + disparity
        cv2.rectangle(img, (x - w // 40, h // 4), (x + w // 40, 3 * h // 4),
                      (230, 120, 30), -1)
        scale = h / 180
        cv2.putText(img, letter, (w // 20, h // 4), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (255, 255, 255), max(1, int(scale * 2)))
        cv2.putText(img, f'{t:7.2f} s', (w // 20, h - h // 12),
                    cv2.FONT_HERSHEY_SIMPLEX, scale / 3, (255, 255, 255),
                    max(1, int(scale)))
        return img

    def close(self):
        pass


class ZedStereo:
    """The ZED camera (ZED SDK, pyzed)."""

    def __init__(self, resolution='HD720', fps=60.0):
        import pyzed.sl as sl  # only in the image, with the ZED SDK
        self.sl = sl
        self.camera = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, resolution)
        init.camera_fps = int(fps)
        # Video only: no depth, so no GPU work beyond the SDK's own
        init.depth_mode = sl.DEPTH_MODE.NONE
        err = self.camera.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(
                f'Cannot open the ZED: {err!r}. Is it plugged in (USB 3), '
                'and does the container have the GPU (make run says so)?')
        info = self.camera.get_camera_information()
        res = info.camera_configuration.resolution
        self.width, self.height = res.width, res.height
        self.left = sl.Mat()
        self.right = sl.Mat()
        self.runtime = sl.RuntimeParameters()

    def grab(self):
        """The next pair, or None if the grab failed (try again)."""
        sl = self.sl
        if self.camera.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return None
        self.camera.retrieve_image(self.left, sl.VIEW.LEFT)
        self.camera.retrieve_image(self.right, sl.VIEW.RIGHT)
        # BGRA from the SDK; copies, since the Mats are reused
        return (cv2.cvtColor(self.left.get_data(), cv2.COLOR_BGRA2RGB),
                cv2.cvtColor(self.right.get_data(), cv2.COLOR_BGRA2RGB))

    def close(self):
        self.camera.close()


def open_source(name, resolution, fps):
    if name == 'fake':
        return FakeStereo(resolution, fps)
    if name == 'zed':
        return ZedStereo(resolution, fps)
    raise ValueError(f"source must be 'zed' or 'fake', not {name!r}")
