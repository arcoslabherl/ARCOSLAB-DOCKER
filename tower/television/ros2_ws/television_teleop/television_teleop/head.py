"""Head pose from the headset (Vuer's CAMERA_MOVE event).

Vuer sends the headset camera's pose as 16 floats, a 4x4 matrix in
column-major order, in a Y-up frame (WebXR). ROS uses Z-up: X forward,
Y left, Z up. Same conversion as Open-TeleVision (constants_vuer.py).
"""

import numpy as np
from scipy.spatial.transform import Rotation

# WebXR Y-up (X right, Y up, Z back) to ROS Z-up (X forward, Y left, Z up)
YUP_TO_ZUP = np.array([[0.0, 0.0, -1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0]])


def matrix_from_event(values):
    """The 4x4 matrix from CAMERA_MOVE's 16 values, or None if invalid."""
    try:
        raw = np.asarray(values, dtype=float)
    except (TypeError, ValueError):
        return None
    if raw.size != 16 or not np.all(np.isfinite(raw)):
        return None
    matrix = raw.reshape(4, 4, order='F')
    rot = matrix[:3, :3]
    # A rotation, not zeros or a scaled matrix
    if not np.allclose(rot.T @ rot, np.eye(3), atol=0.1):
        return None
    if np.linalg.det(rot) < 0.0:
        return None
    return matrix


def yup_to_zup(matrix):
    """Rotation (3x3) and position (3) of a Y-up pose, in Z-up."""
    rot = YUP_TO_ZUP @ matrix[:3, :3] @ YUP_TO_ZUP.T
    pos = YUP_TO_ZUP @ matrix[:3, 3]
    return rot, pos


def quaternion_xyzw(rot):
    """Quaternion (x, y, z, w) of a rotation matrix, re-orthonormalized."""
    return Rotation.from_matrix(rot).as_quat()
