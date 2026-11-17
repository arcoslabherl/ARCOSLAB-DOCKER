import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from television_teleop.head import matrix_from_event, quaternion_xyzw, yup_to_zup


def event_values(rot, pos=(0.0, 0.0, 0.0)):
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = pos
    return list(m.flatten(order='F'))  # Vuer: column-major


def test_identity_looks_forward():
    rot, pos = yup_to_zup(matrix_from_event(event_values(np.eye(3))))
    assert np.allclose(rot, np.eye(3))
    assert np.allclose(quaternion_xyzw(rot), [0, 0, 0, 1])


def test_column_major_position():
    # 1 m up, 2 m back in WebXR (Y up, Z back) = Z 1, X -2 in ROS
    _, pos = yup_to_zup(matrix_from_event(event_values(np.eye(3), (0.0, 1.0, 2.0))))
    assert np.allclose(pos, [-2.0, 0.0, 1.0])


def test_turn_left_is_positive_yaw():
    # Turning the head left: +30 deg about WebXR's Y (up)
    turn = Rotation.from_euler('y', 30, degrees=True).as_matrix()
    rot, _ = yup_to_zup(matrix_from_event(event_values(turn)))
    yaw, pitch, roll = Rotation.from_matrix(rot).as_euler('ZYX', degrees=True)
    assert yaw == pytest.approx(30)
    assert pitch == pytest.approx(0, abs=1e-9)
    assert roll == pytest.approx(0, abs=1e-9)


def test_look_up_is_negative_pitch():
    # Looking up: +20 deg about WebXR's X (right). In ROS (Y left), nose up
    # is a negative rotation about Y.
    up = Rotation.from_euler('x', 20, degrees=True).as_matrix()
    rot, _ = yup_to_zup(matrix_from_event(event_values(up)))
    yaw, pitch, roll = Rotation.from_matrix(rot).as_euler('ZYX', degrees=True)
    assert pitch == pytest.approx(-20)
    assert yaw == pytest.approx(0, abs=1e-9)


@pytest.mark.parametrize('values', [
    [0.0] * 16,                       # no tracking yet
    [1.0] * 15,                       # wrong size
    [float('nan')] + [0.0] * 15,
    event_values(2 * np.eye(3)),      # scaled
    event_values(np.diag([1.0, 1.0, -1.0])),  # mirrored
    'not a matrix',
    None,
])
def test_invalid_matrices_rejected(values):
    assert matrix_from_event(values) is None
