import numpy as np

from television_teleop.sources import FakeStereo


def test_fake_pair_shape_and_eyes_differ():
    src = FakeStereo('VGA', fps=1000.0)
    left, right = src.grab()
    assert left.shape == right.shape == (376, 672, 3)
    assert left.dtype == np.uint8
    assert not np.array_equal(left, right)  # 'L'/'R' and the bar's disparity
