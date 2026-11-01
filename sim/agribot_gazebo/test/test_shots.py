import numpy as np
import pytest

from agribot_gazebo.shots import look_at_quaternion


def forward(q):
    x, y, z, w = q
    return np.array([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)])      # first column of R


@pytest.mark.parametrize("eye, target", [((0, 0, 0), (1, 0, 0)), ((0, 0, 0), (0, 1, 0)), ((1, 2, 3), (4, -2, 0.5)),
                                         ((0, 0, 2), (0.3, 0.1, 0)), ((5, 5, 1), (-3, 2, 1.5))])
def test_camera_looks_at_the_target(eye, target):
    q = look_at_quaternion(eye, target)
    assert sum(c * c for c in q) == pytest.approx(1.0)
    want = np.array(target, float) - np.array(eye, float)
    assert np.allclose(forward(q), want / np.linalg.norm(want), atol=1e-9)


def test_no_roll():
    """The camera's right-hand side stays horizontal: its y axis (left) has no z component."""
    x, y, z, w = look_at_quaternion((0, 0, 0), (1, 2, -1))
    left = np.array([2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)])
    assert left[2] == pytest.approx(0.0, abs=1e-12)


def test_straight_down_is_a_valid_rotation():
    q = look_at_quaternion((0, 0, 2), (0, 0, 0))
    assert sum(c * c for c in q) == pytest.approx(1.0)
    assert np.allclose(forward(q), (0, 0, -1), atol=1e-9)


def test_coincident_points_are_refused():
    with pytest.raises(ValueError):
        look_at_quaternion((1, 1, 1), (1, 1, 1))
