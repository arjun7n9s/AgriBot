"""The simulated cut and crate check: pure logic."""
import math

import numpy as np
import pytest

from agribot_gazebo import cut_model as cm


def test_blade_trigger_fires_once_per_closing():
    t = cm.BladeTrigger(-1.0, -0.3)
    fired = [t.update(p) for p in (0.0, -0.4, -0.9, -1.0, -1.2, -1.2, -1.1)]
    assert fired == [False, False, False, True, False, False, False]


def test_blade_trigger_rearms_only_after_reopening():
    t = cm.BladeTrigger(-1.0, -0.3)
    assert t.update(-1.2)
    assert not t.update(-0.5) and not t.update(-1.2)           # halfway open does not re-arm
    assert not t.update(-0.2)                                    # reopened: re-armed
    assert t.update(-1.1)                                        # second closing fires again


def test_blade_trigger_is_immune_to_jitter_around_the_threshold():
    t = cm.BladeTrigger(-1.0, -0.3)
    fired = sum(t.update(p) for p in (-0.99, -1.01, -0.99, -1.01, -0.98, -1.02, -1.0))
    assert fired == 1


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_blade_trigger_ignores_non_finite_positions(bad):
    t = cm.BladeTrigger()
    assert not t.update(bad)
    assert t.armed


def test_blade_trigger_rejects_inverted_thresholds():
    with pytest.raises(ValueError):
        cm.BladeTrigger(-0.3, -1.0)
    with pytest.raises(ValueError):
        cm.BladeTrigger(-0.5, -0.5)


FRUIT = [{"name": "a", "cut_point": [1.0, 1.0, 0.8]}, {"name": "b", "cut_point": [1.02, 1.0, 0.8]},
         {"name": "c", "cut_point": [3.0, 1.0, 0.8]}]


def test_nearest_fruit_in_the_throat_is_chosen():
    f, d = cm.fruit_in_throat(FRUIT, set(), (1.015, 1.0, 0.8), 0.03)
    assert f["name"] == "b" and d == pytest.approx(0.005)


def test_removed_fruit_are_not_cut_twice():
    f, _ = cm.fruit_in_throat(FRUIT, {"b"}, (1.015, 1.0, 0.8), 0.03)
    assert f["name"] == "a"
    assert cm.fruit_in_throat(FRUIT, {"a", "b", "c"}, (1.0, 1.0, 0.8), 0.03) == (None, None)


def test_nothing_within_the_radius_cuts_nothing():
    assert cm.fruit_in_throat(FRUIT, set(), (2.0, 1.0, 0.8), 0.03) == (None, None)
    assert cm.fruit_in_throat([], set(), (1.0, 1.0, 0.8), 0.03) == (None, None)


def test_radius_boundary_is_exclusive():
    assert cm.fruit_in_throat(FRUIT[2:], set(), (3.0301, 1.0, 0.8), 0.03) == (None, None)
    assert cm.fruit_in_throat(FRUIT[2:], set(), (3.0299, 1.0, 0.8), 0.03)[0]["name"] == "c"


def rot_matrix_xyz_fixed(r, p, y):
    cx, sx, cy, sy, cz, sz = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


def quat_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@pytest.mark.parametrize("rpy", [(0, 0, 0), (0.1, -0.2, 2.9), (-0.008, -0.111, -2.96224), (1.2, 0.3, -1.0), (0, 0, math.pi)])
def test_rpy_to_quaternion_matches_the_rotation_matrix(rpy):
    q = cm.rpy_to_quaternion(*rpy)
    assert sum(c * c for c in q) == pytest.approx(1.0)
    assert np.allclose(quat_matrix(q), rot_matrix_xyz_fixed(*rpy), atol=1e-12)


def test_crate_box_contains():
    c = cm.CrateBox()
    assert c.contains((c.centre[0], c.centre[1], 0.58))
    assert not c.contains((c.centre[0] + 0.25, c.centre[1], 0.58))            # beside the crate
    assert not c.contains((c.centre[0], c.centre[1] + 0.25, 0.58))
    assert not c.contains((c.centre[0], c.centre[1], 0.35))                    # under the crate
    assert not c.contains((c.centre[0], c.centre[1], 0.80))                    # above the rim
    assert c.contains((c.centre[0] + c.half[0] + 0.005, c.centre[1], 0.55))   # within the wall allowance
    assert not c.contains((c.centre[0] + c.half[0] + 0.02, c.centre[1], 0.55))


def test_to_rover_frame():
    assert np.allclose(cm.to_rover_frame((2.0, 2.0, 0.7), (2.0, 1.0, math.pi / 2)), (1.0, 0.0, 0.7))   # ahead
    assert np.allclose(cm.to_rover_frame((1.0, 1.0, 0.5), (2.0, 1.0, math.pi / 2)), (0.0, 1.0, 0.5))   # to its left
    assert np.allclose(cm.to_rover_frame((3.0, 1.0, 0.0), (2.0, 1.0, 0.0)), (1.0, 0.0, 0.0))
    p = (4.2, -1.3, 0.6)
    for yaw in (-2.0, 0.4, 3.0):                                     # round trip with the inverse transform
        x, y, z = cm.to_rover_frame(p, (1.0, 2.0, yaw))
        c, s = math.cos(yaw), math.sin(yaw)
        assert np.allclose((1.0 + c * x - s * y, 2.0 + s * x + c * y), p[:2])


SAMPLE = """Requesting state for world [demo_row]...

Model: [42]
  - Name: plant_b0_r0_t1_f3_cut
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
      [-0.496 0.976 0.534]
      [0.0123 -0.0045 1.5]
"""


def test_parse_model_position():
    assert cm.parse_model_position(SAMPLE) == [-0.496, 0.976, 0.534]


@pytest.mark.parametrize("text", ["", "Model not found", "XYZ\n", "XYZ (m)\n   [1 2]\n", "XYZ (m)\n   [a b c]\n", None])
def test_parse_model_position_rejects_garbage(text):
    assert cm.parse_model_position(text if text is not None else "") is None
