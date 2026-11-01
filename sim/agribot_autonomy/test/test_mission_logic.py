"""Mission decisions: reach window, clustering, steadiness, stall detection."""
import math

import numpy as np
import pytest

from agribot_autonomy import mission_logic as m


def test_reach_contains_edges_and_margin():
    r = m.Reach()
    assert r.contains((0.0, -0.6, 0.8))
    assert r.contains((r.x[0], r.y[0], r.z[0])) and r.contains((r.x[1], r.y[1], r.z[1]))       # edges are inside
    assert not r.contains((r.x[1] + 0.001, -0.6, 0.8))
    assert r.contains((r.x[1] + 0.04, -0.6, 0.8), margin=0.05)
    assert not r.contains((r.x[1] + 0.06, -0.6, 0.8), margin=0.05)
    assert not r.contains((0, -0.6, 0.95)) and not r.contains((0, -0.3, 0.8))


@pytest.mark.parametrize("axis", ["x", "y", "z"])
def test_reach_refuses_an_empty_window(axis):
    with pytest.raises(ValueError):
        m.Reach(**{axis: (0.5, 0.5)})
    with pytest.raises(ValueError):
        m.Reach(**{axis: (0.6, 0.4)})


def test_default_crate_drop_is_over_the_crate():
    c = m.Crate()
    assert c.drop_inside()
    assert not m.Crate(drop=(c.centre[0] + c.half[0] + 0.01, c.centre[1], 0.86)).drop_inside()
    assert not m.Crate(drop=(c.centre[0], c.centre[1] - c.half[1] - 0.01, 0.86)).drop_inside()


def test_world_xy_near_ignores_height():
    assert m.world_xy_near((1.0, 2.0, 0.9), [(1.03, 2.0, 0.0)], 0.04)
    assert not m.world_xy_near((1.0, 2.0, 0.9), [(1.05, 2.0)], 0.04)
    assert not m.world_xy_near((1.0, 2.0), [], 0.04)


def test_steady_targets_compare_world_positions():
    current = [((0.1, -0.5, 0.8), (3.00, 1.00, 0.8)), ((0.0, -0.5, 0.8), (3.5, 1.0, 0.8))]
    previous = [(3.01, 1.00, 0.8), (9.0, 9.0, 0.8)]
    assert m.steady_targets(current, previous, 0.03) == [(0.1, -0.5, 0.8)]
    assert m.steady_targets(current, [], 0.03) == []
    assert m.steady_targets([], previous, 0.03) == []
    assert m.steady_targets(current, [(3.00, 1.04, 0.8)], 0.03) == []                # too far apart


def test_cluster_detections_keeps_consistent_fruit_and_drops_flicker():
    rng = np.random.default_rng(5)
    a, b = np.array([0.10, -0.50, 0.80]), np.array([0.18, -0.46, 0.82])
    frames = []
    for i in range(24):
        f = [a + rng.normal(0, 0.003, 3)]
        if i % 2 == 0:
            f.append(b + rng.normal(0, 0.003, 3))               # seen in half of the frames
        if i == 5:
            f.append(np.array([0.0, -0.6, 0.7]))               # a one-frame glint
        frames.append(f)
    out = m.cluster_detections(frames)
    assert len(out) == 2
    assert min(np.linalg.norm(o - a) for o in out) < 0.004 and min(np.linalg.norm(o - b) for o in out) < 0.004


def test_cluster_detections_requires_a_fraction_of_the_frames():
    a = np.array([0.1, -0.5, 0.8])
    frames = [[a]] * 3 + [[]] * 17                                  # seen in 3 of 20 frames: 15%
    assert m.cluster_detections(frames) == []
    assert len(m.cluster_detections(frames, min_fraction=0.1)) == 1


def test_cluster_detections_edge_cases():
    assert m.cluster_detections([]) == []
    assert m.cluster_detections([[]]) == []
    one = [np.array([0.1, -0.5, 0.8])]
    assert m.cluster_detections([one]) == []                         # a single frame never confirms (min_frames 2)
    assert len(m.cluster_detections([one, one])) == 1
    assert len(m.cluster_detections([one], min_frames=1)) == 1


def test_cluster_detections_separates_fruit_just_over_the_radius_apart():
    a = np.array([0.1, -0.5, 0.8])
    frames = [[a, a + [0.05, 0, 0]]] * 4
    assert len(m.cluster_detections(frames, radius=0.03)) == 2
    assert len(m.cluster_detections(frames, radius=0.06)) == 1


def test_stall_watchdog():
    w = m.StallWatchdog(window=10.0, min_progress=0.05)
    assert not w.update(0.0, 0.0)
    assert not w.update(5.0, 0.01)                                   # slow but within the window
    assert w.update(10.5, 0.02)                                       # no progress for 10.5 s
    w.reset()
    assert not w.update(20.0, 0.0)
    assert not w.update(25.0, 0.2)                                    # moved: the window restarts
    assert not w.update(34.0, 0.21)
    assert w.update(35.5, 0.22)


def test_stall_watchdog_progress_resets_the_timer_each_time():
    w = m.StallWatchdog(window=4.0, min_progress=0.1)
    x = 0.0
    for t in range(0, 100):
        x += 0.12 if t % 3 == 0 else 0.0                              # a step every 3 s: never stalls
        assert not w.update(float(t), x)


def test_yaw_from_quaternion():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 2.5):
        assert m.yaw_from_quaternion(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(yaw)
