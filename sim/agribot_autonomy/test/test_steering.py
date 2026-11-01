"""Steering and wheel-speed kinematics: numerical correctness and boundaries."""
import itertools
import math

import pytest

from agribot_autonomy import steering
from agribot_autonomy.steering import Geometry, plan

G = Geometry()
GRID = list(itertools.product([-0.8, -0.3, 0.05, 0.15, 0.5, 1.0], [-1.0, -0.4, -0.05, 0.0, 0.05, 0.4, 1.0]))


def wheel_velocity(g, v, w, x, y):
    """Velocity of a point rigidly attached to the body at (x, y): v + w x r."""
    return (v - w * y, w * x)


@pytest.mark.parametrize("v, w", GRID)
def test_wheel_speed_matches_rigid_body_motion(v, w):
    """Each wheel's rolling speed is the speed of its own contact point (independent check
    through the instantaneous centre of rotation for turning commands)."""
    p = plan(G, v, w)
    for omega, (x, y) in zip(p.wheel_omega, G.wheels):
        vx, vy = wheel_velocity(G, p.v, p.w, x, y)
        assert abs(omega) * G.wheel_radius == pytest.approx(math.hypot(vx, vy), abs=1e-9)
        if abs(p.w) > 1e-6:
            radius = p.v / p.w                      # signed distance of the centre from the rear axle
            icr_distance = math.hypot(x, y - radius) * abs(p.w)
            assert abs(omega) * G.wheel_radius == pytest.approx(icr_distance, abs=1e-9)


@pytest.mark.parametrize("v, w", GRID)
def test_ackermann_front_wheels_point_along_their_own_velocity(v, w):
    p = plan(G, v, w)
    if abs(v) <= steering.EPS:
        return
    for angle, (x, y) in zip((p.steer_left, p.steer_right), G.wheels[:2]):
        vx, vy = wheel_velocity(G, p.v, p.w, x, y)
        if v < 0:
            vx, vy = -vx, -vy                      # a wheel rolling backwards points the same way
        assert angle == pytest.approx(math.atan2(vy, vx), abs=1e-9)


def test_inner_wheel_turns_more_and_both_stay_within_the_limit():
    for w in (0.1, 0.5, 5.0):
        p = plan(G, 0.3, w)                         # left turn: left wheel is the inner one
        assert p.steer_left > p.steer_right > 0
        assert p.steer_left <= G.steer_limit + 1e-12
    for w in (-0.1, -0.5, -5.0):
        p = plan(G, 0.3, w)
        assert p.steer_right < p.steer_left < 0
        assert -p.steer_right <= G.steer_limit + 1e-12


def test_command_beyond_the_limit_saturates_at_the_inner_wheel_limit():
    p = plan(G, 0.3, 50.0)
    assert p.steer_left == pytest.approx(G.steer_limit, abs=1e-12)
    assert p.centre_angle == pytest.approx(G.max_centre_angle, abs=1e-12)
    assert p.w == pytest.approx(0.3 * math.tan(G.max_centre_angle) / G.wheelbase)      # recomputed, not the 50 asked for


def test_unclipped_command_gives_the_requested_yaw_rate():
    for v in (0.2, -0.2):
        p = plan(G, v, 0.1)
        assert p.w == pytest.approx(0.1, abs=1e-12)
        assert p.v == v


def test_parallel_steering_has_equal_knuckle_angles():
    g = Geometry(ackermann=False)
    p = plan(g, 0.3, 0.3)
    assert p.steer_left == p.steer_right == p.centre_angle
    assert plan(g, 0.3, 50.0).steer_left == pytest.approx(g.steer_limit)


def test_straight_driving_gives_equal_wheel_speeds_and_no_steering():
    p = plan(G, 0.4, 0.0)
    assert p.steer_left == p.steer_right == 0.0
    assert set(round(o, 12) for o in p.wheel_omega) == {round(0.4 / G.wheel_radius, 12)}


def test_turning_left_makes_the_right_wheels_faster():
    p = plan(G, 0.3, 0.3)
    fl, fr, rl, rr = p.wheel_omega
    assert fr > fl > 0 and rr > rl > 0
    assert fl > rl and fr > rr                      # front wheels travel a longer arc


def test_mirror_symmetry():
    a, b = plan(G, 0.3, 0.25), plan(G, 0.3, -0.25)
    assert a.steer_left == pytest.approx(-b.steer_right) and a.steer_right == pytest.approx(-b.steer_left)
    assert a.wheel_omega[0] == pytest.approx(b.wheel_omega[1]) and a.wheel_omega[2] == pytest.approx(b.wheel_omega[3])


def test_reverse_gives_negative_wheel_speeds():
    p = plan(G, -0.3, 0.2)
    assert all(o < 0 for o in p.wheel_omega)


@pytest.mark.parametrize("v", [0.0, 0.0005, -0.0009])
def test_near_standstill_holds_the_steering_angle_and_stops_the_wheels(v):
    held = 0.2
    p = plan(G, v, 1.0, held_angle=held)
    assert p.centre_angle == pytest.approx(held)
    assert p.wheel_omega == (0.0, 0.0, 0.0, 0.0)
    assert p.v == 0.0 and p.w == 0.0


def test_held_angle_is_clipped_to_the_limit():
    p = plan(G, 0.0, 0.0, held_angle=1.5)
    assert p.centre_angle == pytest.approx(G.max_centre_angle)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_commands_are_refused(bad):
    with pytest.raises(ValueError):
        plan(G, bad, 0.0)
    with pytest.raises(ValueError):
        plan(G, 0.3, bad)
    with pytest.raises(ValueError):
        plan(G, 0.3, 0.0, held_angle=bad)


@pytest.mark.parametrize("field, value", [("wheelbase", 0.0), ("track", -0.5), ("wheel_radius", float("nan")),
                                          ("steer_limit", 0.0), ("steer_limit", math.pi / 2)])
def test_geometry_rejects_impossible_values(field, value):
    with pytest.raises(ValueError):
        Geometry(**{field: value})


def test_minimum_turn_radius_matches_the_steering_limit():
    """At full lock the rear-axle centre turns on a circle of radius L / tan(centre angle)."""
    p = plan(G, 0.2, 10.0)
    radius = p.v / p.w
    assert radius == pytest.approx(G.wheelbase / math.tan(G.max_centre_angle))
    # and the inner front wheel's own turn radius is wheelbase / sin(limit)
    inner = math.hypot(G.wheelbase, radius - G.track / 2)
    assert inner == pytest.approx(G.wheelbase / math.sin(G.steer_limit), rel=1e-9)
