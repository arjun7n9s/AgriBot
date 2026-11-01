"""Steering and wheel-speed kinematics for the car chassis (no ROS imports).

The chassis steers with its two front wheels and has four driven wheels. A body
velocity command (v, w) becomes:

* a steering angle for each front knuckle, and
* a speed for each wheel, from how fast that wheel's own contact point moves.

The simulator's wheels are stiff velocity servos: four wheels commanded the same
speed fight every turn. Real DC wheel motors give under load, which hides this.

Frame: origin at the rear-axle centre, x forward, y left. Positive w and positive
steering angles turn left. Distances in metres, angles in radians.
"""
import math
from dataclasses import dataclass

EPS = 1e-3          # m/s: below this speed the steering angle is held, not recomputed


@dataclass(frozen=True)
class Geometry:
    wheelbase: float = 0.72         # assumed, see docs/work-plan.md
    track: float = 0.55             # assumed
    wheel_radius: float = 0.19
    steer_limit: float = math.radians(25)   # largest angle of the inner front wheel
    ackermann: bool = True          # False: both knuckles at the same angle (parallel steer)

    def __post_init__(self):
        for name in ("wheelbase", "track", "wheel_radius", "steer_limit"):
            v = getattr(self, name)
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"{name} must be a positive number, got {v}")
        if self.steer_limit >= math.pi / 2:
            raise ValueError("steer_limit must be below 90 degrees")

    @property
    def wheels(self):
        """Wheel centres [front left, front right, rear left, rear right] as (x, y)."""
        return [(self.wheelbase, self.track / 2), (self.wheelbase, -self.track / 2),
                (0.0, self.track / 2), (0.0, -self.track / 2)]

    @property
    def max_centre_angle(self):
        """Largest bicycle (centre) angle: keeps the inner wheel within steer_limit."""
        if not self.ackermann:
            return self.steer_limit
        k = self.track / (2 * self.wheelbase)
        t = math.tan(self.steer_limit)
        return math.atan(t / (1 + k * t))


@dataclass(frozen=True)
class DrivePlan:
    steer_left: float       # rad, left knuckle
    steer_right: float      # rad, right knuckle
    wheel_omega: tuple      # rad/s [front left, front right, rear left, rear right]
    centre_angle: float     # rad, the bicycle angle behind this plan (carry it into the next call)
    v: float                # m/s, the body velocity this plan gives
    w: float                # rad/s


def plan(geom, v, w, held_angle=0.0):
    """Turn a body command into steering angles and wheel speeds.

    The centre angle is atan(wheelbase * w / v), clipped to the steering limit, and
    the yaw rate is recomputed from the clipped angle so every wheel speed matches
    the path the steering can actually follow. Near standstill the angle is held at
    `held_angle` (a car cannot turn on the spot) and the plan is a stop.
    Raises ValueError for non-finite input.
    """
    if not (math.isfinite(v) and math.isfinite(w) and math.isfinite(held_angle)):
        raise ValueError(f"non-finite command: v={v}, w={w}, held_angle={held_angle}")
    limit = geom.max_centre_angle
    if abs(v) > EPS:
        delta = max(-limit, min(limit, math.atan(geom.wheelbase * w / v)))
        w_eff = v * math.tan(delta) / geom.wheelbase
        v_eff = v
    else:
        delta = max(-limit, min(limit, held_angle))
        w_eff, v_eff = 0.0, 0.0
    if geom.ackermann:
        k = geom.track / (2 * geom.wheelbase) * math.tan(delta)
        left = math.atan(math.tan(delta) / (1 - k))
        right = math.atan(math.tan(delta) / (1 + k))
    else:
        left = right = delta
    omegas = []
    for x, y in geom.wheels:
        speed = math.hypot(v_eff - w_eff * y, w_eff * x)        # body velocity of the wheel centre
        omegas.append(math.copysign(speed, v_eff) / geom.wheel_radius if speed else 0.0)
    return DrivePlan(left, right, tuple(omegas), delta, v_eff, w_eff)
