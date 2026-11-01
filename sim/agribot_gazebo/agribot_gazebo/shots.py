"""Camera maths for the film director (no ROS)."""
import math


def look_at_quaternion(eye, target):
    """(x, y, z, w) of a camera at `eye` looking at `target`: x forward, z up, no roll.

    Raises ValueError if the two points coincide. Looking straight up or down keeps the
    yaw of the horizontal part (zero for a vertical line), so the result is still a valid
    unit quaternion.
    """
    f = [t - e for t, e in zip(target, eye)]
    n = math.sqrt(sum(c * c for c in f))
    if n < 1e-9:
        raise ValueError("eye and target coincide")
    f = [c / n for c in f]
    yaw, pitch = math.atan2(f[1], f[0]), -math.asin(max(-1.0, min(1.0, f[2])))
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)
