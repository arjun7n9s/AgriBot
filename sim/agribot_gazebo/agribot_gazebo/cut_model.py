"""Pure logic of the simulated cut and the crate check (no ROS, no Gazebo).

Gazebo cannot cut a pedicel, so the simulator models the cut: when the blade closes
with a fruit's pedicel in the throat, that fruit is replaced by a free copy that
the cradle can catch. These helpers decide when the blade has closed, which fruit
is in the throat, and whether a fruit ended up in the crate.
"""
import math
from dataclasses import dataclass

import numpy as np


class BladeTrigger:
    """Fires once each time the blade closes; re-arms when it has opened again.

    The two thresholds make a hysteresis band, so a joint value that jitters around
    one threshold cannot fire twice. Closed is a lower (more negative) joint value.
    """

    def __init__(self, closed_below=-1.0, reopened_above=-0.3):
        if not closed_below < reopened_above:
            raise ValueError("closed_below must be lower than reopened_above")
        self.closed_below, self.reopened_above = closed_below, reopened_above
        self.armed = True

    def update(self, position):
        if not math.isfinite(position):
            return False
        if self.armed and position <= self.closed_below:
            self.armed = False
            return True
        if not self.armed and position >= self.reopened_above:
            self.armed = True
        return False


def fruit_in_throat(fruit, removed, throat, radius):
    """(fruit entry, distance) of the nearest not yet cut fruit whose pedicel is within
    `radius` of the throat, or (None, None). `fruit` entries have a 'name' and a
    'cut_point' (world xyz)."""
    best, dist = None, radius
    for f in fruit:
        if f["name"] in removed:
            continue
        d = float(np.linalg.norm(np.asarray(f["cut_point"]) - np.asarray(throat)))
        if d < dist:
            best, dist = f, d
    return (best, dist) if best is not None else (None, None)


def rpy_to_quaternion(r, p, y):
    """(x, y, z, w) of the rotation about fixed x, then y, then z (the SDF convention)."""
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


@dataclass(frozen=True)
class CrateBox:
    """Inside of the crate in the rover frame; a fruit counts when it is inside this box."""
    centre: tuple = (-0.36, 0.02)
    half: tuple = (0.192, 0.142)
    floor_z: float = 0.53
    rim_z: float = 0.73
    margin: float = 0.01            # sideways slack for the walls' thickness
    floor_slack: float = 0.02       # below the floor level this much still counts as lying on it

    def contains(self, local):
        x, y, z = local
        return (abs(x - self.centre[0]) < self.half[0] + self.margin
                and abs(y - self.centre[1]) < self.half[1] + self.margin
                and self.floor_z - self.floor_slack < z < self.rim_z)


def to_rover_frame(point_world, base_world_pose):
    """World point to the rover frame, given the rover's (x, y, yaw) in the world."""
    x, y, yaw = base_world_pose
    dx, dy = point_world[0] - x, point_world[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy, point_world[2])


def parse_model_position(text):
    """Position from `ign model -m <name> -p` output, or None."""
    try:
        line = text.split("XYZ", 1)[1].splitlines()[1]
        xyz = [float(v) for v in line.strip(" []").split()]
        return xyz if len(xyz) == 3 else None
    except (IndexError, ValueError):
        return None
