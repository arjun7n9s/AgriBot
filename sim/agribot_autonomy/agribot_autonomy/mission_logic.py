"""Decision logic of the harvest mission, free of ROS so it can be tested directly."""
import math
from dataclasses import dataclass

import numpy as np

AXES = "xyz"


@dataclass(frozen=True)
class Reach:
    """Where the arm can cut, in the rover frame (base_footprint), as (low, high) per axis."""
    x: tuple = (-0.20, 0.22)
    y: tuple = (-0.74, -0.45)
    z: tuple = (0.62, 0.90)

    def __post_init__(self):
        for axis in AXES:
            lo, hi = getattr(self, axis)
            if not lo < hi:
                raise ValueError(f"reach {axis} window must have low < high, got ({lo}, {hi})")

    def contains(self, p, margin=0.0):
        return all(getattr(self, a)[0] - margin <= v <= getattr(self, a)[1] + margin for a, v in zip(AXES, p))


@dataclass(frozen=True)
class Crate:
    """Crate position the arm drops into, in the rover frame."""
    centre: tuple = (-0.36, 0.02)
    half: tuple = (0.192, 0.142)     # inside half extents (x, y)
    drop: tuple = (-0.29, -0.03, 0.86)

    def drop_inside(self):
        return (abs(self.drop[0] - self.centre[0]) < self.half[0]
                and abs(self.drop[1] - self.centre[1]) < self.half[1])


def world_xy_near(point, others, radius):
    """True if the point is within `radius` (horizontally) of any of `others`."""
    return any(math.hypot(point[0] - o[0], point[1] - o[1]) < radius for o in others)


def steady_targets(current, previous, tolerance):
    """Detections (as (rover_point, world_point) pairs) that were also seen last frame.

    Frames are compared in the world frame: the rover moves between frames, the fruit
    does not. A blob that shows up in a single frame is usually a glint or an edge
    effect of the colour rule.
    """
    return [c for c, w in current if any(np.linalg.norm(np.asarray(w) - np.asarray(p)) < tolerance
                                         for p in previous)]


def cluster_detections(frames, radius=0.03, min_fraction=0.3, min_frames=2):
    """Group detections gathered over several frames and keep those seen in most of them.

    `frames` is a list (one entry per image) of lists of 3-vectors. A detection joins the
    first cluster whose median is within `radius`. A cluster is kept when it appears in at
    least max(min_frames, min_fraction * len(frames)) frames, and is reported as its median.
    With no frames nothing is confirmed.
    """
    if not frames:
        return []
    clusters = []
    for frame in frames:
        for c in frame:
            c = np.asarray(c, float)
            for cl in clusters:
                if np.linalg.norm(np.median(cl, axis=0) - c) < radius:
                    cl.append(c)
                    break
            else:
                clusters.append([c])
    need = max(min_frames, int(min_fraction * len(frames)))
    return [np.median(cl, axis=0) for cl in clusters if len(cl) >= need]


class StallWatchdog:
    """Detects that the rover has stopped making progress.

    update(t, x) returns True once `x` has not advanced by `min_progress` for `window` seconds.
    """

    def __init__(self, window, min_progress):
        self.window, self.min_progress = window, min_progress
        self.t0 = self.x0 = None

    def reset(self):
        self.t0 = self.x0 = None

    def update(self, t, x):
        if self.t0 is None or x - self.x0 >= self.min_progress:
            self.t0, self.x0 = t, x
            return False
        return t - self.t0 >= self.window


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
