"""Image processing for row following and fruit detection (numpy and OpenCV only).

Everything here is a pure function of its inputs, so it can be tested with
synthetic images and run without ROS. The nodes only move data in and out.
"""
import math
from dataclasses import dataclass, field

import cv2
import numpy as np

# ---------------------------------------------------------------- row following


@dataclass(frozen=True)
class RowParams:
    band: tuple = (0.60, 0.97)      # rows of the image used, as fractions of its height
    step: int = 6                   # sample every `step` image rows
    min_run: float = 0.06           # a furrow run must be at least this fraction of the width
    min_points: int = 5             # a line needs at least this many samples
    soil_hue: tuple = (3, 30)       # OpenCV hue (0-179) of bare soil
    soil_min_sat: int = 25
    soil_min_val: int = 45          # darker than this is mulch


@dataclass(frozen=True)
class Furrow:
    near: float = 0.0               # half-widths from the image centre, + = right
    far: float = 0.0
    confidence: float = 0.0
    points: list = field(default_factory=list)   # [(image row, run midpoint column)]


def valid_bgr(img):
    return (isinstance(img, np.ndarray) and img.ndim == 3 and img.shape[2] == 3 and img.dtype == np.uint8
            and img.shape[0] >= 16 and img.shape[1] >= 16)


def soil_mask(bgr, p=RowParams()):
    """Bare soil: brownish, not too dark (mulch) and not green (leaves)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    soil = (h >= p.soil_hue[0]) & (h <= p.soil_hue[1]) & (s >= p.soil_min_sat) & (v >= p.soil_min_val)
    mask = soil.astype(np.uint8) * 255
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (15, 3)))


def furrow_line(bgr, p=RowParams()):
    """Fit a line through the furrow and report where it crosses the band's ends.

    For each sampled image row, the soil run nearest the image centre (or the previous
    row's run, tracking upward from the bottom) is the furrow; a least-squares line
    through the run midpoints gives `near` (bottom of the image) and `far` (top of the
    band). A frame that is not a usable colour image gives an empty result.
    """
    if not valid_bgr(bgr):
        return Furrow()
    h, w = bgr.shape[:2]
    mask = soil_mask(bgr, p)
    points = []
    centre = w / 2
    for y in range(int(p.band[0] * h), int(p.band[1] * h), p.step):
        row = mask[y] > 0
        edges = np.flatnonzero(np.diff(np.r_[0, row.astype(np.int8), 0]))
        runs = [(a, b) for a, b in zip(edges[::2], edges[1::2]) if b - a >= p.min_run * w]
        if not runs:
            continue
        a, b = min(runs, key=lambda r: abs((r[0] + r[1]) / 2 - centre))
        if a <= 1 and b >= w - 1:           # whole row is soil: no beds in view here
            continue
        points.append((y, (a + b) / 2.0))
        centre = (a + b) / 2.0
    if len(points) < p.min_points:
        return Furrow(points=points)
    ys, xs = np.array(points).T
    slope, intercept = np.polyfit(ys, xs, 1)
    near = (slope * (h - 1) + intercept - w / 2) / (w / 2)
    far = (slope * p.band[0] * h + intercept - w / 2) / (w / 2)
    residual = np.abs(xs - (slope * ys + intercept)).mean() / w
    coverage = len(points) / ((p.band[1] - p.band[0]) * h / p.step)
    confidence = float(np.clip(coverage, 0, 1) * (1 - min(1, 10 * residual)))
    return Furrow(float(near), float(far), confidence, points)


def row_command(furrow, speed, k_near, k_far, min_confidence):
    """(linear x m/s, angular z rad/s) that steers along the furrow.

    A furrow to the right of the image centre (near or far > 0) means turn right,
    which is a negative yaw rate. Under perspective a sideways offset also tilts
    the line, so the law works on the two points rather than on the tilt. Below the
    confidence threshold the rover stops and waits.
    """
    if furrow.confidence < min_confidence:
        return 0.0, 0.0
    return speed, -(k_near * furrow.near + k_far * furrow.far)


def draw_row(bgr, furrow, p=RowParams()):
    out = bgr.copy()
    h, w = out.shape[:2]
    tint = np.zeros_like(out)
    tint[soil_mask(bgr, p) > 0] = (60, 160, 255)
    out = cv2.addWeighted(out, 1.0, tint, 0.35, 0)
    for y, x in furrow.points:
        cv2.circle(out, (int(x), int(y)), 3, (0, 255, 255), -1)
    if furrow.points:
        x0, x1 = furrow.near * w / 2 + w / 2, furrow.far * w / 2 + w / 2
        cv2.line(out, (int(x0), h - 1), (int(x1), int(p.band[0] * h)), (0, 255, 0), 2)
    cv2.line(out, (w // 2, h - 1), (w // 2, int(p.band[0] * h)), (255, 255, 255), 1)
    cv2.putText(out, f"near {furrow.near:+.2f}  far {furrow.far:+.2f}  conf {furrow.confidence:.2f}",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return out


# -------------------------------------------------------------- fruit detection


@dataclass(frozen=True)
class FruitParams:
    fruit_radius: float = 0.028     # m: depth hits the near surface; the centre is this far behind
    min_area: int = 150             # px
    min_depth_pixels: int = 20      # valid depth samples a blob needs
    min_range: float = 0.05         # m
    max_range: float = 0.9          # m: beyond this the arm cannot reach anyway
    core_fraction: float = 0.3      # depth is read within this fraction of the blob's radius around its centroid
    depth_percentile: float = 30.0  # fallback when the core has too few valid pixels: near surface of the whole blob
    red_hue: tuple = (8, 170)       # hue <= first or >= second counts as red (OpenCV 0-179)
    red_min_sat: int = 120
    red_min_val: int = 60


@dataclass(frozen=True)
class Blob:
    pixel: tuple                    # (u, v) centroid
    bbox: tuple                     # (x, y, w, h)
    range: float                    # m, near surface along the optical axis
    camera_xyz: tuple               # fruit centre in the camera optical frame
    area: int


def red_mask(bgr, p=FruitParams()):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = ((h <= p.red_hue[0]) | (h >= p.red_hue[1])) & (s >= p.red_min_sat) & (v >= p.red_min_val)
    mask = red.astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def blob_depth(depth, member, u, v, area, p):
    """Depth of the fruit's near surface, or None if the blob has too few valid depth samples.

    The pixels around the blob's centroid look along the fruit's central ray, so their depth
    is the near surface; the silhouette's pixels see the sides and the background edge.
    For a sphere this reads the near surface to a millimetre; a percentile over the whole
    blob reads 3-4 mm too far.
    """
    valid = member & np.isfinite(depth) & (depth >= p.min_range)
    if np.count_nonzero(valid) < p.min_depth_pixels:
        return None
    ys, xs = np.nonzero(valid)
    radius = p.core_fraction * math.sqrt(area / math.pi)
    core = (xs - u) ** 2 + (ys - v) ** 2 <= max(radius, 2.0) ** 2
    if np.count_nonzero(core) >= p.min_depth_pixels // 4:
        return float(np.median(depth[ys[core], xs[core]]))
    return float(np.percentile(depth[ys, xs], p.depth_percentile))


def locate_blobs(bgr, depth, K, p=FruitParams()):
    """Red blobs with their 3D centre in the camera optical frame (x right, y down, z forward).

    `depth` is the metric depth image (float, same size as `bgr`; NaN or inf where
    there is no return). A blob is dropped if it is too small, has too few valid
    depth samples, or lies outside [min_range, max_range]. Returns (blobs, mask).
    Bad input (missing intrinsics, mismatched sizes, wrong dtype) gives no blobs.
    """
    if not valid_bgr(bgr) or K is None or depth is None:
        return [], None
    K = np.asarray(K, float)
    depth = np.asarray(depth)
    if K.shape != (3, 3) or depth.shape != bgr.shape[:2] or not K[0, 0] > 0 or not K[1, 1] > 0:
        return [], None
    depth = depth.astype(np.float32)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    mask = red_mask(bgr, p)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    blobs = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < p.min_area:
            continue
        u, v = cents[i]
        z = blob_depth(depth, labels == i, u, v, stats[i, cv2.CC_STAT_AREA], p)
        if z is None or not (p.min_range <= z <= p.max_range):
            continue
        ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        centre = ray * z + ray / np.linalg.norm(ray) * p.fruit_radius
        blobs.append(Blob((float(u), float(v)), tuple(int(s) for s in stats[i, :4]), z,
                          tuple(float(c) for c in centre), int(stats[i, cv2.CC_STAT_AREA])))
    return blobs, mask


def quat_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        raise ValueError("zero quaternion")
    s = 2.0 / n
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def to_target_frame(camera_xyz, rotation, translation):
    """Rotate and shift a camera-frame point into the target frame (R @ p + t)."""
    return np.asarray(rotation) @ np.asarray(camera_xyz) + np.asarray(translation)


def draw_fruit(bgr, mask, blobs):
    out = bgr.copy()
    if mask is not None:
        tint = np.zeros_like(out)
        tint[mask > 0] = (255, 255, 0)
        out = cv2.addWeighted(out, 1.0, tint, 0.35, 0)
    for b in blobs:
        x, y, w, h = b.bbox
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.putText(out, f"{b.range:.2f} m", (x, max(14, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(out, f"{len(blobs)} red fruit", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return out
