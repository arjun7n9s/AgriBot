"""Fruit detection: where a red sphere is found, and what happens with bad input."""
import math

import numpy as np
import pytest

from agribot_autonomy import vision

W, H = 640, 480
FX = FY = 554.0
CX, CY = 320.0, 240.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1.0]])
RED = (30, 30, 200)
LEAF = (70, 120, 60)


def render_sphere(centre, radius=0.028, colour=RED, img=None, depth=None):
    """Colour and depth image of a sphere seen by a pinhole camera (z forward, depth = z of the surface)."""
    img = np.full((H, W, 3), LEAF, np.uint8) if img is None else img
    depth = np.full((H, W), np.inf, np.float32) if depth is None else depth
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones((H, W))], -1)
    c = np.asarray(centre, float)
    a, b, cc = (rays ** 2).sum(-1), -2 * (rays @ c), c @ c - radius ** 2
    disc = b * b - 4 * a * cc
    hit = disc >= 0
    t = np.where(hit, (-b - np.sqrt(np.where(hit, disc, 0))) / (2 * a), np.inf)
    nearer = hit & (t < depth)
    img[nearer], depth[nearer] = colour, t[nearer].astype(np.float32)
    return img, depth


@pytest.mark.parametrize("centre", [(0, 0, 0.4), (0.05, -0.03, 0.35), (-0.08, 0.04, 0.5), (0.1, 0.05, 0.3),
                                    (0, 0, 0.7), (-0.15, -0.08, 0.6)])
def test_sphere_centre_is_recovered_to_a_millimetre_or_two(centre):
    img, depth = render_sphere(centre)
    blobs, _ = vision.locate_blobs(img, depth, K)
    assert len(blobs) == 1
    assert np.linalg.norm(np.array(blobs[0].camera_xyz) - centre) < 0.002
    # range is the depth (z) of the near surface on the ray through the centre
    ray = np.array([centre[0] / centre[2], centre[1] / centre[2], 1.0])
    assert blobs[0].range == pytest.approx((np.linalg.norm(centre) - 0.028) / np.linalg.norm(ray), abs=0.002)


def test_two_fruit_are_two_blobs_at_their_own_positions():
    img, depth = render_sphere((-0.1, 0.0, 0.45))
    img, depth = render_sphere((0.1, 0.02, 0.4), img=img, depth=depth)
    blobs, _ = vision.locate_blobs(img, depth, K)
    assert len(blobs) == 2
    found = sorted(b.camera_xyz for b in blobs)
    assert np.allclose(found[0], (-0.1, 0.0, 0.45), atol=0.002) and np.allclose(found[1], (0.1, 0.02, 0.4), atol=0.002)


def test_a_nearer_object_hiding_part_of_the_fruit_does_not_move_the_range_into_the_object():
    img, depth = render_sphere((0, 0, 0.5))
    img[200:300, 250:300] = LEAF                    # a leaf covers the left part of the red patch
    depth[200:300, 250:300] = 0.2
    blobs, _ = vision.locate_blobs(img, depth, K)
    assert len(blobs) == 1
    assert blobs[0].range == pytest.approx(0.5 - 0.028, abs=0.02)


def test_green_and_orange_objects_are_not_red():
    for colour_hsv in ((60, 200, 150), (18, 220, 230), (100, 200, 150)):         # leaf, orange, blue
        import cv2
        bgr = cv2.cvtColor(np.uint8([[colour_hsv]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
        img, depth = render_sphere((0, 0, 0.4), colour=tuple(bgr))
        assert vision.locate_blobs(img, depth, K)[0] == []


def test_red_hue_wraps_around_zero():
    import cv2
    for h in (0, 4, 175, 179):
        bgr = cv2.cvtColor(np.uint8([[(h, 220, 200)]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
        img, depth = render_sphere((0, 0, 0.4), colour=tuple(bgr))
        assert len(vision.locate_blobs(img, depth, K)[0]) == 1, h


def test_small_blobs_are_dropped():
    img, depth = render_sphere((0, 0, 3.0))                                        # tiny at 3 m
    assert vision.locate_blobs(img, depth, K, vision.FruitParams(max_range=10.0))[0] == []
    img, depth = render_sphere((0, 0, 0.4), radius=0.004)
    assert vision.locate_blobs(img, depth, K)[0] == []


def test_fruit_beyond_max_range_or_inside_min_range_is_dropped():
    img, depth = render_sphere((0, 0, 1.2), radius=0.06)
    assert vision.locate_blobs(img, depth, K)[0] == []
    assert len(vision.locate_blobs(img, depth, K, vision.FruitParams(max_range=2.0))[0]) == 1
    img, depth = render_sphere((0, 0, 0.5))
    depth[np.isfinite(depth)] = 0.02
    assert vision.locate_blobs(img, depth, K)[0] == []


@pytest.mark.parametrize("bad", [np.nan, np.inf, 0.0, -1.0])
def test_blobs_without_valid_depth_are_dropped(bad):
    img, depth = render_sphere((0, 0, 0.4))
    depth[np.isfinite(depth)] = bad
    assert vision.locate_blobs(img, depth, K)[0] == []


def test_a_few_valid_depth_pixels_are_not_enough():
    img, depth = render_sphere((0, 0, 0.4))
    valid = np.isfinite(depth)
    depth[valid] = np.nan
    ys, xs = np.nonzero(valid)
    depth[ys[:5], xs[:5]] = 0.4
    assert vision.locate_blobs(img, depth, K)[0] == []


def test_sparse_depth_still_gives_a_range_when_the_core_is_missing():
    img, depth = render_sphere((0, 0, 0.4))
    centre = (np.arange(H)[:, None] - CY) ** 2 + (np.arange(W)[None, :] - CX) ** 2 < 25 ** 2
    depth[centre] = np.nan                           # depth hole in the middle of the fruit
    blobs, _ = vision.locate_blobs(img, depth, K)
    assert len(blobs) == 1 and blobs[0].range == pytest.approx(0.4 - 0.028, abs=0.02)


@pytest.mark.parametrize("make", [
    lambda i, d: (None, d, K), lambda i, d: (i, None, K), lambda i, d: (i, d, None),
    lambda i, d: (i, d[:100], K), lambda i, d: (i, d[:, :100], K), lambda i, d: (i, d, K[:2]),
    lambda i, d: (i, d, np.zeros((3, 3))), lambda i, d: (i, d, np.full((3, 3), np.nan)),
    lambda i, d: (i[..., 0], d, K), lambda i, d: (i.astype(np.float32), d, K), lambda i, d: ("x", d, K),
])
def test_bad_input_gives_no_blobs_and_no_exception(make):
    img, depth = render_sphere((0, 0, 0.4))
    blobs, mask = vision.locate_blobs(*make(img, depth))
    assert blobs == []


def test_empty_scene_gives_no_blobs():
    img = np.full((H, W, 3), LEAF, np.uint8)
    assert vision.locate_blobs(img, np.full((H, W), 0.5, np.float32), K)[0] == []


def test_depth_given_as_float64_or_float16_works():
    img, depth = render_sphere((0, 0, 0.4))
    for dtype in (np.float64, np.float16):
        assert len(vision.locate_blobs(img, depth.astype(dtype), K)[0]) == 1


# ------------------------------------------------------------------ transforms
def rot_z(a):
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def test_quaternion_to_matrix_known_rotations():
    s = math.sqrt(0.5)
    assert np.allclose(vision.quat_to_matrix(0, 0, s, s) @ [1, 0, 0], [0, 1, 0])           # +90 deg about z
    assert np.allclose(vision.quat_to_matrix(s, 0, 0, s) @ [0, 1, 0], [0, 0, 1])           # +90 deg about x
    assert np.allclose(vision.quat_to_matrix(0, 0, 0, 1), np.eye(3))
    assert np.allclose(vision.quat_to_matrix(0, 0, 0, 2.0), np.eye(3))                     # not normalised: scaled out
    a = 0.7
    assert np.allclose(vision.quat_to_matrix(0, 0, math.sin(a / 2), math.cos(a / 2)), rot_z(a))


def test_quaternion_matrices_are_rotations():
    rng = np.random.default_rng(1)
    for _ in range(20):
        R = vision.quat_to_matrix(*rng.normal(size=4))
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12) and np.linalg.det(R) == pytest.approx(1.0)


def test_zero_quaternion_is_refused():
    with pytest.raises(ValueError):
        vision.quat_to_matrix(0, 0, 0, 0)


def test_target_frame_transform():
    p = vision.to_target_frame((1, 0, 0), rot_z(math.pi / 2), (0.5, 0.2, 0.7))
    assert np.allclose(p, (0.5, 1.2, 0.7))


def test_camera_frame_detection_lands_on_the_known_world_point():
    """A fruit at a known place, seen by a camera at a known pose, is reported at that place."""
    world_fruit = np.array([1.0, -0.4, 0.8])
    # Camera 0.5 m to the side of the fruit, optical axis along the world -y direction... build it explicitly:
    # optical frame: x right, y down, z forward. Look along world +y: z_cam = +y_w, x_cam = +x_w, y_cam = -z_w.
    R = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float).T            # camera axes expressed in the world frame
    # columns of R are camera x, y, z in the world: x_w, -z_w, y_w
    R = np.column_stack([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    cam_pos = world_fruit + np.array([0.03, -0.45, 0.04])
    in_cam = R.T @ (world_fruit - cam_pos)
    img, depth = render_sphere(in_cam)
    blobs, _ = vision.locate_blobs(img, depth, K)
    found = vision.to_target_frame(blobs[0].camera_xyz, R, cam_pos)
    assert np.linalg.norm(found - world_fruit) < 0.002
