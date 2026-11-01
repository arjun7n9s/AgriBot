"""Row following: furrow line fitting and the steering law, on synthetic and real frames."""
import pathlib

import cv2
import numpy as np
import pytest

from agribot_autonomy import vision

DATA = pathlib.Path(__file__).parent / "data"
W, H = 640, 480
HORIZON = int(0.55 * H)
SOIL = cv2.cvtColor(np.uint8([[[15, 150, 160]]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()     # warm brown
MULCH = [25, 25, 25]
LEAF = cv2.cvtColor(np.uint8([[[55, 200, 120]]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
SKY = [235, 200, 150]


def furrow_image(cx_bottom=W / 2, cx_horizon=W / 2, half_bottom=0.22 * W, half_horizon=0.03 * W):
    """A furrow seen in perspective: bare soil trapezoid between two mulch beds."""
    img = np.zeros((H, W, 3), np.uint8)
    img[:HORIZON] = SKY
    img[HORIZON:] = MULCH
    for y in range(HORIZON, H):
        t = (y - HORIZON) / (H - HORIZON)
        cx = cx_horizon + (cx_bottom - cx_horizon) * t
        half = half_horizon + (half_bottom - half_horizon) * t
        img[y, int(cx - half):int(cx + half)] = SOIL
    return img


def expected(cx_bottom, cx_horizon, band_top=0.60):
    """near / far from the geometry of the synthetic furrow, independent of the fitting code."""
    def at(y):
        t = (y - HORIZON) / (H - HORIZON)
        return ((cx_horizon + (cx_bottom - cx_horizon) * t) - W / 2) / (W / 2)
    return at(H - 1), at(band_top * H)


def test_centred_furrow_gives_zero_offsets_and_high_confidence():
    f = vision.furrow_line(furrow_image())
    assert f.confidence > 0.9
    assert f.near == pytest.approx(0, abs=0.02) and f.far == pytest.approx(0, abs=0.02)


@pytest.mark.parametrize("cx_bottom, cx_horizon", [(W / 2 + 60, W / 2), (W / 2 - 90, W / 2), (W / 2, W / 2 + 25),
                                                   (W / 2 + 40, W / 2 - 20), (W / 2 - 50, W / 2 + 30)])
def test_offsets_match_the_known_geometry(cx_bottom, cx_horizon):
    f = vision.furrow_line(furrow_image(cx_bottom, cx_horizon))
    near, far = expected(cx_bottom, cx_horizon)
    assert f.confidence > 0.9
    assert f.near == pytest.approx(near, abs=0.03)
    assert f.far == pytest.approx(far, abs=0.03)


def test_mirrored_image_negates_both_offsets():
    img = furrow_image(W / 2 + 70, W / 2 - 15)
    a, b = vision.furrow_line(img), vision.furrow_line(img[:, ::-1].copy())
    assert b.near == pytest.approx(-a.near, abs=0.02) and b.far == pytest.approx(-a.far, abs=0.02)


def test_steering_signs():
    """Furrow to the right: turn right (negative yaw rate); to the left: positive; centred: straight."""
    right = vision.row_command(vision.furrow_line(furrow_image(W / 2 + 80, W / 2 + 10)), 0.15, 1.4, 1.6, 0.35)
    left = vision.row_command(vision.furrow_line(furrow_image(W / 2 - 80, W / 2 - 10)), 0.15, 1.4, 1.6, 0.35)
    centred = vision.row_command(vision.furrow_line(furrow_image()), 0.15, 1.4, 1.6, 0.35)
    assert right[0] == left[0] == centred[0] == 0.15
    assert right[1] < -0.1 and left[1] > 0.1 and abs(centred[1]) < 0.05


def test_steering_law_value():
    f = vision.Furrow(near=0.2, far=-0.1, confidence=0.9)
    assert vision.row_command(f, 0.15, 1.4, 1.6, 0.35) == (0.15, pytest.approx(-(1.4 * 0.2 + 1.6 * -0.1)))


def test_low_confidence_stops_the_rover():
    f = vision.Furrow(near=0.5, far=0.5, confidence=0.2)
    assert vision.row_command(f, 0.15, 1.4, 1.6, 0.35) == (0.0, 0.0)
    assert vision.row_command(vision.Furrow(), 0.15, 1.4, 1.6, 0.0) == (0.15, 0.0)     # threshold 0 still follows


@pytest.mark.parametrize("name, img", [
    ("black", np.zeros((H, W, 3), np.uint8)),
    ("all soil", np.full((H, W, 3), SOIL, np.uint8)),
    ("all leaves", np.full((H, W, 3), LEAF, np.uint8)),
    ("all sky", np.full((H, W, 3), SKY, np.uint8)),
    ("white", np.full((H, W, 3), 255, np.uint8)),
])
def test_scenes_without_a_furrow_give_no_confidence(name, img):
    f = vision.furrow_line(img)
    assert f.confidence == 0.0 and f.near == 0.0 and f.far == 0.0


def test_narrow_slit_is_not_a_furrow():
    img = furrow_image(half_bottom=0.02 * W, half_horizon=0.01 * W)      # runs shorter than min_run
    assert vision.furrow_line(img).confidence == 0.0


def test_noise_does_not_make_a_confident_furrow():
    rng = np.random.default_rng(7)
    for _ in range(5):
        img = rng.integers(0, 256, (H, W, 3), dtype=np.uint8)
        assert vision.furrow_line(img).confidence < 0.35


def test_a_jagged_edge_lowers_confidence():
    clean = vision.furrow_line(furrow_image()).confidence
    img = furrow_image()
    rng = np.random.default_rng(3)
    for y in range(int(0.6 * H), H, 3):               # ragged soil edges: random shifts of the run
        shift = int(rng.integers(-35, 36))
        img[y] = np.roll(img[y], shift, axis=0)
    assert vision.furrow_line(img).confidence < clean


@pytest.mark.parametrize("img", [
    None, np.zeros((10, 10, 3), np.uint8), np.zeros((H, W), np.uint8), np.zeros((H, W, 3), np.float32),
    np.zeros((H, W, 4), np.uint8), np.zeros((0, 0, 3), np.uint8), "not an image", [[1, 2, 3]],
])
def test_unusable_frames_give_an_empty_result_not_an_exception(img):
    f = vision.furrow_line(img)
    assert (f.near, f.far, f.confidence) == (0.0, 0.0, 0.0)


def test_other_resolutions_give_the_same_offsets():
    big = cv2.resize(furrow_image(W / 2 + 60, W / 2), (1280, 960), interpolation=cv2.INTER_NEAREST)
    small = cv2.resize(furrow_image(W / 2 + 60, W / 2), (320, 240), interpolation=cv2.INTER_NEAREST)
    ref = vision.furrow_line(furrow_image(W / 2 + 60, W / 2))
    for img in (big, small):
        f = vision.furrow_line(img)
        assert f.near == pytest.approx(ref.near, abs=0.04) and f.far == pytest.approx(ref.far, abs=0.04)


def test_frame_with_a_whole_soil_row_in_the_band_is_skipped_not_fitted():
    img = furrow_image()
    img[int(0.8 * H):int(0.8 * H) + 2] = SOIL          # one full-width soil row: no beds visible there
    f = vision.furrow_line(img)
    assert f.confidence > 0.8 and abs(f.near) < 0.03


@pytest.mark.parametrize("name, tolerance", [("front_row_centred.png", 0.1), ("front_row_later.png", 0.1)])
def test_real_gazebo_frames_find_the_furrow(name, tolerance):
    """Regression on frames from the simulator's front camera, taken while the rover drove the row."""
    img = cv2.imread(str(DATA / name))
    assert img is not None
    f = vision.furrow_line(img)
    assert f.confidence > 0.5
    assert abs(f.near) < tolerance and abs(f.far) < tolerance
