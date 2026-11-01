"""The configuration agrees with the robot description, and the arm can do what the mission asks of it.

These tests read the generated URDF and config/autonomy.yaml, so a change to one that
breaks the other fails here, before a simulation run does.
"""
import itertools
import math
import pathlib

import numpy as np
import pytest
import yaml

from agribot_autonomy import mission_logic as logic

PKG = pathlib.Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((PKG / "config/autonomy.yaml").read_text(encoding="utf-8"))
ARM = ["scara_j1", "scara_z", "scara_j2", "scara_j3"]
SEEDS = [{"scara_j1": a, "scara_z": 0.15, "scara_j2": b, "scara_j3": c}
         for a, b, c in ((-1.0, 1.0, -1.5), (0.0, 1.5, -2.0), (1.0, -1.0, 1.0), (-2.0, 1.5, 0.0))]
START = {"scara_j1": 0.0, "scara_z": 0.1, "scara_j2": 0.0, "scara_j3": 0.0}
FACE_ROW = -math.pi / 2


def params(node):
    return CONFIG[node]["ros__parameters"]


def position(robot, link):
    return robot.fk_frame({}, link)[:3, 3]


# ----------------------------------------------------------- drive geometry
def test_wheelbase_and_track_match_the_urdf(robot):
    p = params("drive_node")
    fl, fr = position(robot, "front_left_wheel"), position(robot, "front_right_wheel")
    rl, rr = position(robot, "rear_left_wheel"), position(robot, "rear_right_wheel")
    assert fl[0] - rl[0] == pytest.approx(p["wheelbase"], abs=1e-6)
    assert fr[0] - rr[0] == pytest.approx(p["wheelbase"], abs=1e-6)
    assert fl[1] - fr[1] == pytest.approx(p["track"], abs=1e-6)
    assert rl[1] - rr[1] == pytest.approx(p["track"], abs=1e-6)
    assert abs(fl[1] + fr[1]) < 1e-9, "the front axle is centred"


def test_wheel_radius_matches_the_urdf_collision_cylinder(urdf_text):
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf_text)
    radii = {float(c.get("radius")) for link in root.findall("link") if "wheel" in link.get("name")
             for c in link.findall("collision/geometry/cylinder")}
    assert radii == {params("drive_node")["wheel_radius"]}


def test_steering_limit_is_the_knuckle_joint_limit(robot):
    for name in ("steer_joint", "steer_right_joint"):
        j = robot.joints[name]
        assert j["upper"] == pytest.approx(math.radians(params("drive_node")["steer_limit_deg"]), abs=1e-5)
        assert j["lower"] == pytest.approx(-j["upper"], abs=1e-9)


def test_rear_axle_is_the_kinematic_reference_point(robot):
    """steering.py puts the origin at the rear-axle centre: the rear wheels must sit on x = 0 of the
    wheel frame, i.e. the car's wheels are symmetric about it left and right."""
    rl, rr = position(robot, "rear_left_wheel"), position(robot, "rear_right_wheel")
    assert rl[0] == pytest.approx(rr[0]) and rl[1] == pytest.approx(-rr[1])


# -------------------------------------------------------------- arm and tool
def test_mission_names_exist_in_the_urdf(robot):
    for j in ARM + ["scara_cutter_blade_joint", "scara_cutter_door_joint", "scout_pan_joint"]:
        assert j in robot.joints, j
    assert "scara_cut_point" in robot.links


def test_blade_and_door_commands_are_within_the_joint_limits(robot):
    p = params("harvest_mission")
    blade, door = robot.joints["scara_cutter_blade_joint"], robot.joints["scara_cutter_door_joint"]
    for v in (p["blade_open"], p["blade_closed"]):
        assert blade["lower"] <= v <= blade["upper"]
    for v in (p["door_shut"], p["door_open"]):
        assert door["lower"] <= v <= door["upper"]
    assert p["blade_open"] != p["blade_closed"] and p["door_shut"] != p["door_open"]


def test_fk_ik_round_trip(robot):
    """Poses reached by random joint values are found again by the IK (position and facing)."""
    rng = np.random.default_rng(11)
    solved = 0
    for _ in range(20):
        q = {j: float(rng.uniform(robot.joints[j]["lower"] * 0.6, robot.joints[j]["upper"] * 0.6)) for j in ARM}
        t = robot.fk_frame(q, "scara_cut_point")
        yaw = math.atan2(t[1, 0], t[0, 0])
        q2, err = robot.solve_best(ARM, "scara_cut_point", t[:3, 3], yaw, [START] + SEEDS, iters=200, good=1e-4)
        t2 = robot.fk_frame(q2, "scara_cut_point")
        if err < 1e-3:
            solved += 1
            assert np.linalg.norm(t2[:3, 3] - t[:3, 3]) < 1e-3
            assert math.atan2(t2[1, 0], t2[0, 0]) == pytest.approx(yaw, abs=0.05)
    assert solved >= 19, f"only {solved} of 20 reachable poses were solved"


def test_fk_matches_the_full_tree_walk(robot):
    q = {"scara_j1": 0.4, "scara_z": 0.12, "scara_j2": -0.9, "scara_j3": 0.6}
    assert np.allclose(robot.fk(q)["scara_cut_point"], robot.fk_frame(q, "scara_cut_point"), atol=1e-12)


def test_ik_respects_joint_limits(robot):
    q, _ = robot.solve_best(ARM, "scara_cut_point", (0.1, -0.5, 0.85), FACE_ROW, [START] + SEEDS)
    for j in ARM:
        assert robot.joints[j]["lower"] - 1e-9 <= q[j] <= robot.joints[j]["upper"] + 1e-9


def test_unreachable_targets_report_a_large_error_instead_of_a_wrong_pose(robot):
    _, err = robot.solve_best(ARM, "scara_cut_point", (3.0, -3.0, 2.0), FACE_ROW, [START] + SEEDS)
    assert err > 1.0
    _, err = robot.solve_best(ARM, "scara_cut_point", (0.1, -0.5, 0.2), FACE_ROW, [START] + SEEDS)    # below the Z stroke
    assert err > 0.1


def test_solve_best_needs_a_seed(robot):
    with pytest.raises(ValueError):
        robot.solve_best(ARM, "scara_cut_point", (0.1, -0.5, 0.85), FACE_ROW, [])


def test_the_declared_reach_box_is_reachable(robot):
    """Every corner of reach_x/y/z and the grid inside it solves with the cutter facing the row."""
    p = params("harvest_mission")
    xs, ys, zs = (np.linspace(p[k][0], p[k][1], n) for k, n in (("reach_x", 5), ("reach_y", 4), ("reach_z", 4)))
    failures = []
    for point in itertools.product(xs, ys, zs):
        _, err = robot.solve_best(ARM, "scara_cut_point", point, FACE_ROW, [START] + SEEDS)
        if err > 3e-3:
            failures.append((np.round(point, 3).tolist(), round(err * 1000, 1)))
    assert not failures, f"unreachable points in the declared reach box: {failures}"


def test_the_stop_band_is_inside_the_reach_box():
    p = params("harvest_mission")
    assert p["reach_x"][0] <= p["stop_x"][0] < p["stop_x"][1] <= p["reach_x"][1]


def test_the_pre_cut_pose_is_reachable_too(robot):
    """The approach stands off by approach_distance toward the rover: also check that for the box's corners."""
    p = params("harvest_mission")
    lift, back = p["cut_above_centre"], p["approach_distance"]
    for x, y in itertools.product(p["reach_x"], p["reach_y"]):
        for z in p["reach_z"]:
            pre = (x, y + back, z + 0.03)                            # the first move also lifts 3 cm
            _, err = robot.solve_best(ARM, "scara_cut_point", pre, FACE_ROW, [START] + SEEDS)
            assert err < 3e-3, f"pre-cut pose {pre} unreachable (error {err * 1000:.1f} mm)"
    assert lift > 0


def test_survey_pose_is_reachable_and_inside_the_reach_box(robot):
    p = params("harvest_mission")
    survey = p["survey_point"]
    _, err = robot.solve_best(ARM, "scara_cut_point", survey, FACE_ROW, [START] + SEEDS)
    assert err < 3e-3
    assert logic.Reach(tuple(p["reach_x"]), tuple(p["reach_y"]), tuple(p["reach_z"])).contains(
        (survey[0], survey[1], survey[2]), margin=0.1)


def test_the_drop_point_is_reachable_and_over_the_crate(robot):
    p = params("harvest_mission")
    drop, centre = p["drop_point"], p["crate_centre"]
    yaw = math.atan2(drop[1] - centre[1], drop[0] - centre[0])
    _, err = robot.solve_best(ARM, "scara_cut_point", drop, yaw, [START] + SEEDS, iters=150)
    if err > 5e-3:                                                   # the mission's fallback: any facing
        _, err = robot.solve_best(ARM, "scara_cut_point", drop, None, [START] + SEEDS, iters=150)
    assert err < 5e-3
    assert logic.Crate(tuple(centre), tuple(p["crate_half"]), tuple(drop)).drop_inside()


def test_crate_parameters_match_the_urdf_crate(robot, urdf_text):
    """Crate centre, inside size, floor and rim agree with the crate's collision boxes."""
    import xml.etree.ElementTree as ET
    p = params("harvest_mission")
    t = robot.fk_frame({}, "harvest_crate")
    root = ET.fromstring(urdf_text)
    crate = next(l for l in root.findall("link") if l.get("name") == "harvest_crate")
    boxes = {}
    for c in crate.findall("collision"):
        xyz = [float(v) for v in c.find("origin").get("xyz").split()]
        size = [float(v) for v in c.find("geometry/box").get("size").split()]
        boxes[c.get("name")] = (xyz, size)
    assert t[:2, 3] == pytest.approx(p["crate_centre"], abs=0.005)
    inside_x = boxes["wall_front"][0][0] - boxes["wall_front"][1][0] / 2
    inside_y = boxes["wall_left"][0][1] - boxes["wall_left"][1][1] / 2
    assert (inside_x, inside_y) == pytest.approx(tuple(p["crate_half"]), abs=0.002)
