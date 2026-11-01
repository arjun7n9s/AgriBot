"""Configuration consistency: the bridge, the controllers, the world and the nodes agree.

Pure file checks, so they run without ROS or Gazebo. Each guards a mismatch that would
otherwise show up only as a silent failure in a simulation run.
"""
import json
import math
import pathlib
import re

import numpy as np
import pytest
import yaml

from agribot_gazebo import cut_model

PKG = pathlib.Path(__file__).resolve().parents[1]
SIM = PKG.parent
AUTONOMY = yaml.safe_load((SIM / "agribot_autonomy/config/autonomy.yaml").read_text(encoding="utf-8"))
CONTROLLERS = yaml.safe_load((PKG / "config/controllers.yaml").read_text(encoding="utf-8"))
BRIDGE = yaml.safe_load((PKG / "config/bridge.yaml").read_text(encoding="utf-8"))
TARGETS = json.loads((PKG / "worlds/demo_row_targets.json").read_text(encoding="utf-8"))
WORLD = (PKG / "worlds/demo_row.sdf").read_text(encoding="utf-8")


def node_text(path):
    return (SIM / path).read_text(encoding="utf-8")


# ------------------------------------------------------------------ bridge
def test_bridge_provides_every_topic_the_autonomy_nodes_subscribe_to():
    provided = {e["ros_topic_name"] for e in BRIDGE}
    for topic in ("/front_camera/image", "/arm_camera/image", "/arm_camera/depth_image", "/arm_camera/camera_info",
                  "/odom", "/clock"):
        assert topic in provided, topic


def test_bridge_entries_are_complete_and_unique():
    names = [e["ros_topic_name"] for e in BRIDGE]
    assert len(names) == len(set(names))
    for e in BRIDGE:
        assert {"ros_topic_name", "gz_topic_name", "ros_type_name", "gz_type_name", "direction"} <= e.keys()
        assert e["direction"] in ("GZ_TO_ROS", "ROS_TO_GZ", "BIDIRECTIONAL")


# ------------------------------------------------------------- controllers
def test_controller_names_match_the_command_topics_the_nodes_use():
    declared = set(CONTROLLERS["controller_manager"]["ros__parameters"])
    for topic_owner in ("wheel_controller", "steering_controller", "arm_controller", "tool_controller", "scout_controller"):
        assert topic_owner in declared
    drive = node_text("agribot_autonomy/agribot_autonomy/drive_node.py")
    mission = node_text("agribot_autonomy/agribot_autonomy/harvest_mission_node.py")
    for name in re.findall(r'"/(\w+_controller)/', drive + mission):
        assert name in declared, name


def test_controlled_joints_exist_in_the_urdf(robot):
    for name, cfg in CONTROLLERS.items():
        joints = cfg.get("ros__parameters", {}).get("joints") if isinstance(cfg, dict) else None
        for j in joints or []:
            assert j in robot.joints, f"{name}: {j}"


def test_wheel_controller_order_is_front_left_front_right_rear_left_rear_right():
    """drive_node writes [fl, fr, rl, rr]; the controller's joint order must match it."""
    assert CONTROLLERS["wheel_controller"]["ros__parameters"]["joints"] == [
        "front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"]


def test_steering_controller_order_is_left_then_right():
    assert CONTROLLERS["steering_controller"]["ros__parameters"]["joints"] == ["steer_joint", "steer_right_joint"]


def test_tool_controller_order_is_blade_then_door():
    assert CONTROLLERS["tool_controller"]["ros__parameters"]["joints"] == [
        "scara_cutter_blade_joint", "scara_cutter_door_joint"]


def test_left_steering_joint_is_on_the_left(robot):
    left, right = robot.joints["steer_joint"], robot.joints["steer_right_joint"]
    assert left["origin"][1, 3] > 0 > right["origin"][1, 3]


def test_arm_controller_joint_order_matches_the_mission():
    assert CONTROLLERS["arm_controller"]["ros__parameters"]["joints"] == ["scara_j1", "scara_z", "scara_j2", "scara_j3"]


# ----------------------------------------------------------------- cut model
def test_blade_trigger_thresholds_sit_between_open_and_closed():
    p = AUTONOMY["harvest_mission"]["ros__parameters"]
    trig = cut_model.BladeTrigger()           # the simulator node's defaults
    assert p["blade_closed"] < trig.closed_below < trig.reopened_above < p["blade_open"]


def test_sim_crate_defaults_match_the_mission_crate():
    p = AUTONOMY["harvest_mission"]["ros__parameters"]
    box = cut_model.CrateBox()
    assert tuple(p["crate_centre"]) == box.centre and tuple(p["crate_half"]) == box.half


def test_crate_floor_and_rim_match_the_urdf(robot):
    """The crate box heights, in the rover frame, come from the URDF crate's floor and wall tops."""
    t = robot.fk_frame({}, "harvest_crate")
    box = cut_model.CrateBox()
    assert t[2, 3] + 0.008 == pytest.approx(box.floor_z, abs=0.005)          # top of the 8 mm floor plate
    assert t[2, 3] + 0.2 == pytest.approx(box.rim_z, abs=0.01)               # top of the 0.2 m walls


def test_the_cut_point_drops_into_the_crate_footprint():
    p = AUTONOMY["harvest_mission"]["ros__parameters"]
    box = cut_model.CrateBox()
    assert box.contains((p["drop_point"][0], p["drop_point"][1], 0.6))


# -------------------------------------------------------------------- world
def test_every_fruit_entry_is_well_formed_and_unique():
    fruit = TARGETS["fruit"]
    assert len(fruit) > 20
    names = [f["name"] for f in fruit]
    assert len(names) == len(set(names))
    for f in fruit:
        assert f["kind"] in ("tomato_ripe", "tomato_half", "tomato_green")
        for key in ("knuckle", "rpy", "cut_point"):
            assert len(f[key]) == 3 and all(math.isfinite(v) for v in f[key]), (f["name"], key)
        assert np.linalg.norm(np.array(f["knuckle"]) - np.array(f["cut_point"])) < 0.05
        assert f["name"] in WORLD, f["name"]


def test_fruit_models_carry_the_collision_group_the_cut_rewrites():
    """sim_cut_model turns the on-plant group 0x04 into the catchable group 0x02 by replacing that text."""
    for kind in {f["kind"] for f in TARGETS["fruit"]}:
        sdf = (SIM / "agribot_field/models" / kind / "model.sdf").read_text(encoding="utf-8")
        assert "0x04" in sdf, kind


def test_ripe_fruit_stand_where_the_rover_can_reach_them():
    """A reachable ripe fruit exists beside the rover's path: the furrow is at y = furrow_y."""
    furrow_y = TARGETS["furrow_y"]
    near = [f for f in TARGETS["fruit"] if f["kind"] == "tomato_ripe" and 0.3 < abs(f["cut_point"][1] - furrow_y) < 0.7]
    assert len(near) >= 5


def test_the_note_in_the_targets_file_is_not_stale():
    assert "not in the loop" not in TARGETS["note"]
