"""ROS-level tests of the drive, row-follower and fruit-detector nodes, and of the parameter file.

Each test starts the node in-process on wall time, feeds it messages and reads what it
publishes. They need a sourced ROS 2 workspace and are skipped otherwise.
"""
import math
import pathlib
import time

import numpy as np
import pytest
import yaml

# Not imported from conftest: run together with agribot_gazebo's tests, `conftest` names the other package's.
PKG = pathlib.Path(__file__).resolve().parents[1]

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("agribot_interfaces.msg")

from agribot_interfaces.msg import RowState  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from geometry_msgs.msg import PoseArray, TransformStamped, Twist  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from agribot_autonomy import steering  # noqa: E402
from agribot_autonomy.drive_node import DriveNode  # noqa: E402
from agribot_autonomy.fruit_detector_node import FruitDetector  # noqa: E402
from agribot_autonomy.row_follower_node import RowFollower  # noqa: E402
from ros_harness import Harness  # noqa: E402

CONFIG = yaml.safe_load((PKG / "config/autonomy.yaml").read_text(encoding="utf-8"))
BRIDGE = CvBridge()


@pytest.fixture()
def h():
    harness = Harness()
    yield harness
    harness.close()


def image_msg(bgr, stamp=None, frame="cam"):
    msg = BRIDGE.cv2_to_imgmsg(bgr, "bgr8")
    msg.header.frame_id = frame
    if stamp is not None:
        msg.header.stamp = stamp
    return msg


# ------------------------------------------------------------- parameter file
NODES = {"drive_node": "agribot_autonomy.drive_node:DriveNode", "row_follower": "agribot_autonomy.row_follower_node:RowFollower",
         "fruit_detector": "agribot_autonomy.fruit_detector_node:FruitDetector",
         "harvest_mission": "agribot_autonomy.harvest_mission_node:Mission"}


def load(spec):
    module, cls = spec.split(":")
    return getattr(__import__(module, fromlist=[cls]), cls)


@pytest.mark.parametrize("name", list(NODES))
def test_node_starts_with_the_shipped_parameter_file_and_declares_every_key_in_it(h, name):
    yaml_params = CONFIG[name]["ros__parameters"]
    node = h.make(load(NODES[name]), **yaml_params)
    declared = set(node.get_parameters_by_prefix(""))
    assert set(yaml_params) <= declared, f"keys in the YAML that the node ignores: {set(yaml_params) - declared}"
    assert declared - {"use_sim_time"} <= set(yaml_params), \
        f"parameters the YAML does not mention: {declared - set(yaml_params) - {'use_sim_time'}}"


@pytest.mark.parametrize("name", list(NODES))
def test_node_starts_with_no_parameter_file(h, name):
    h.make(load(NODES[name]))                      # the declared defaults must stand on their own


@pytest.mark.parametrize("name, bad", [
    ("drive_node", {"wheelbase": -1.0}), ("drive_node", {"track": 0.0}), ("drive_node", {"steer_limit_deg": 95.0}),
    ("drive_node", {"max_speed": 0.0}), ("drive_node", {"cmd_timeout": 0.0}),
    ("row_follower", {"speed": -0.1}), ("row_follower", {"min_confidence": 1.5}), ("row_follower", {"k_near": -1.0}),
    ("fruit_detector", {"fruit_radius": 0.0}), ("fruit_detector", {"max_range": 99.0}),
    ("harvest_mission", {"row_length": 0.0}), ("harvest_mission", {"confirm_seconds": 0.0}),
    ("harvest_mission", {"cut_hold": 100.0}),
])
def test_out_of_range_parameters_stop_the_node_at_startup(h, name, bad):
    with pytest.raises(Exception) as err:
        h.make(load(NODES[name]), **bad)
    assert "range" in str(err.value).lower() or "invalid" in str(err.value).lower() or "parameter" in str(err.value).lower()


@pytest.mark.parametrize("name, bad", [
    ("drive_node", {"steering_geometry": "tricycle"}),
    ("row_follower", {"band_top": 0.9, "band_bottom": 0.5}),
    ("harvest_mission", {"drop_point": [1.0, 1.0, 0.8]}),             # not above the crate
    ("harvest_mission", {"reach_x": [0.2, 0.1]}),                     # inverted window
    ("harvest_mission", {"stop_x": [0.1, 0.0]}),
    ("harvest_mission", {"survey_point": [0.0, 0.0]}),                # wrong length
    ("harvest_mission", {"crate_half": [0.0, 0.1]}),
])
def test_inconsistent_parameters_stop_the_node_with_a_reason(h, name, bad):
    with pytest.raises((ValueError, Exception)):
        h.make(load(NODES[name]), **bad)


def test_a_running_node_refuses_a_bad_parameter_and_keeps_the_old_value(h):
    node = h.make(RowFollower)
    result = node.set_parameters([Parameter("speed", value=5.0)])[0]
    assert not result.successful
    assert node.get_parameter("speed").value == 0.15
    assert node.set_parameters([Parameter("speed", value=0.2)])[0].successful


# ------------------------------------------------------------------ drive node
def test_drive_node_publishes_the_planned_wheel_speeds_and_steering(h):
    node = h.make(DriveNode, **CONFIG["drive_node"]["ros__parameters"])
    wheels = h.collect(Float64MultiArray, "/wheel_controller/commands")
    steer = h.collect(Float64MultiArray, "/steering_controller/commands")
    pub = h.node.create_publisher(Twist, "/cmd_vel", 10)
    h.wait_for(lambda: pub.get_subscription_count() > 0)
    msg = Twist()
    msg.linear.x, msg.angular.z = 0.3, 0.25
    pub.publish(msg)
    assert h.wait_for(lambda: wheels and steer)
    want = steering.plan(node.geom, 0.3, 0.25)
    assert list(wheels[0].data) == pytest.approx(list(want.wheel_omega))
    assert list(steer[0].data) == pytest.approx([want.steer_left, want.steer_right])


def test_drive_node_limits_speed_and_yaw_rate(h):
    h.make(DriveNode, max_speed=0.5, max_yaw_rate=0.2)
    wheels = h.collect(Float64MultiArray, "/wheel_controller/commands")
    pub = h.node.create_publisher(Twist, "/cmd_vel", 10)
    h.wait_for(lambda: pub.get_subscription_count() > 0)
    msg = Twist()
    msg.linear.x = 3.0
    pub.publish(msg)
    assert h.wait_for(lambda: wheels)
    assert max(wheels[0].data) == pytest.approx(0.5 / 0.19)


@pytest.mark.parametrize("v, w", [(float("nan"), 0.0), (0.2, float("nan")), (float("inf"), 0.0), (0.2, -float("inf"))])
def test_drive_node_ignores_non_finite_commands(h, v, w):
    h.make(DriveNode)
    wheels = h.collect(Float64MultiArray, "/wheel_controller/commands")
    pub = h.node.create_publisher(Twist, "/cmd_vel", 10)
    h.wait_for(lambda: pub.get_subscription_count() > 0)
    msg = Twist()
    msg.linear.x, msg.angular.z = v, w
    pub.publish(msg)
    time.sleep(0.4)
    assert wheels == []


def test_drive_node_stops_the_wheels_when_commands_stop(h):
    h.make(DriveNode, cmd_timeout=0.3)
    wheels = h.collect(Float64MultiArray, "/wheel_controller/commands")
    pub = h.node.create_publisher(Twist, "/cmd_vel", 10)
    h.wait_for(lambda: pub.get_subscription_count() > 0)
    msg = Twist()
    msg.linear.x = 0.3
    pub.publish(msg)
    assert h.wait_for(lambda: len(wheels) >= 1)
    assert h.wait_for(lambda: len(wheels) >= 2, timeout=2.0), "no stop command after the timeout"
    assert all(abs(w) < 1e-12 for w in wheels[-1].data)
    count = len(wheels)
    time.sleep(0.6)
    assert len(wheels) == count, "the stop must be sent once, not repeated"


def test_drive_node_keeps_publishing_while_commands_keep_coming(h):
    h.make(DriveNode, cmd_timeout=0.5)
    wheels = h.collect(Float64MultiArray, "/wheel_controller/commands")
    pub = h.node.create_publisher(Twist, "/cmd_vel", 10)
    h.wait_for(lambda: pub.get_subscription_count() > 0)
    msg = Twist()
    msg.linear.x = 0.3
    for _ in range(15):
        pub.publish(msg)
        time.sleep(0.1)
    assert all(max(w.data) > 0 for w in wheels), "a stop was sent during an active command stream"


def test_drive_node_holds_the_steering_angle_through_a_stop(h):
    h.make(DriveNode)
    steer = h.collect(Float64MultiArray, "/steering_controller/commands")
    pub = h.node.create_publisher(Twist, "/cmd_vel", 10)
    h.wait_for(lambda: pub.get_subscription_count() > 0)
    msg = Twist()
    msg.linear.x, msg.angular.z = 0.3, 0.3
    pub.publish(msg)
    assert h.wait_for(lambda: len(steer) >= 1)
    turning = list(steer[0].data)
    pub.publish(Twist())
    assert h.wait_for(lambda: len(steer) >= 2)
    assert list(steer[1].data) == pytest.approx(turning)


# ---------------------------------------------------------------- row follower
W, H = 640, 480


def furrow_frame(shift=0.0):
    from test_vision_row import furrow_image
    return furrow_image(W / 2 + shift, W / 2)


def test_row_follower_publishes_state_for_every_image_even_when_disabled(h):
    h.make(RowFollower)
    states, cmds = h.collect(RowState, "/row_follower/state"), h.collect(Twist, "/cmd_vel")
    pub = h.node.create_publisher(Image, "/front_camera/image", qos_profile_sensor_data)
    h.wait_for(lambda: pub.get_subscription_count() > 0)
    pub.publish(image_msg(furrow_frame(70)))
    assert h.wait_for(lambda: states)
    s = states[0]
    assert s.tracking and s.confidence > 0.9 and s.near > 0.15 and s.header.frame_id == "cam"
    time.sleep(0.3)
    assert cmds == [], "a disabled follower must not command the rover"


def test_row_follower_steers_toward_the_furrow_when_enabled(h):
    h.make(RowFollower, speed=0.2, k_near=1.0, k_far=1.0)
    cmds = h.collect(Twist, "/cmd_vel")
    state = h.collect(RowState, "/row_follower/state")
    img = h.node.create_publisher(Image, "/front_camera/image", qos_profile_sensor_data)
    en = h.node.create_publisher(Bool, "/row_follower/enable", 10)
    h.wait_for(lambda: img.get_subscription_count() > 0 and en.get_subscription_count() > 0)
    en.publish(Bool(data=True))
    time.sleep(0.2)
    img.publish(image_msg(furrow_frame(80)))
    assert h.wait_for(lambda: cmds and state)
    assert cmds[-1].linear.x == pytest.approx(0.2)
    assert cmds[-1].angular.z == pytest.approx(-(state[0].near + state[0].far), abs=1e-5) and cmds[-1].angular.z < -0.1
    cmds.clear()
    img.publish(image_msg(furrow_frame(-80)))
    assert h.wait_for(lambda: cmds)
    assert cmds[-1].angular.z > 0.1


def test_row_follower_stops_when_disabled_and_when_the_furrow_is_lost(h):
    h.make(RowFollower)
    cmds = h.collect(Twist, "/cmd_vel")
    img = h.node.create_publisher(Image, "/front_camera/image", qos_profile_sensor_data)
    en = h.node.create_publisher(Bool, "/row_follower/enable", 10)
    h.wait_for(lambda: img.get_subscription_count() > 0 and en.get_subscription_count() > 0)
    en.publish(Bool(data=True))
    time.sleep(0.2)
    img.publish(image_msg(furrow_frame()))
    assert h.wait_for(lambda: cmds and cmds[-1].linear.x > 0)
    img.publish(image_msg(np.zeros((H, W, 3), np.uint8)))                   # furrow lost
    assert h.wait_for(lambda: cmds[-1].linear.x == 0.0)
    img.publish(image_msg(furrow_frame()))
    assert h.wait_for(lambda: cmds[-1].linear.x > 0)
    n = len(cmds)
    en.publish(Bool(data=False))
    assert h.wait_for(lambda: len(cmds) > n and cmds[-1].linear.x == 0.0 and cmds[-1].angular.z == 0.0)


def test_row_follower_stops_when_the_camera_goes_quiet(h):
    h.make(RowFollower, image_timeout=0.5)
    cmds = h.collect(Twist, "/cmd_vel")
    img = h.node.create_publisher(Image, "/front_camera/image", qos_profile_sensor_data)
    en = h.node.create_publisher(Bool, "/row_follower/enable", 10)
    h.wait_for(lambda: img.get_subscription_count() > 0 and en.get_subscription_count() > 0)
    en.publish(Bool(data=True))
    time.sleep(0.2)
    img.publish(image_msg(furrow_frame()))
    assert h.wait_for(lambda: cmds and cmds[-1].linear.x > 0)
    assert h.wait_for(lambda: cmds[-1].linear.x == 0.0, timeout=3.0), "still driving on a stale image"
    n = len(cmds)
    time.sleep(1.0)
    assert len(cmds) == n, "the stop must be published once"


def test_row_follower_survives_corrupt_images(h):
    h.make(RowFollower)
    states = h.collect(RowState, "/row_follower/state")
    img = h.node.create_publisher(Image, "/front_camera/image", qos_profile_sensor_data)
    h.wait_for(lambda: img.get_subscription_count() > 0)
    bad = Image(height=480, width=640, encoding="bgr8", step=640 * 3, data=[0] * 100)        # truncated data
    img.publish(bad)
    odd = Image(height=4, width=4, encoding="not_an_encoding", step=4, data=[0] * 16)
    img.publish(odd)
    time.sleep(0.3)
    img.publish(image_msg(furrow_frame()))
    assert h.wait_for(lambda: states), "the node died or stopped after bad images"
    assert states[-1].confidence > 0.9


def test_row_follower_accepts_a_greyscale_image_without_crashing(h):
    h.make(RowFollower)
    states = h.collect(RowState, "/row_follower/state")
    img = h.node.create_publisher(Image, "/front_camera/image", qos_profile_sensor_data)
    h.wait_for(lambda: img.get_subscription_count() > 0)
    grey = BRIDGE.cv2_to_imgmsg(np.full((H, W), 128, np.uint8), "mono8")
    img.publish(grey)
    assert h.wait_for(lambda: states)
    assert states[0].confidence == 0.0 and not states[0].tracking


def test_row_follower_debug_image_only_when_someone_listens(h):
    h.make(RowFollower)
    img = h.node.create_publisher(Image, "/front_camera/image", qos_profile_sensor_data)
    h.wait_for(lambda: img.get_subscription_count() > 0)
    img.publish(image_msg(furrow_frame()))
    time.sleep(0.3)
    debug = h.collect(Image, "/row_follower/debug_image")
    h.wait_for(lambda: True)
    time.sleep(0.5)
    img.publish(image_msg(furrow_frame()))
    assert h.wait_for(lambda: debug)
    assert (debug[0].height, debug[0].width) == (H, W)


# -------------------------------------------------------------- fruit detector
FX = FY = 554.0
K = [FX, 0.0, 320.0, 0.0, FY, 240.0, 0.0, 0.0, 1.0]
# Camera optical frame (x right, y down, z forward) in the rover frame: a camera looking along -y with its top up.
OPT_IN_BASE = np.column_stack([[-1, 0, 0], [0, 0, -1], [0, -1, 0]])            # columns: optical x, y, z in base axes
CAM_POS = np.array([0.05, -0.1, 0.8])


def quaternion_of(R):
    """(x, y, z, w) of a rotation matrix (Shepperd's method: stable for every rotation, including 180 degrees)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        w = math.sqrt(1 + t) / 2
        return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s_ = math.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0]
    q[i] = s_ / 4
    q[j] = (R[j, i] + R[i, j]) / s_
    q[k] = (R[k, i] + R[i, k]) / s_
    return (q[0], q[1], q[2], (R[k, j] - R[j, k]) / s_)


def publish_camera_tf(h, frame="arm_cam_optical"):
    br = StaticTransformBroadcaster(h.node)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "base_footprint", frame
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, CAM_POS)
    q = quaternion_of(OPT_IN_BASE)
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
    br.sendTransform(tf)
    return br


def detector_inputs(h):
    info = h.node.create_publisher(CameraInfo, "/arm_camera/camera_info", qos_profile_sensor_data)
    rgb = h.node.create_publisher(Image, "/arm_camera/image", qos_profile_sensor_data)
    depth = h.node.create_publisher(Image, "/arm_camera/depth_image", qos_profile_sensor_data)
    return info, rgb, depth


def send_frame(h, info, rgb, depth, bgr, depth_img, frame="arm_cam_optical", with_info=True):
    stamp = h.node.get_clock().now().to_msg()
    if with_info:
        ci = CameraInfo(k=K, width=W, height=H)
        ci.header.stamp, ci.header.frame_id = stamp, frame
        info.publish(ci)
        time.sleep(0.05)
    rgb.publish(image_msg(bgr, stamp, frame))
    dm = BRIDGE.cv2_to_imgmsg(depth_img.astype(np.float32), "32FC1")
    dm.header.stamp, dm.header.frame_id = stamp, frame
    depth.publish(dm)


def test_fruit_detector_reports_the_fruit_in_the_rover_frame(h):
    from test_vision_fruit import render_sphere
    h.make(FruitDetector)
    keep = publish_camera_tf(h)                                                # noqa: F841
    info, rgb, depth = detector_inputs(h)
    poses = h.collect(PoseArray, "/fruit/detections")
    h.wait_for(lambda: rgb.get_subscription_count() > 0 and depth.get_subscription_count() > 0)
    world = np.array([0.12, -0.50, 0.83])                                       # fruit centre in the rover frame
    in_cam = OPT_IN_BASE.T @ (world - CAM_POS)
    bgr, d = render_sphere(in_cam)
    for _ in range(4):
        send_frame(h, info, rgb, depth, bgr, d)
        time.sleep(0.15)
    assert h.wait_for(lambda: any(p.poses for p in poses), timeout=8.0)
    found = next(p for p in poses if p.poses)
    assert found.header.frame_id == "base_footprint"
    p = found.poses[0].position
    assert np.linalg.norm([p.x - world[0], p.y - world[1], p.z - world[2]]) < 0.003


def test_fruit_detector_publishes_an_empty_list_when_nothing_is_red(h):
    from test_vision_fruit import LEAF
    h.make(FruitDetector)
    keep = publish_camera_tf(h)                                                # noqa: F841
    info, rgb, depth = detector_inputs(h)
    poses = h.collect(PoseArray, "/fruit/detections")
    h.wait_for(lambda: rgb.get_subscription_count() > 0)
    for _ in range(3):
        send_frame(h, info, rgb, depth, np.full((H, W, 3), LEAF, np.uint8), np.full((H, W), 0.5))
        time.sleep(0.15)
    assert h.wait_for(lambda: poses, timeout=8.0)
    assert all(not p.poses for p in poses)


def test_fruit_detector_publishes_nothing_without_camera_info_or_a_transform(h):
    from test_vision_fruit import render_sphere
    h.make(FruitDetector, tf_timeout=0.1)
    info, rgb, depth = detector_inputs(h)
    poses = h.collect(PoseArray, "/fruit/detections")
    h.wait_for(lambda: rgb.get_subscription_count() > 0)
    bgr, d = render_sphere((0, 0, 0.4))
    for _ in range(3):
        send_frame(h, info, rgb, depth, bgr, d, with_info=False)                # no camera_info
        time.sleep(0.15)
    for _ in range(3):
        send_frame(h, info, rgb, depth, bgr, d)                                 # camera_info, but no TF
        time.sleep(0.3)
    time.sleep(0.5)
    assert poses == [], "a detection was published without a valid transform"


def test_fruit_detector_survives_garbage_images(h):
    from test_vision_fruit import render_sphere
    h.make(FruitDetector)
    keep = publish_camera_tf(h)                                                # noqa: F841
    info, rgb, depth = detector_inputs(h)
    poses = h.collect(PoseArray, "/fruit/detections")
    h.wait_for(lambda: rgb.get_subscription_count() > 0)
    stamp = h.node.get_clock().now().to_msg()
    ci = CameraInfo(k=K)
    ci.header.stamp = stamp
    info.publish(ci)
    for img in (Image(height=480, width=640, encoding="bgr8", step=1920, data=[0] * 10),
                Image(height=2, width=2, encoding="weird", step=2, data=[0] * 4)):
        img.header.stamp = stamp
        img.header.frame_id = "arm_cam_optical"
        rgb.publish(img)
        depth.publish(img)
    time.sleep(0.5)
    bgr, d = render_sphere((0.0, 0.0, 0.4))
    # depth image of the wrong size for the colour image: no detection, no crash
    for _ in range(3):
        send_frame(h, info, rgb, depth, bgr, d[:100, :100])
        time.sleep(0.15)
    for _ in range(3):
        send_frame(h, info, rgb, depth, bgr, d)
        time.sleep(0.15)
    assert h.wait_for(lambda: any(p.poses for p in poses), timeout=8.0), "the node stopped working after bad input"


def test_fruit_detector_uses_the_newest_transform_for_an_image_slightly_ahead_of_tf_but_not_a_stale_one(h):
    """TF arrives at the joint-state rate, so a camera stamp can be a little newer than the latest transform."""
    from tf2_ros import TransformBroadcaster
    from test_vision_fruit import render_sphere
    h.make(FruitDetector, tf_max_age=0.3, tf_timeout=0.05)
    info, rgb, depth = detector_inputs(h)
    poses = h.collect(PoseArray, "/fruit/detections")
    br = TransformBroadcaster(h.node)
    h.wait_for(lambda: rgb.get_subscription_count() > 0)
    world = np.array([0.12, -0.50, 0.83])
    bgr, d = render_sphere(OPT_IN_BASE.T @ (world - CAM_POS))

    def broadcast():
        tf = TransformStamped()
        tf.header.stamp = h.node.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = "base_footprint", "arm_cam_optical"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, CAM_POS)
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = quaternion_of(OPT_IN_BASE)
        br.sendTransform(tf)
        return tf.header.stamp

    def frame_at(offset_s):
        now = h.node.get_clock().now()
        stamp = (now + rclpy.duration.Duration(seconds=offset_s)).to_msg()
        ci = CameraInfo(k=K)
        ci.header.stamp = stamp
        info.publish(ci)
        time.sleep(0.05)
        rgb.publish(image_msg(bgr, stamp, "arm_cam_optical"))
        dm = BRIDGE.cv2_to_imgmsg(d.astype(np.float32), "32FC1")
        dm.header.stamp, dm.header.frame_id = stamp, "arm_cam_optical"
        depth.publish(dm)

    for _ in range(3):
        broadcast()
        time.sleep(0.1)
    broadcast()
    frame_at(0.08)                                                       # 80 ms newer than the transform
    assert h.wait_for(lambda: any(p.poses for p in poses), timeout=5.0), "a slightly newer image was dropped"
    p = next(p for p in poses if p.poses).poses[0].position
    assert np.linalg.norm([p.x - world[0], p.y - world[1], p.z - world[2]]) < 0.003

    poses.clear()
    broadcast()
    frame_at(2.0)                                                        # 2 s newer: the transform is stale
    time.sleep(1.0)
    assert not any(p.poses for p in poses), "a detection was made with a stale transform"
