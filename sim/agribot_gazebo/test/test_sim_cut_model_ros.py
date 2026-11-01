"""sim_cut_model as a ROS node, without Gazebo: a fake blade, TF and odometry drive it.

Gazebo's services are not available, so the node logs that its spawn calls were not confirmed;
what is tested is the decision logic around them: when it cuts, which fruit, how often, and
what /crate/verify says.
"""
import importlib.util
import json
import os
import pathlib
import time

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
os.environ.setdefault("ROS_DOMAIN_ID", "86")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("agribot_interfaces.srv")
pytest.importorskip("agribot_gazebo")

from agribot_interfaces.srv import VerifyPick  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

try:
    from ament_index_python.packages import get_package_share_directory
    SHARE = pathlib.Path(get_package_share_directory("agribot_gazebo"))
except Exception:                                   # noqa: BLE001
    pytest.skip("agribot_gazebo is not installed", allow_module_level=True)

spec = importlib.util.spec_from_file_location("sim_cut_model", PKG / "scripts/sim_cut_model.py")
sim_cut_model = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sim_cut_model)

FRUIT = json.loads((PKG / "worlds/demo_row_targets.json").read_text())["fruit"]
TARGET = next(f for f in FRUIT if f["kind"] == "tomato_ripe")


class Rig:
    def __init__(self, throat=None, **params):
        params = {"service_timeout": 0.3, **params}                 # no Gazebo here: fail fast
        rclpy.init()
        self.node = Node("rig")
        try:
            self.model = sim_cut_model.SimCutModel(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        except Exception:
            self.node.destroy_node()
            rclpy.shutdown()                                  # the node refused to start: leave no context behind
            raise
        self.ex = MultiThreadedExecutor(num_threads=6)
        self.ex.add_node(self.node)
        self.ex.add_node(self.model)
        import threading
        threading.Thread(target=self.ex.spin, daemon=True).start()
        self.events = []
        self.node.create_subscription(String, "/sim/events", lambda m: self.events.append(m.data), 10)
        self.joints = self.node.create_publisher(JointState, "/joint_states", 10)
        odom = self.node.create_publisher(Odometry, "/odom", 10)
        self.odom = odom
        self.tf = StaticTransformBroadcaster(self.node)
        self.client = self.node.create_client(VerifyPick, "/crate/verify")
        self.set_throat(throat if throat is not None else TARGET["cut_point"])
        self.publish_odom()

    def set_throat(self, world_xyz):
        """The cut point at a world position (rover at the origin, no rotation)."""
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "base_footprint", "scara_cut_point"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, world_xyz)
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)
        time.sleep(0.3)

    def publish_odom(self, x=0.0, y=0.0, yaw=0.0):
        import math
        o = Odometry()
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.odom.publish(o)
        time.sleep(0.2)

    def blade(self, position):
        msg = JointState(name=["scara_cutter_blade_joint"], position=[float(position)])
        self.joints.publish(msg)

    def wait(self, cond, timeout=8.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(0.05)
        return cond()

    def verify(self):
        assert self.client.wait_for_service(timeout_sec=5.0)
        fut = self.client.call_async(VerifyPick.Request())
        assert self.wait(fut.done, 30.0)
        return fut.result()

    def close(self):
        self.ex.shutdown()
        self.model.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()


@pytest.fixture()
def rig():
    made = []

    def make(**kw):
        r = Rig(**kw)
        made.append(r)
        return r
    yield make
    for r in made:
        r.close()


def test_closing_the_blade_on_a_pedicel_cuts_that_fruit(rig):
    r = rig()
    r.blade(0.0)
    time.sleep(0.2)
    r.blade(-1.2)
    assert r.wait(lambda: any("cut " in e and TARGET["name"] in e for e in r.events)), r.events


def test_one_closing_cuts_once_and_a_second_closing_needs_a_reopening(rig):
    r = rig()
    r.blade(-1.2)
    assert r.wait(lambda: len(r.events) >= 1)
    for _ in range(5):
        r.blade(-1.2)
        time.sleep(0.05)
    time.sleep(0.5)
    assert len(r.events) == 1
    r.blade(0.0)
    time.sleep(0.2)
    r.blade(-1.2)                                           # the same fruit is already cut: nothing left in the throat
    assert r.wait(lambda: len(r.events) == 2)
    assert "nothing" in r.events[1]


def test_blade_closing_in_empty_air_cuts_nothing(rig):
    r = rig(throat=(50.0, 50.0, 5.0))
    r.blade(-1.2)
    assert r.wait(lambda: r.events)
    assert "nothing" in r.events[0]
    assert r.verify().in_crate is False and "no fruit has been cut" in r.verify().detail


def test_a_half_closed_blade_does_not_cut(rig):
    r = rig()
    for p in (0.0, -0.4, -0.7, -0.9):
        r.blade(p)
        time.sleep(0.1)
    time.sleep(0.5)
    assert r.events == []


def test_the_rovers_pose_moves_the_throat_in_the_world(rig):
    """Rover at (10, 5) facing +y: the throat, 0.5 m ahead of it in the rover frame, is at (10, 5.5) in the world."""
    import math
    r = rig(throat=(0.5, 0.0, 0.8))
    r.publish_odom(10.0, 5.0, math.pi / 2)
    throat = r.model.throat_world()
    assert throat.tolist() == pytest.approx([10.0, 5.5, 0.8], abs=1e-6)


def test_verify_reports_failure_cleanly_when_gazebo_cannot_be_read(rig):
    r = rig()
    r.blade(-1.2)
    assert r.wait(lambda: r.events)
    reply = r.verify()
    assert reply.in_crate is False
    assert "could not read the fruit position" in reply.detail or "at world" in reply.detail


def test_bad_parameters_stop_the_node(rig):
    with pytest.raises(Exception):
        rig(throat_radius=-1.0)
    with pytest.raises(Exception):
        rig(targets_file="/nonexistent/targets.json")
