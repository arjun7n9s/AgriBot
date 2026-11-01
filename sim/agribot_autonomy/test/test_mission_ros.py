"""The harvest mission against a fake robot: no Gazebo, a fake clock that runs 5 times faster.

The fake robot publishes what the real one does (robot description, odometry, joint states,
fruit detections) and records what the mission commands (arm trajectories, tool, row
follower). A fake /crate/verify answers as configured. This tests the mission's decisions,
its sequencing, the numbers it sends to the arm, and what it does when things go wrong.
"""
import math
import pathlib
import threading
import time

import numpy as np
import pytest

# Not imported from conftest: run together with agribot_gazebo's tests, `conftest` names the other package's.
REPO = pathlib.Path(__file__).resolve().parents[3]

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("agribot_interfaces.msg")

from agribot_interfaces.action import Harvest  # noqa: E402
from agribot_interfaces.msg import HarvestStatus  # noqa: E402
from agribot_interfaces.srv import VerifyPick  # noqa: E402
from geometry_msgs.msg import Pose, PoseArray  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray, String  # noqa: E402
from trajectory_msgs.msg import JointTrajectory  # noqa: E402

from agribot_autonomy.harvest_mission_node import Mission  # noqa: E402
from ros_harness import Harness  # noqa: E402

ARM = ["scara_j1", "scara_z", "scara_j2", "scara_j3"]
SPEED = 5.0                                           # fake clock speed-up
FRUIT = (0.09, -0.50, 0.82)                           # centre of the fake fruit, rover frame (inside the reach box)


class FakeRobot:
    def __init__(self, h, urdf_text, fruit=(FRUIT,), verify=True, odom=(0.0, 0.0, 0.0)):
        self.h = h
        self.fruit = list(fruit)
        self.events = []                              # ("traj", positions) / ("tool", data) / ("follow", bool) in order
        self.q = {"scara_j1": 0.0, "scara_z": 0.1, "scara_j2": 0.0, "scara_j3": 0.0}
        self.odom = list(odom)
        self.start = time.monotonic()
        n = h.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clock_pub = n.create_publisher(Clock, "/clock", 10)
        self.desc_pub = n.create_publisher(String, "/robot_description", latched)
        self.odom_pub = n.create_publisher(Odometry, "/odom", 10)
        self.joint_pub = n.create_publisher(JointState, "/joint_states", 10)
        self.det_pub = n.create_publisher(PoseArray, "/fruit/detections", 5)
        n.create_subscription(JointTrajectory, "/arm_controller/joint_trajectory", self.on_traj, 10)
        n.create_subscription(Float64MultiArray, "/tool_controller/commands", lambda m: self.events.append(("tool", list(m.data))), 10)
        n.create_subscription(Bool, "/row_follower/enable", lambda m: self.events.append(("follow", m.data)), 10)
        n.create_subscription(Float64MultiArray, "/scout_controller/commands", lambda m: None, 10)
        self.status = h.collect(HarvestStatus, "/harvest/status", QoSProfile(depth=50, durability=DurabilityPolicy.VOLATILE))
        self.verify_answer = verify
        if verify is not None:
            n.create_service(VerifyPick, "/crate/verify", self.on_verify)
        self.desc_pub.publish(String(data=urdf_text))
        self.stop = threading.Event()
        threading.Thread(target=self.clock_loop, daemon=True).start()
        n.create_timer(0.025, self.publish_state)

    def on_traj(self, msg):
        self.events.append(("traj", list(msg.points[0].positions)))
        for name, v in zip(msg.joint_names, msg.points[0].positions):
            self.q[name] = v

    def on_verify(self, request, response):
        response.in_crate, response.detail = bool(self.verify_answer), "fake crate sensor"
        return response

    def clock_loop(self):
        while not self.stop.is_set():
            t = (time.monotonic() - self.start) * SPEED
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = int(t), int((t % 1) * 1e9)
            self.clock_pub.publish(msg)
            time.sleep(0.005)

    def publish_state(self):
        now = self.h.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.odom[0], self.odom[1]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.odom[2] / 2), math.cos(self.odom[2] / 2)
        self.odom_pub.publish(odom)
        js = JointState(name=list(self.q), position=[float(v) for v in self.q.values()])
        js.header.stamp = now
        self.joint_pub.publish(js)
        det = PoseArray()
        det.header.stamp, det.header.frame_id = now, "base_footprint"
        for f in self.fruit:
            p = Pose()
            p.position.x, p.position.y, p.position.z = map(float, f)
            p.orientation.w = 1.0
            det.poses.append(p)
        self.det_pub.publish(det)

    def close(self):
        self.stop.set()

    def of(self, kind):
        return [e[1] for e in self.events if e[0] == kind]


@pytest.fixture(scope="module")
def urdf_text_mod():
    text = (REPO / "sim/agribot_description/urdf/agribot.urdf").read_text(encoding="utf-8")
    return text.replace('xyz="0 -0.095 0.46"', 'xyz="0 -0.095 0.64"')


@pytest.fixture()
def h():
    harness = Harness()
    yield harness
    harness.close()


def start_mission(h, sim_time=True, **params):
    """The mission under test. Without a fake clock (sim_time=False) it runs on wall time."""
    return h.make(Mission, **{"use_sim_time": sim_time, "confirm_seconds": 1.5, **params})


def send_goal(h, picks=1, distance=0.0):
    client = ActionClient(h.node, Harvest, "harvest")
    assert client.wait_for_server(timeout_sec=5.0)
    fut = client.send_goal_async(Harvest.Goal(max_picks=picks, max_distance=distance))
    assert h.wait_for(fut.done, timeout=5.0)
    return client, fut.result()


def result_of(h, handle, timeout=120.0):
    fut = handle.get_result_async()
    assert h.wait_for(fut.done, timeout=timeout), "the goal did not finish"
    return fut.result()


def cut_point(robot_model, q):
    return robot_model.fk_frame(q, "scara_cut_point")[:3, 3]


def states_of(robot):
    out = []
    for m in robot.status:
        if not out or out[-1] != m.state:
            out.append(m.state)
    return out


# ----------------------------------------------------------------- goal checks
@pytest.mark.parametrize("picks, distance", [(0, 1.0), (1, float("nan")), (1, -1.0), (1, float("inf"))])
def test_invalid_goals_are_rejected(h, picks, distance):
    start_mission(h)
    client = ActionClient(h.node, Harvest, "harvest")
    assert client.wait_for_server(timeout_sec=5.0)
    fut = client.send_goal_async(Harvest.Goal(max_picks=picks, max_distance=distance))
    assert h.wait_for(fut.done, timeout=5.0)
    assert not fut.result().accepted


def test_a_second_goal_is_rejected_while_one_runs_and_cancel_works(h):
    start_mission(h, sim_time=False, startup_timeout=60.0)
    client, first = send_goal(h)
    assert first.accepted
    second = client.send_goal_async(Harvest.Goal(max_picks=1))
    assert h.wait_for(second.done, timeout=5.0)
    assert not second.result().accepted
    cancel = first.cancel_goal_async()
    assert h.wait_for(cancel.done, timeout=5.0)
    res = result_of(h, first, timeout=10.0)
    assert res.status == 5                              # GoalStatus.STATUS_CANCELED
    assert res.result.message == "canceled"
    again = client.send_goal_async(Harvest.Goal(max_picks=1))                 # the server is free again
    assert h.wait_for(again.done, timeout=5.0) and again.result().accepted
    h.wait_for(lambda: False, timeout=0.2)
    again.result().cancel_goal_async()


def test_goal_aborts_with_the_missing_inputs_when_the_robot_never_appears(h):
    start_mission(h, sim_time=False, startup_timeout=3.0)
    _, handle = send_goal(h)
    res = result_of(h, handle, timeout=30.0)
    assert res.status == 6                              # STATUS_ABORTED
    assert not res.result.success
    for name in ("/robot_description", "/odom", "/joint_states", "/arm_controller/joint_trajectory"):
        assert name in res.result.message, res.result.message


# ------------------------------------------------------------------ full picks
def run_pick(h, urdf, **robot_kwargs):
    start_mission(h, stall_window=20.0)
    fake = FakeRobot(h, urdf, **robot_kwargs)
    _, handle = send_goal(h, picks=1, distance=3.0)
    res = result_of(h, handle, timeout=150.0)
    return fake, res


def test_a_verified_pick_end_to_end(h, urdf_text_mod):
    from agribot_autonomy.urdf_fk import Robot
    model = Robot(xml=urdf_text_mod)
    fake, res = run_pick(h, urdf_text_mod)
    fake.close()
    assert res.status == 4 and res.result.success                              # STATUS_SUCCEEDED
    assert (res.result.attempted, res.result.verified) == (1, 1)

    # The mission drove, then stopped.
    follow = fake.of("follow")
    assert follow[0] is True and follow[-1] is False

    # Every state, in order.
    order = states_of(fake)
    wanted = [HarvestStatus.STARTING, HarvestStatus.DRIVING, HarvestStatus.CONFIRMING, HarvestStatus.APPROACHING,
              HarvestStatus.ENGAGING, HarvestStatus.CUTTING, HarvestStatus.CARRYING, HarvestStatus.DROPPING,
              HarvestStatus.DONE]
    it = iter(order)
    assert all(any(s == w for s in it) for w in wanted), f"states seen: {order}"

    # The arm pose when the blade starts closing puts the cutter on the pedicel: 30 mm above the fruit centre.
    events = fake.events
    first_close = next(i for i, e in enumerate(events) if e[0] == "tool" and e[1][0] < -0.1)
    q_cut = next(e[1] for e in reversed(events[:first_close]) if e[0] == "traj")
    cut = cut_point(model, dict(zip(ARM, q_cut)))
    assert np.linalg.norm(cut - (np.array(FRUIT) + (0, 0, 0.03))) < 0.003, cut

    # The blade closes completely, opens, and the trap door opens over the crate and shuts.
    tool = fake.of("tool")
    assert min(t[0] for t in tool) == pytest.approx(-1.2)
    assert max(t[1] for t in tool) == pytest.approx(1.2)
    door_open = next(i for i, e in enumerate(events) if e[0] == "tool" and e[1][1] > 1.0)
    q_drop = next(e[1] for e in reversed(events[:door_open]) if e[0] == "traj")
    drop = cut_point(model, dict(zip(ARM, q_drop)))
    assert np.linalg.norm(drop - np.array([-0.29, -0.03, 0.86])) < 0.01, drop

    # The mission ends with the arm back at the survey pose.
    survey = cut_point(model, dict(zip(ARM, fake.of("traj")[-1])))
    assert np.linalg.norm(survey - np.array([0.06, -0.385, 0.84])) < 0.003


def test_a_pick_the_crate_sensor_denies_is_not_counted(h, urdf_text_mod):
    fake, res = run_pick(h, urdf_text_mod, verify=False)
    fake.close()
    assert res.result.success and (res.result.attempted, res.result.verified) == (1, 0)


def test_a_missing_crate_sensor_leaves_the_pick_unverified_but_the_goal_succeeds(h, urdf_text_mod):
    fake, res = run_pick(h, urdf_text_mod, verify=None)
    fake.close()
    assert res.result.success and (res.result.attempted, res.result.verified) == (1, 0)


# ----------------------------------------------------------------- failure modes
def test_fruit_outside_the_reach_box_is_ignored_and_a_stalled_rover_aborts(h, urdf_text_mod):
    start_mission(h, stall_window=6.0)
    fake = FakeRobot(h, urdf_text_mod, fruit=[(0.09, -0.95, 0.82), (0.5, -0.5, 0.82), (0.09, -0.5, 1.4)])
    _, handle = send_goal(h, picks=1, distance=3.0)
    res = result_of(h, handle, timeout=120.0)
    fake.close()
    assert res.status == 6 and "no progress" in res.result.message
    assert res.result.attempted == 0
    assert all(t[0] == 0.0 for t in fake.of("tool")), "the blade moved although no fruit was in reach"
    assert fake.of("follow")[-1] is False, "the row follower was left enabled"


def test_a_stalled_rover_aborts_and_stops_the_follower(h, urdf_text_mod):
    start_mission(h, stall_window=6.0)
    fake = FakeRobot(h, urdf_text_mod, fruit=[])
    _, handle = send_goal(h, picks=1, distance=3.0)
    res = result_of(h, handle, timeout=120.0)
    fake.close()
    assert res.status == 6 and "no progress" in res.result.message
    assert fake.of("follow")[-1] is False
    assert states_of(fake)[-1] == HarvestStatus.FAILED


def test_the_mission_finishes_when_the_row_ends(h, urdf_text_mod):
    """With no fruit and the rover creeping forward, the goal ends normally at max_distance."""
    start_mission(h, stall_window=60.0)
    fake = FakeRobot(h, urdf_text_mod, fruit=[])
    stop = threading.Event()

    def creep():
        while not stop.is_set():
            fake.odom[0] += 0.02
            time.sleep(0.02)
    threading.Thread(target=creep, daemon=True).start()
    _, handle = send_goal(h, picks=3, distance=1.0)
    res = result_of(h, handle, timeout=120.0)
    stop.set()
    fake.close()
    assert res.result.success and res.result.attempted == 0
    assert res.result.distance >= 1.0 - 1e-6


def test_a_broken_robot_description_aborts_instead_of_crashing(h, urdf_text_mod):
    start_mission(h, startup_timeout=3.0)
    fake = FakeRobot(h, "<robot><not-urdf", fruit=[])
    _, handle = send_goal(h, picks=1, distance=3.0)
    res = result_of(h, handle, timeout=60.0)
    fake.close()
    assert res.status == 6 and "/robot_description" in res.result.message


def test_a_robot_without_the_arm_aborts_with_a_reason(h):
    start_mission(h, startup_timeout=5.0)
    bare = '<robot name="x"><link name="base_footprint"/></robot>'
    fake = FakeRobot(h, bare, fruit=[])
    _, handle = send_goal(h, picks=1, distance=3.0)
    res = result_of(h, handle, timeout=60.0)
    fake.close()
    assert res.status == 6 and "scara_cut_point" in res.result.message
