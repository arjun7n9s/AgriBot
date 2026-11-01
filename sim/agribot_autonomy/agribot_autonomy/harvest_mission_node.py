"""Harvest mission: follow the row by camera, stop at detected fruit, pick them.

Interface
    action  /harvest  agribot_interfaces/action/Harvest    goal: max_picks, max_distance
    topic   /harvest/status  agribot_interfaces/HarvestStatus  state changes, plus 2 Hz
    topic   /harvest/phase   std_msgs/String                   same events, as text
    topic   /harvest/count   std_msgs/Int32                    picks verified so far
    service /crate/verify    agribot_interfaces/srv/VerifyPick (client; optional)

One goal runs at a time; a second is rejected. Cancel stops the rover and the goal ends
CANCELED. The mission only uses what the robot has: the front camera (through the row
follower), the arm camera (through the fruit detector), joint states, odometry and
the robot description. Per goal:

    1. Survey pose: the arm holds the cutter facing the row, so the arm camera looks at
       the fruit band beside the rover.
    2. Drive: /row_follower/enable follows the furrow. Stop when a detection inside the
       reach window shows up in two consecutive frames at the same world position.
    3. Confirm: collect detections for confirm_seconds while standing still and keep
       the clusters seen in most frames.
    4. Pick: solve IK on the robot's URDF for the pedicel above the fruit, approach, close
       the blade, hold, carry the fruit to the crate and open the trap door.
    5. Verify with /crate/verify, if some node provides it.
    6. Repeat until max_picks are attempted or the row ends.

It aborts, with the reason in the result, if the robot never becomes ready, if the rover
makes no progress for stall_window seconds (lost row, blocked) or if the arm cannot reach
a pose the plan needs.
"""
import math
import time

import numpy as np
import rclpy
from agribot_interfaces.action import Harvest
from agribot_interfaces.msg import HarvestStatus
from agribot_interfaces.srv import VerifyPick
from geometry_msgs.msg import Point, PoseArray
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agribot_autonomy import mission_logic as logic
from agribot_autonomy import params
from agribot_autonomy.urdf_fk import Robot, rpy_matrix, transform

ARM = ["scara_j1", "scara_z", "scara_j2", "scara_j3"]
CUT_FRAME = "scara_cut_point"
FACE_ROW = -math.pi / 2          # cutter mouth facing the harvest row (-y of the rover)
SEEDS = [{"scara_j1": a, "scara_z": 0.15, "scara_j2": b, "scara_j3": c}
         for a, b, c in ((-1.0, 1.0, -1.5), (0.0, 1.5, -2.0), (1.0, -1.0, 1.0), (-2.0, 1.5, 0.0))]


class MissionAbort(Exception):
    """The goal cannot continue; the message goes into the result."""


class MissionCanceled(Exception):
    pass


class Mission(Node):
    def __init__(self, **node_args):
        super().__init__("harvest_mission", **node_args)    # node_args: parameter_overrides, context (used by the tests)
        self.row_length = params.number(self, "row_length", 4.6, 0.1, 100.0, "Row to cover if the goal gives no distance, m")
        self.startup_timeout = params.number(self, "startup_timeout", 60.0, 1.0, 600.0,
                                             "Wait this long for the robot's inputs before aborting, s")
        self.stall_window = params.number(self, "stall_window", 20.0, 2.0, 600.0, "Abort if driving makes no progress for this long, s")
        self.stall_progress = params.number(self, "stall_progress", 0.05, 0.001, 1.0, "Progress that counts as moving, m")
        self.confirm_seconds = params.number(self, "confirm_seconds", 1.5, 0.2, 10.0, "Detections are collected this long when stopped, s")
        self.approach = params.number(self, "approach_distance", 0.08, 0.0, 0.3, "Pre-cut stand-off toward the furrow, m")
        self.cut_above_centre = params.number(self, "cut_above_centre", 0.030, 0.0, 0.1, "Cut point above the fruit centre, m")
        self.cut_hold = params.number(self, "cut_hold", 3.5, 0.5, 20.0, "Keep the blade closed this long after closing, s")
        self.blade_open = params.number(self, "blade_open", -0.03, -3.2, 3.2, "Blade joint, open, rad")
        self.blade_closed = params.number(self, "blade_closed", -1.2, -3.2, 3.2, "Blade joint, closed, rad")
        self.door_shut = params.number(self, "door_shut", 0.0, -3.2, 3.2, "Trap-door joint, shut, rad")
        self.door_open = params.number(self, "door_open", 1.2, -3.2, 3.2, "Trap-door joint, open, rad")
        self.verify_timeout = params.number(self, "verify_timeout", 60.0, 1.0, 300.0, "Wait for /crate/verify this long, s")
        self.reach = logic.Reach(
            x=tuple(params.vector(self, "reach_x", (0.02, 0.22), 2, -3.0, 3.0, "Cut-point reach box, rover frame, m")),
            y=tuple(params.vector(self, "reach_y", (-0.60, -0.45), 2, -3.0, 3.0, "")),
            z=tuple(params.vector(self, "reach_z", (0.70, 0.90), 2, 0.0, 3.0, "")))
        self.stop_x = tuple(params.vector(self, "stop_x", (0.02, 0.12), 2, -3.0, 3.0,
                                          "Stop when a fruit centre is within this x band (rover frame), m"))
        self.survey_point = tuple(params.vector(self, "survey_point", (0.06, -0.385, 0.84), 3, -3.0, 3.0,
                                                "Cut-point target while scanning, rover frame, m"))
        self.crate = logic.Crate(
            centre=tuple(params.vector(self, "crate_centre", (-0.36, 0.02), 2, -3.0, 3.0, "Crate centre xy, rover frame, m")),
            half=tuple(params.vector(self, "crate_half", (0.192, 0.142), 2, 0.01, 3.0, "Crate inside half extents xy, m")),
            drop=tuple(params.vector(self, "drop_point", (-0.29, -0.03, 0.86), 3, -3.0, 3.0,
                                     "Cut-point target over the crate, rover frame, m")))
        if not self.crate.drop_inside():
            raise ValueError("drop_point is not above the crate (crate_centre, crate_half)")
        if not self.stop_x[0] < self.stop_x[1]:
            raise ValueError("stop_x must have low < high")

        cbg = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_description, latched, callback_group=cbg)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10, callback_group=cbg)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10, callback_group=cbg)
        self.create_subscription(PoseArray, "/fruit/detections", self.on_detections, 5, callback_group=cbg)
        self.arm = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.tool = self.create_publisher(Float64MultiArray, "/tool_controller/commands", 10)
        self.scout = self.create_publisher(Float64MultiArray, "/scout_controller/commands", 10)
        self.follow = self.create_publisher(Bool, "/row_follower/enable", 10)
        self.status_pub = self.create_publisher(HarvestStatus, "/harvest/status", latched)
        self.phase_pub = self.create_publisher(String, "/harvest/phase", latched)
        self.count_pub = self.create_publisher(Int32, "/harvest/count", latched)
        self.verify_client = self.create_client(VerifyPick, "/crate/verify", callback_group=cbg)
        self.server = ActionServer(self, Harvest, "harvest", execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=cbg)
        self.create_timer(0.5, self.heartbeat, callback_group=cbg)

        self.robot = None
        self.pose = None                # (x, y, yaw) of base_footprint in the world
        self.measured = {}
        self.detections = []
        self.det_time = 0.0
        self.det_count = 0
        self.q = {"scara_j1": 0.0, "scara_z": 0.1, "scara_j2": 0.0, "scara_j3": 0.0}
        self.busy = False
        self.goal = None
        self.skip = []
        self.start, self.heading = (0.0, 0.0), (1.0, 0.0)
        self.focus = None
        self.state, self.detail = HarvestStatus.IDLE, "waiting for a goal"
        self.attempts = self.verified = 0
        self.distance = 0.0
        self.publish_status()

    # ---------------------------------------------------------------- inputs
    def on_description(self, msg):
        if self.robot is None:
            try:
                self.robot = Robot(xml=msg.data)
            except Exception as e:                  # noqa: BLE001 - a bad description must not kill the node
                self.get_logger().error(f"cannot read /robot_description: {e}")

    def on_odom(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, logic.yaw_from_quaternion(o.x, o.y, o.z, o.w))

    def on_joints(self, msg):
        self.measured.update(dict(zip(msg.name, msg.position)))

    def on_detections(self, msg):
        self.detections = [np.array((q.position.x, q.position.y, q.position.z)) for q in msg.poses]
        self.det_time = self.now()
        self.det_count += 1

    # ---------------------------------------------------------- status output
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_status(self):
        msg = HarvestStatus(state=self.state, detail=self.detail, attempts=self.attempts, verified=self.verified)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        point = self.focus if self.focus is not None else (self.pose[:2] + (0.0,) if self.pose else None)
        if point is not None:
            msg.focus_valid = True
            msg.focus = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
        self.status_pub.publish(msg)

    def set_state(self, state, detail, log=None):
        """Move to a new state: publish it, and log `log` (the text make_video.py reads) if given."""
        self.state, self.detail = state, detail
        if log:
            self.get_logger().info(log)
            self.phase_pub.publish(String(data=log))
        self.publish_status()
        self.send_feedback()

    def send_feedback(self):
        if self.goal is None:
            return
        fb = Harvest.Feedback(state=self.state, detail=self.detail, attempts=self.attempts, verified=self.verified,
                              distance=float(self.distance))
        try:
            self.goal.publish_feedback(fb)
        except Exception:                           # noqa: BLE001 - goal already finished
            pass

    def heartbeat(self):
        self.publish_status()

    # --------------------------------------------------------------- helpers
    def wait(self, seconds):
        """Sleep on the ROS clock; raise if the goal is canceled meanwhile."""
        end = self.now() + seconds
        while self.context.ok() and self.now() < end:
            self.check_cancel()
            time.sleep(0.01)

    def check_cancel(self):
        if self.goal is not None and self.goal.is_cancel_requested:
            raise MissionCanceled()
        if not self.context.ok():
            raise MissionAbort("ROS is shutting down")

    def world(self):
        x, y, yaw = self.pose
        return transform((x, y, 0.0), rpy_matrix(0, 0, yaw))

    def to_world(self, p):
        return (self.world() @ np.r_[p, 1.0])[:3]

    def solve(self, target, yaw=FACE_ROW, iters=120):
        """IK in the rover frame (base_footprint) from the current joints, then spread-out seeds."""
        return self.robot.solve_best(ARM, CUT_FRAME, target, target_yaw=yaw, seeds=[dict(self.q)] + SEEDS,
                                     iters=iters)

    def move_arm(self, q, seconds):
        traj = JointTrajectory(joint_names=ARM)
        traj.points = [JointTrajectoryPoint(positions=[float(q[j]) for j in ARM],
                                            time_from_start=Duration(seconds=seconds).to_msg())]
        self.arm.publish(traj)
        self.q.update({j: q[j] for j in ARM})
        self.wait(seconds + 0.4)

    def set_tool(self, blade, door, settle=0.8):
        self.tool.publish(Float64MultiArray(data=[blade, door]))
        self.wait(settle)

    def drive(self, on):
        for _ in range(3):
            self.follow.publish(Bool(data=on))
            self.wait(0.05)

    def stop_following(self):
        """Tell the row follower to stop, without waiting (safe while canceling)."""
        if self.context.ok():
            for _ in range(3):
                self.follow.publish(Bool(data=False))

    def progress(self):
        """Metres travelled along the heading the goal started with."""
        x, y, _ = self.pose
        return (x - self.start[0]) * self.heading[0] + (y - self.start[1]) * self.heading[1]

    # ------------------------------------------------------------ behaviours
    def candidates(self, margin=0.0):
        """Detections whose cut point lies inside the reach box and that have not been tried yet."""
        if self.now() - self.det_time > 1.0:
            return []
        lift = np.array((0.0, 0.0, self.cut_above_centre))
        return [c for c in self.detections
                if self.reach.contains(c + lift, margin) and not logic.world_xy_near(self.to_world(c), self.skip, 0.04)]

    def confirm(self):
        """Collect detections while standing still; return the clusters seen in most frames."""
        frames, end, last = [], self.now() + self.confirm_seconds, self.det_count
        while self.now() < end:
            self.check_cancel()
            time.sleep(0.01)
            if self.det_count != last:
                last = self.det_count
                frames.append(self.candidates(margin=0.05))   # positions settle once stopped
        confirmed = logic.cluster_detections(frames)
        self.get_logger().info(f"confirm: {len(frames)} frames, {len(confirmed)} fruit confirmed")
        return confirmed

    def pick(self, centre):
        """Cut the fruit at `centre` (rover frame). Returns False if it is out of reach."""
        cut = centre + np.array((0, 0, self.cut_above_centre))
        pre = cut + np.array((0, self.approach, 0))
        q_pre, e_pre = self.solve(pre)
        q_cut, e_cut = self.solve(cut)
        if e_pre > 3e-3 or e_cut > 3e-3:
            self.get_logger().info(f"fruit at {np.round(centre, 3)} (rover frame) is out of reach")
            return False
        self.focus = self.to_world(cut)
        self.set_state(HarvestStatus.APPROACHING, "arm to the pre-cut pose",
                       f"approach: detected fruit at {np.round(centre, 3)} m (rover frame)")
        self.set_tool(self.blade_open, self.door_shut, 0.2)
        self.move_arm(self.solve(pre + np.array((0, 0, 0.03)))[0], 2.5)
        self.move_arm(q_pre, 1.5)
        self.set_state(HarvestStatus.ENGAGING, "cutter onto the pedicel")
        self.move_arm(q_cut, 2.0)
        miss = np.linalg.norm(self.robot.fk(dict(self.measured), np.eye(4))[CUT_FRAME][:3, 3] - cut)
        self.get_logger().info(f"throat at the estimated pedicel: {miss * 1000:.1f} mm from the target")

        self.set_state(HarvestStatus.CUTTING, "closing the blade", "cut: closing the blade across the throat")
        for k in range(1, 7):
            self.set_tool(self.blade_open + (self.blade_closed - self.blade_open) * k / 6, self.door_shut, 0.15)
        self.wait(self.cut_hold)                    # the fruit settles into the cradle
        self.set_tool(self.blade_open, self.door_shut, 0.4)

        self.set_state(HarvestStatus.CARRYING, "carrying the fruit to the crate", "retreat and carry to the crate")
        self.move_arm(self.solve(pre + np.array((0, 0.02, 0.0)))[0], 2.5)
        self.move_arm(self.solve(pre + np.array((0, 0.02, 0.06)))[0], 2.0)
        drop = np.array(self.crate.drop)
        yaw = math.atan2(drop[1] - self.crate.centre[1], drop[0] - self.crate.centre[0])
        q, e = self.solve(drop, yaw, iters=150)
        if e > 5e-3:
            q, e = self.solve(drop, None, iters=150)
        if e > 5e-3:
            raise MissionAbort(f"the arm cannot reach the crate (IK error {e * 1000:.0f} mm); fruit is still in the cradle")
        self.move_arm(q, 4.5)
        self.focus = None
        self.set_state(HarvestStatus.DROPPING, "trap door open over the crate", "drop: opening the trap door over the crate")
        self.set_tool(self.blade_open, self.door_open, 1.6)
        self.set_tool(self.blade_open, self.door_shut, 0.8)
        return True

    def verify(self):
        """True or False from /crate/verify, or None if nothing could say."""
        if not self.verify_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("no /crate/verify service: pick not verified")
            return None
        future = self.verify_client.call_async(VerifyPick.Request())
        end = self.now() + self.verify_timeout
        while not future.done():
            self.check_cancel()
            if self.now() > end:
                self.get_logger().warn("/crate/verify timed out: pick not verified")
                return None
            time.sleep(0.02)
        reply = future.result()
        self.get_logger().info(f"verify: {'IN the crate' if reply.in_crate else 'NOT in the crate'} ({reply.detail})")
        return bool(reply.in_crate)

    def survey(self, seconds=3.0):
        q, e = self.solve(np.array(self.survey_point))
        if e > 3e-3:
            raise MissionAbort(f"the survey pose is out of reach (IK error {e * 1000:.0f} mm)")
        self.move_arm(q, seconds)

    def wait_ready(self):
        """Wait for the robot description, odometry, joint states and the arm controller."""
        end = self.now() + self.startup_timeout
        while True:
            self.check_cancel()
            missing = [name for name, ok in (
                ("/robot_description", self.robot is not None), ("/odom", self.pose is not None),
                ("/joint_states with the arm joints", all(j in self.measured for j in ARM)),
                ("/arm_controller/joint_trajectory subscriber", self.arm.get_subscription_count() > 0)) if not ok]
            if not missing:
                break
            if self.now() > end:
                raise MissionAbort("robot not ready after %.0f s, missing: %s" % (self.startup_timeout, ", ".join(missing)))
            time.sleep(0.1)
        links = self.robot.links
        if CUT_FRAME not in links or any(j not in self.robot.joints for j in ARM):
            raise MissionAbort(f"the robot description has no {CUT_FRAME} or lacks arm joints {ARM}")

    # ----------------------------------------------------------------- goals
    def on_goal(self, goal):
        if self.busy:
            self.get_logger().warn("goal rejected: a harvest is already running")
            return GoalResponse.REJECT
        if goal.max_picks < 1 or not math.isfinite(goal.max_distance) or goal.max_distance < 0:
            self.get_logger().warn("goal rejected: max_picks must be at least 1 and max_distance finite and not negative")
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        self.goal = goal_handle
        request = goal_handle.request
        result = Harvest.Result()
        try:
            self.run(request.max_picks, request.max_distance or self.row_length)
            result.success, result.message = True, f"{self.verified} of {self.attempts} picks verified in the crate"
            goal_handle.succeed()
        except MissionCanceled:
            result.message = "canceled"
            self.set_state(HarvestStatus.CANCELED, "canceled by the client", "done: canceled")
            goal_handle.canceled()
        except MissionAbort as e:
            result.message = str(e)
            self.get_logger().error(f"aborted: {e}")
            self.set_state(HarvestStatus.FAILED, str(e))
            goal_handle.abort()
        except Exception as e:                      # noqa: BLE001 - report any bug as an abort, not a dead node
            result.message = f"internal error: {type(e).__name__}: {e}"
            self.get_logger().error(f"aborted: {result.message}")
            self.set_state(HarvestStatus.FAILED, result.message)
            goal_handle.abort()
        finally:
            self.stop_following()
            result.attempted, result.verified = self.attempts, self.verified
            result.distance = float(self.distance)
            self.goal = None
            self.busy = False
        return result

    def run(self, max_picks, max_distance):
        self.attempts = self.verified = 0
        self.distance = 0.0
        self.skip = []
        self.set_state(HarvestStatus.STARTING, "waiting for the robot",
                       "start: camera row-following, colour fruit detection (ripeness model to come)")
        self.wait_ready()
        self.wait(2.0)
        self.start = self.pose[:2]
        self.heading = (math.cos(self.pose[2]), math.sin(self.pose[2]))
        self.survey()
        self.scout.publish(Float64MultiArray(data=[-1.2]))
        stall = logic.StallWatchdog(self.stall_window, self.stall_progress)

        while self.attempts < max_picks and self.progress() < max_distance:
            self.focus = None
            self.set_state(HarvestStatus.DRIVING, "following the row, scanning for fruit",
                           "drive: following the row with the front camera, scanning for fruit")
            self.drive(True)
            stall.reset()
            target, previous, last = None, [], self.det_count
            while self.progress() < max_distance:
                self.check_cancel()
                time.sleep(0.01)
                self.distance = self.progress()
                if stall.update(self.now(), self.distance):
                    self.stop_following()
                    raise MissionAbort(f"no progress for {self.stall_window:.0f} s at {self.distance:.2f} m (row lost or blocked)")
                if self.det_count == last:
                    continue
                last = self.det_count
                ahead = [c for c in self.candidates() if self.stop_x[0] <= c[0] <= self.stop_x[1]]
                seen = [(c, self.to_world(c)) for c in ahead]
                steady = logic.steady_targets(seen, previous, 0.03)
                if steady:
                    target = steady
                    self.get_logger().info(f"stopping for fruit at {[list(np.round(c, 3)) for c in steady]}")
                    break
                previous = [w for _, w in seen]
            self.drive(False)
            self.distance = self.progress()
            if target is None:
                break
            self.set_state(HarvestStatus.CONFIRMING, "stopped; confirming the detection")
            self.wait(1.0)
            confirmed = self.confirm()
            if not confirmed:
                self.skip.extend(self.to_world(c)[:2] for c in target)
                continue
            mid = 0.5 * (self.stop_x[0] + self.stop_x[1])
            centre = min(confirmed, key=lambda c: abs(c[0] - mid))
            self.skip.append(self.to_world(centre)[:2])
            if not self.pick(centre):
                continue
            self.attempts += 1
            ok = self.verify()
            self.verified += int(bool(ok))
            self.count_pub.publish(Int32(data=self.verified))
            outcome = {True: "fruit in the crate", False: "missed", None: "not verified"}[ok]
            self.set_state(HarvestStatus.DRIVING, "pick finished",
                           f"pick {self.attempts}: {outcome} ({self.verified} in the crate so far)")
            self.survey(2.5)

        self.focus = None
        self.set_state(HarvestStatus.DONE, f"{self.verified} of {self.attempts} picks verified",
                       f"done: {self.verified} of {self.attempts} picks verified in the crate")
        self.wait(2.0)


def main():
    rclpy.init()
    node = Mission()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
