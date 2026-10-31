#!/usr/bin/env python3
"""Scripted harvest demo: drive the furrow, stop beside a ripe tomato, cut it,
catch it, and drop it in the crate. Repeats for `picks` fruit.

Targets come from worlds/demo_row_targets.json (ground truth written by the
world generator). The ripe-fruit detector is not in this loop yet; the log and
/harvest/phase say so.

Fruit on the plants are static and ignored by the robot's collisions. The cut is
modelled here: when the blade closes, the fruit is removed and a free-falling
copy is spawned at the same pose in the collision group the cradle and crate
catch (see tools/sim/make_world.py). Each pick is then verified by reading the
fruit's position back from Gazebo.

Arm targets are solved with damped least squares on the robot's own URDF
(agribot_gazebo.urdf_fk), so the cutter's throat meets the pedicel with the
mouth facing the plant. Motions go to arm_controller (JointTrajectoryController);
blade and trap door to tool_controller.

Optional "director" mode drives the Gazebo GUI camera, takes screenshots and
records video through the GUI services, and triggers capture_frames.py.
"""
import json
import math
import os
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray, Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agribot_gazebo.urdf_fk import Robot, rpy_matrix, transform

ARM = ["scara_j1", "scara_z", "scara_j2", "scara_j3"]
BLADE_OPEN, BLADE_CLOSED = 0.0, -1.2
DOOR_SHUT, DOOR_OPEN = 0.0, 1.2
APPROACH = 0.08                  # pre-cut stand-off toward the furrow (m)
DROP_IN_BASE = (-0.26, -0.06)    # cut-point target over the crate, base_footprint xy
DROP_Z = 0.86                    # world z: cradle floor clears the crate rim
CRATE = {"centre": (-0.36, 0.02), "half": (0.19, 0.14), "floor_z": 0.53, "rim_z": 0.73}


def look_at_quat(eye, target):
    """Quaternion (x, y, z, w) for a camera at eye looking at target (x forward, z up)."""
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    yaw, pitch = math.atan2(f[1], f[0]), -math.asin(f[2])
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)


class Harvest(Node):
    def __init__(self):
        super().__init__("harvest_demo")
        p = self.declare_parameter
        self.spawn = (p("spawn_x", -1.6).value, p("spawn_y", 1.0).value)
        self.targets_file = p("targets_file", "").value
        self.picks = int(p("picks", 2).value)
        self.capture_dir = p("capture_dir", "").value
        self.director = bool(self.capture_dir)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.measured = {}
        self.arm = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.tool = self.create_publisher(Float64MultiArray, "/tool_controller/commands", 10)
        self.scout = self.create_publisher(Float64MultiArray, "/scout_controller/commands", 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.phase_pub = self.create_publisher(String, "/harvest/phase", latched)
        self.count_pub = self.create_publisher(Int32, "/harvest/count", latched)
        self.capture_pub = self.create_publisher(String, "/capture", 10)
        self.robot = None
        self.pose = None            # (x, y, yaw) of base_footprint in the world
        self.odom_offset = None
        self.q = {"scara_j1": 0.0, "scara_z": 0.1, "scara_j2": 0.0, "scara_j3": 0.0}
        self.shot = 0

    # ---------------------------------------------------------------- inputs
    def on_description(self, msg):
        if self.robot is None:
            path = os.path.join(tempfile.gettempdir(), "agribot_harvest.urdf")
            with open(path, "w") as f:
                f.write(msg.data)
            self.robot = Robot(path)

    def on_odom(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        if self.odom_offset is None:
            # OdometryPublisher may report relative to the spawn pose; detect and correct.
            far = math.hypot(p.x - self.spawn[0], p.y - self.spawn[1]) > 0.3
            self.odom_offset = (self.spawn[0] - p.x, self.spawn[1] - p.y) if far else (0.0, 0.0)
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        self.pose = (p.x + self.odom_offset[0], p.y + self.odom_offset[1], yaw)

    def on_joints(self, msg):
        self.measured.update(dict(zip(msg.name, msg.position)))

    def reached(self, frame="scara_cut_point"):
        """World position of `frame` from the measured joints and odometry."""
        return self.robot.fk(dict(self.measured), self.base())[frame][:3, 3]

    # --------------------------------------------------------------- helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, seconds):
        end = self.now() + seconds
        while rclpy.ok() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def phase(self, text):
        self.get_logger().info(text)
        self.phase_pub.publish(String(data=text))

    def base(self, pose=None):
        x, y, yaw = pose or self.pose
        return transform((x, y, 0.0), rpy_matrix(0, 0, yaw))

    def solve(self, target, yaw, pose=None):
        q, err = self.robot.solve(ARM, "scara_cut_point", target, target_yaw=yaw,
                                  q0=dict(self.q), base=self.base(pose), iters=120)
        return q, err

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

    def ign(self, service, reqtype, req, reptype="ignition.msgs.Boolean", always=False):
        if not (self.director or always):
            return ""
        out = subprocess.run(["ign", "service", "-s", service, "--reqtype", reqtype, "--reptype", reptype,
                              "--timeout", "5000", "--req", req], capture_output=True, text=True, timeout=20)
        return out.stdout

    def cut_free(self, fruit):
        """Replace the attached fruit with a free body at the same pose (group 0x02)."""
        sdf_path = os.path.join(get_package_share_directory("agribot_field"), "models", fruit["kind"], "model.sdf")
        sdf = open(sdf_path).read().replace("0x04", "0x02")
        sdf = " ".join(sdf.replace("'", '"').split())
        sdf = sdf.split("?>", 1)[-1].strip()
        x, y, z = fruit["knuckle"]
        r, p, yw = fruit["rpy"]
        cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                                  math.cos(yw / 2), math.sin(yw / 2))
        qw, qx = cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy
        qy, qz = cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy
        picked = fruit["name"] + "_cut"
        self.ign("/world/demo_row/create", "ignition.msgs.EntityFactory",
                 f"sdf: '{sdf}', name: \"{picked}\", allow_renaming: false, "
                 f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}, "
                 f"orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}", always=True)
        self.ign("/world/demo_row/remove", "ignition.msgs.Entity",
                 f'name: "{fruit["name"]}", type: MODEL', always=True)
        return picked

    def camera(self, eye, target):
        qx, qy, qz, qw = look_at_quat(eye, target)
        self.ign("/gui/follow", "ignition.msgs.StringMsg", 'data: ""')
        self.ign("/gui/move_to/pose", "ignition.msgs.GUICamera",
                 f"pose: {{position: {{x: {eye[0]:.3f}, y: {eye[1]:.3f}, z: {eye[2]:.3f}}}, "
                 f"orientation: {{x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}}}}}")

    def capture(self, label):
        if not self.director:
            return
        self.shot += 1
        tag = f"{self.shot:02d}_{label}"
        self.capture_pub.publish(String(data=tag))
        # Gazebo GUI stills come from the screen recording (tools/sim/make_video.py);
        # the GUI screenshot service shows a banner that would appear in the video.

    # ------------------------------------------------------------ behaviours
    def drive_to(self, x_goal, y_line):
        self.phase(f"drive: following the furrow to x = {x_goal:.2f} m")
        t0 = self.now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, yaw = self.pose
            ex = x_goal - x
            if abs(ex) < 0.01:
                break
            v = max(-0.3, min(0.3, 0.8 * ex))
            if abs(v) < 0.06:
                v = math.copysign(0.06, ex)
            w = -1.5 * (y - y_line) - 2.0 * yaw
            pan = 0.9 * math.sin(0.8 * (self.now() - t0))        # scouting sweep
            self.scout.publish(Float64MultiArray(data=[pan]))
            self.cmd.publish(Twist(linear=self._v3(v), angular=self._v3(0, 0, w * (1 if v > 0 else -1))))
        for _ in range(10):
            self.cmd.publish(Twist())
            self.wait(0.05)
        self.scout.publish(Float64MultiArray(data=[-1.2]))    # look at the harvest side
        self.wait(0.6)

    @staticmethod
    def _v3(x=0.0, y=0.0, z=0.0):
        from geometry_msgs.msg import Vector3
        return Vector3(x=float(x), y=float(y), z=float(z))

    def plan(self, fruit, furrow_y):
        """Rover x that lets the arm reach this fruit from the furrow, or None."""
        cut = np.array(fruit["cut_point"])
        pre = cut + np.array((0, APPROACH, 0))
        best = None
        for dx in (-0.06, -0.12, 0.0, -0.18, 0.06):
            pose = (cut[0] + dx, furrow_y, 0.0)
            q1, e1 = self.solve(pre, -math.pi / 2, pose)
            q2, e2 = self.solve(cut, -math.pi / 2, pose)
            if e1 < 3e-3 and e2 < 3e-3:
                score = abs(abs(q2["scara_j2"]) - 1.1)
                if best is None or score < best[0]:
                    best = (score, cut[0] + dx)
        return best[1] if best else None

    def harvest(self, fruit):
        cut = np.array(fruit["cut_point"])
        pre = cut + np.array((0, APPROACH, 0))

        self.phase(f"approach: {fruit['name']} ({fruit['kind']}), target from world ground truth")
        self.set_tool(BLADE_OPEN, DOOR_SHUT, 0.2)
        q, e = self.solve(pre + np.array((0, 0, 0.03)), -math.pi / 2)
        self.move_arm(q, 3.0)
        # Close-up from just ahead of the rover, looking back at the cutter (the mast
        # would block a view from behind).
        self.camera(cut + np.array((0.52, 0.20, 0.12)), cut + np.array((0, 0.02, -0.03)))
        self.capture("approach")
        q, e = self.solve(pre, -math.pi / 2)
        self.move_arm(q, 1.5)
        q, e = self.solve(cut, -math.pi / 2)
        self.move_arm(q, 2.0)
        miss = np.linalg.norm(self.reached() - cut)
        joints = {j: round(self.measured.get(j, float("nan")), 3) for j in ARM}
        self.get_logger().info(f"planned joints {({j: round(q[j], 3) for j in ARM})}")
        self.get_logger().info(f"throat on the pedicel: planned error {e * 1000:.1f} mm, "
                               f"measured error {miss * 1000:.1f} mm, joints {joints}")
        self.capture("engage")

        self.phase("cut: closing the blade across the throat")
        for k in range(1, 7):                       # slow cut, per docs/scara-redesign.md
            self.set_tool(BLADE_OPEN + (BLADE_CLOSED - BLADE_OPEN) * k / 6, DOOR_SHUT, 0.15)
        picked = self.cut_free(fruit)
        self.wait(1.2)
        self.capture("caught")
        self.set_tool(BLADE_OPEN, DOOR_SHUT, 0.4)

        caught = self.where(picked)
        if caught is not None:
            gap = np.linalg.norm(caught - cut)
            self.get_logger().info(f"fruit after the cut: {gap * 100:.1f} cm from the cut point "
                                   f"({'in the cradle' if gap < 0.08 else 'NOT in the cradle'})")

        self.phase("retreat and carry to the crate")
        q, e = self.solve(pre + np.array((0, 0.02, 0.0)), -math.pi / 2)
        self.move_arm(q, 2.5)
        q, e = self.solve(pre + np.array((0, 0.02, 0.06)), -math.pi / 2)
        self.move_arm(q, 2.0)
        drop = self.drop_point()
        # Point the cutter so the fruit rolls off the trap door toward the crate centre.
        cx, cy = self.base_xy(CRATE["centre"])
        yaw = math.atan2(drop[1] - cy, drop[0] - cx)
        q, e = self.robot.solve(ARM, "scara_cut_point", drop, target_yaw=yaw, q0=dict(self.q),
                                base=self.base(), iters=150)
        if e > 5e-3:
            q, e = self.robot.solve(ARM, "scara_cut_point", drop, q0=dict(self.q), base=self.base(), iters=150)
        self.move_arm(q, 4.5)
        x, y, _ = self.pose
        self.camera((x - 1.1, y + 1.05, 1.75), (x - 0.25, y - 0.15, 0.7))
        self.phase("drop: opening the trap door over the crate")
        self.set_tool(BLADE_OPEN, DOOR_OPEN, 1.6)
        self.capture("drop")
        self.set_tool(BLADE_OPEN, DOOR_SHUT, 0.8)
        return self.in_crate(picked)

    def where(self, model):
        """World position of a Gazebo model (slow: one `ign model` call)."""
        try:
            out = subprocess.run(["ign", "model", "-m", model, "-p"], capture_output=True, text=True,
                                 timeout=20).stdout
            line = out.split("XYZ", 1)[1].splitlines()[1]
            return np.array([float(v) for v in line.strip(" []").split()])
        except Exception:                       # noqa: BLE001 - verification is best effort
            return None

    def in_crate(self, model):
        p = self.where(model)
        if p is None:
            self.get_logger().warn("could not read the fruit position; pick not verified")
            return False
        x, y, yaw = self.pose
        dx, dy = p[0] - x, p[1] - y
        bx = math.cos(yaw) * dx + math.sin(yaw) * dy - CRATE["centre"][0]
        by = -math.sin(yaw) * dx + math.cos(yaw) * dy - CRATE["centre"][1]
        ok = abs(bx) < CRATE["half"][0] and abs(by) < CRATE["half"][1] and             CRATE["floor_z"] - 0.02 < p[2] < CRATE["rim_z"]
        self.get_logger().info(f"fruit at {np.round(p, 3)}: {'IN the crate' if ok else 'NOT in the crate'}")
        return ok

    def base_xy(self, local):
        t = self.base() @ transform((local[0], local[1], 0.0))
        return t[0, 3], t[1, 3]

    def drop_point(self):
        t = self.base() @ transform((DROP_IN_BASE[0], DROP_IN_BASE[1], 0.0))
        return np.array((t[0, 3], t[1, 3], DROP_Z))

    # ------------------------------------------------------------------ main
    def run(self):
        while rclpy.ok() and (self.robot is None or self.pose is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.wait(2.0)
        data = json.load(open(self.targets_file))
        furrow_y = data["furrow_y"]
        self.phase("start: targets from the world file; detector not yet in the loop")

        if self.director:
            self.ign("/gui/record_video", "ignition.msgs.VideoRecord",
                     f'start: true, format: "mp4", save_filename: "{os.path.join(self.capture_dir, "agribot_demo.mp4")}"')
            self.ign("/gui/follow", "ignition.msgs.StringMsg", 'data: "agribot"')
            self.ign("/gui/follow/offset", "ignition.msgs.Vector3d", "x: -2.2, y: 1.7, z: 1.5")

        stow = self.robot.solve(ARM, "scara_cut_point", self.drop_point(), q0=dict(self.q),
                                base=self.base(), iters=120)[0]
        self.move_arm(stow, 3.0)
        self.capture("start")

        ripe = sorted((f for f in data["fruit"] if f["kind"] == "tomato_ripe"), key=lambda f: f["cut_point"][0])
        done = attempts = 0
        for fruit in ripe:
            if attempts >= self.picks:
                break
            if fruit["cut_point"][1] < 0.40:           # too close to the stem line
                continue
            x_goal = self.plan(fruit, furrow_y)
            if x_goal is None or x_goal < self.pose[0] - 0.05:
                continue
            if self.director:
                self.ign("/gui/follow", "ignition.msgs.StringMsg", 'data: "agribot"')
            self.drive_to(x_goal, furrow_y)
            self.capture("arrived")
            ok = self.harvest(fruit)
            attempts += 1
            done += int(ok)
            self.count_pub.publish(Int32(data=done))
            self.phase(f"pick {attempts}: {'fruit in the crate' if ok else 'missed'} "
                       f"({done} in the crate so far)")
            self.move_arm(stow, 2.5)

        self.phase(f"done: {done} of {attempts} picks verified in the crate")
        if self.director:
            x, y, _ = self.pose
            self.camera((x - 2.6, y + 2.2, 2.3), (x + 0.4, y - 0.6, 0.5))
            self.wait(2.0)
            self.capture("final")
            self.ign("/gui/record_video", "ignition.msgs.VideoRecord", "stop: true")
            time.sleep(3.0)


def main():
    rclpy.init()
    node = Harvest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
