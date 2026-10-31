#!/usr/bin/env python3
"""Harvest mission: follow the row by camera, stop at detected fruit, pick them.

    1. Survey pose: the arm holds the cutter facing the row, so the arm camera
       looks at the fruit band beside the rover.
    2. Drive: /row_follower/enable -> the camera row-follower steers along the
       furrow while fruit_detector reports red fruit in the rover frame.
    3. When a fruit comes into the reachable window, stop, collect detections
       for a moment, average them, plan the cut with IK on the robot's URDF (in
       the rover frame: no map or odometry needed for targeting), and pick it.
    4. Resume until the end of the row (odometry distance) or `picks` attempts.

The cut itself is modelled: when the blade closes, the fruit whose pedicel is in
the throat (within 3 cm, looked up in the world file) is removed and a free
copy spawned, which the cradle catches. Each pick is verified by reading the
fruit's position back from Gazebo: it counts only if it lies in the crate.

Ripeness is a colour rule in fruit_detector.py for now; a learned detector will
replace it.
"""
import json
import math
import os
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from agribot_gazebo.urdf_fk import Robot, rpy_matrix, transform

ARM = ["scara_j1", "scara_z", "scara_j2", "scara_j3"]
BLADE_OPEN, BLADE_CLOSED = 0.0, -1.2
DOOR_SHUT, DOOR_OPEN = 0.0, 1.2
FACE_ROW = -math.pi / 2          # cutter mouth facing the harvest row (-y of the rover)
APPROACH = 0.08                  # pre-cut stand-off toward the furrow (m)
CUT_ABOVE_CENTRE = 0.030         # pedicel cut point above the fruit centre (fruit model)
SURVEY = (0.06, -0.385, 0.84)    # cut-point target while scanning, rover frame
STOP_X = (-0.06, 0.12)           # stop when a fruit centre enters this x band (rover frame)
WINDOW = {"x": (-0.20, 0.22), "y": (-0.74, -0.45), "z": (0.62, 0.90)}
DROP_IN_BASE = (-0.29, -0.03, 0.86)  # over the crate, toward its centre (reach limit ~0.43 m)
# Crate interior in the rover frame: walls 8 mm thick on a 0.40 x 0.30 m crate.
CRATE = {"centre": (-0.36, 0.02), "half": (0.192, 0.142), "floor_z": 0.53, "rim_z": 0.73}
THROAT_RADIUS = 0.03             # a pedicel this close to the throat gets cut


def look_at_quat(eye, target):
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    yaw, pitch = math.atan2(f[1], f[0]), -math.asin(f[2])
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)


def in_window(p, margin=0.0):
    return all(WINDOW[k][0] - margin <= v <= WINDOW[k][1] + margin for k, v in zip("xyz", p))


class Mission(Node):
    def __init__(self):
        super().__init__("harvest_mission")
        p = self.declare_parameter
        self.targets_file = p("targets_file", "").value
        self.picks = int(p("picks", 2).value)
        self.row_length = float(p("row_length", 4.6).value)
        self.capture_dir = p("capture_dir", "").value
        self.director = bool(self.capture_dir)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(PoseArray, "/fruit/detections", self.on_detections, 5)
        self.arm = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.tool = self.create_publisher(Float64MultiArray, "/tool_controller/commands", 10)
        self.scout = self.create_publisher(Float64MultiArray, "/scout_controller/commands", 10)
        self.follow = self.create_publisher(Bool, "/row_follower/enable", 10)
        self.phase_pub = self.create_publisher(String, "/harvest/phase", latched)
        self.count_pub = self.create_publisher(Int32, "/harvest/count", latched)
        self.capture_pub = self.create_publisher(String, "/capture", 10)
        self.robot = None
        self.pose = None
        self.measured = {}
        self.detections = []        # latest list of fruit centres, rover frame
        self.det_stamp = 0.0
        self.q = {"scara_j1": 0.0, "scara_z": 0.1, "scara_j2": 0.0, "scara_j3": 0.0}
        self.skip = []              # world xy of fruit already tried
        self.shot = 0

    # ---------------------------------------------------------------- inputs
    def on_description(self, msg):
        if self.robot is None:
            path = os.path.join(tempfile.gettempdir(), "agribot_mission.urdf")
            with open(path, "w") as f:
                f.write(msg.data)
            self.robot = Robot(path)

    def on_odom(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        self.pose = (p.x, p.y, yaw)

    def on_joints(self, msg):
        self.measured.update(dict(zip(msg.name, msg.position)))

    def on_detections(self, msg):
        self.detections = [np.array((q.position.x, q.position.y, q.position.z)) for q in msg.poses]
        self.det_stamp = self.now()

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

    def world(self):
        x, y, yaw = self.pose
        return transform((x, y, 0.0), rpy_matrix(0, 0, yaw))

    def to_world(self, p):
        return (self.world() @ np.r_[p, 1.0])[:3]

    SEEDS = [{"scara_j1": a, "scara_z": 0.15, "scara_j2": b, "scara_j3": c}
             for a, b, c in ((-1.0, 1.0, -1.5), (0.0, 1.5, -2.0), (1.0, -1.0, 1.0), (-2.0, 1.5, 0.0))]

    def solve(self, target, yaw=FACE_ROW, iters=120):
        """IK in the rover frame (base_footprint). Tries the current joints first, then
        a few spread-out seeds, and keeps the best (damped least squares can stall in
        a local minimum when the start is far from the goal)."""
        best = None
        for seed in [dict(self.q)] + [dict(s) for s in self.SEEDS]:
            q, e = self.robot.solve(ARM, "scara_cut_point", target, target_yaw=yaw, q0=seed,
                                    base=np.eye(4), iters=iters)
            if best is None or e < best[1]:
                best = (q, e)
            if e < 1e-3:
                break
        return best

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

    def ign(self, service, reqtype, req, reptype="ignition.msgs.Boolean", always=False):
        if not (self.director or always):
            return ""
        out = subprocess.run(["ign", "service", "-s", service, "--reqtype", reqtype, "--reptype", reptype,
                              "--timeout", "5000", "--req", req], capture_output=True, text=True, timeout=20)
        return out.stdout

    def camera(self, eye, target):
        qx, qy, qz, qw = look_at_quat(eye, target)
        self.ign("/gui/follow", "ignition.msgs.StringMsg", 'data: ""')
        self.ign("/gui/move_to/pose", "ignition.msgs.GUICamera",
                 f"pose: {{position: {{x: {eye[0]:.3f}, y: {eye[1]:.3f}, z: {eye[2]:.3f}}}, "
                 f"orientation: {{x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}}}}}")

    def chase(self):
        self.ign("/gui/follow", "ignition.msgs.StringMsg", 'data: "agribot"')
        self.ign("/gui/follow/offset", "ignition.msgs.Vector3d", "x: -2.2, y: 1.7, z: 1.5")

    def capture(self, label):
        if self.director:
            self.shot += 1
            self.capture_pub.publish(String(data=f"{self.shot:02d}_{label}"))

    # ------------------------------------------------------- modelled physics
    def release_fruit_in_throat(self):
        """Model the cut: free the fruit whose pedicel is in the throat."""
        throat = self.robot.fk(dict(self.measured), self.world())["scara_cut_point"][:3, 3]
        best, dist = None, THROAT_RADIUS
        for f in self.fruit:
            if f["name"] in self.removed:
                continue
            d = np.linalg.norm(np.array(f["cut_point"]) - throat)
            if d < dist:
                best, dist = f, d
        if best is None:
            self.get_logger().warn("blade closed on nothing: no pedicel within 3 cm of the throat")
            return None
        self.get_logger().info(f"pedicel of {best['name']} ({best['kind']}) in the throat, {dist * 1000:.0f} mm off")
        sdf_path = os.path.join(get_package_share_directory("agribot_field"), "models", best["kind"], "model.sdf")
        sdf = open(sdf_path).read().replace("0x04", "0x02")
        sdf = " ".join(sdf.replace("'", '"').split()).split("?>", 1)[-1].strip()
        x, y, z = best["knuckle"]
        r, p, yw = best["rpy"]
        cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                                  math.cos(yw / 2), math.sin(yw / 2))
        qw, qx = cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy
        qy, qz = cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy
        free = best["name"] + "_cut"
        self.ign("/world/demo_row/create", "ignition.msgs.EntityFactory",
                 f"sdf: '{sdf}', name: \"{free}\", allow_renaming: false, "
                 f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}, "
                 f"orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}", always=True)
        self.ign("/world/demo_row/remove", "ignition.msgs.Entity", f'name: "{best["name"]}", type: MODEL',
                 always=True)
        self.removed.add(best["name"])
        return free, best

    def where(self, model):
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
        local = np.linalg.inv(self.world()) @ np.r_[p, 1.0]
        ok = (abs(local[0] - CRATE["centre"][0]) < CRATE["half"][0] + 0.01
              and abs(local[1] - CRATE["centre"][1]) < CRATE["half"][1] + 0.01
              and CRATE["floor_z"] - 0.02 < p[2] < CRATE["rim_z"])
        self.get_logger().info(f"fruit at {np.round(p, 3)}: {'IN the crate' if ok else 'NOT in the crate'}")
        return ok

    # ------------------------------------------------------------ behaviours
    def candidates(self, margin=0.0):
        """Detections inside the reach window that have not been tried yet."""
        if self.now() - self.det_stamp > 1.0:
            return []
        out = []
        for c in self.detections:
            if not in_window(c, margin):
                continue
            w = self.to_world(c)
            if any(math.hypot(w[0] - s[0], w[1] - s[1]) < 0.04 for s in self.skip):
                continue
            out.append(c)
        return out

    def confirm(self, seconds=1.5):
        """Average detections while standing still; return clusters seen in most frames."""
        samples, end, last = [], self.now() + seconds, None
        while self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.det_stamp != last:
                last = self.det_stamp
                samples.append(self.candidates(margin=0.05))   # positions settle once stopped
        clusters = []
        for frame in samples:
            for c in frame:
                for cl in clusters:
                    if np.linalg.norm(np.median(cl, axis=0) - c) < 0.03:
                        cl.append(c)
                        break
                else:
                    clusters.append([c])
        need = max(2, int(0.3 * len(samples)))
        self.get_logger().info(f"confirm: {len(samples)} frames, clusters seen in "
                               f"{sorted((len(cl) for cl in clusters), reverse=True)} frames (need {need})")
        return [np.median(cl, axis=0) for cl in clusters if len(cl) >= need]

    def pick(self, centre):
        cut = centre + np.array((0, 0, CUT_ABOVE_CENTRE))
        pre = cut + np.array((0, APPROACH, 0))
        q_pre, e_pre = self.solve(pre)
        q_cut, e_cut = self.solve(cut)
        if e_pre > 3e-3 or e_cut > 3e-3:
            self.get_logger().info(f"fruit at {np.round(centre, 3)} (rover frame) is out of reach")
            return None
        self.phase(f"approach: detected fruit at {np.round(centre, 3)} m (rover frame)")
        w_cut = self.to_world(cut)
        near = sorted((np.linalg.norm(np.array(f["cut_point"]) - w_cut), f["name"], f["kind"])
                      for f in self.fruit if f["name"] not in self.removed)[:3]
        self.get_logger().info("diagnostic, nearest pedicels in the world file: "
                               + ", ".join(f"{n} ({k}) {d * 1000:.0f} mm" for d, n, k in near))
        self.set_tool(BLADE_OPEN, DOOR_SHUT, 0.2)
        self.move_arm(self.solve(pre + np.array((0, 0, 0.03)))[0], 2.5)
        self.camera(w_cut + np.array((0.52, 0.20, 0.12)), w_cut + np.array((0, 0.02, -0.03)))
        self.capture("approach")
        self.move_arm(q_pre, 1.5)
        self.move_arm(q_cut, 2.0)
        miss = np.linalg.norm(self.robot.fk(dict(self.measured), np.eye(4))["scara_cut_point"][:3, 3] - cut)
        self.get_logger().info(f"throat at the estimated pedicel: {miss * 1000:.1f} mm from the target")
        self.capture("engage")

        self.phase("cut: closing the blade across the throat")
        for k in range(1, 7):
            self.set_tool(BLADE_OPEN + (BLADE_CLOSED - BLADE_OPEN) * k / 6, DOOR_SHUT, 0.15)
        released = self.release_fruit_in_throat()
        self.wait(1.2)
        self.capture("caught")
        self.set_tool(BLADE_OPEN, DOOR_SHUT, 0.4)

        self.phase("retreat and carry to the crate")
        self.move_arm(self.solve(pre + np.array((0, 0.02, 0.0)))[0], 2.5)
        self.move_arm(self.solve(pre + np.array((0, 0.02, 0.06)))[0], 2.0)
        drop = np.array(DROP_IN_BASE)
        yaw = math.atan2(drop[1] - CRATE["centre"][1], drop[0] - CRATE["centre"][0])
        q, e = self.solve(drop, yaw, iters=150)
        if e > 5e-3:
            q, e = self.solve(drop, None, iters=150)
        self.move_arm(q, 4.5)
        x, y, _ = self.pose
        self.camera((x - 1.1, y + 1.05, 1.75), (x - 0.25, y - 0.15, 0.7))
        self.phase("drop: opening the trap door over the crate")
        self.set_tool(BLADE_OPEN, DOOR_OPEN, 1.6)
        self.capture("drop")
        self.set_tool(BLADE_OPEN, DOOR_SHUT, 0.8)
        if released is None:
            return False
        return self.in_crate(released[0])

    def survey(self, seconds=3.0):
        q, e = self.solve(np.array(SURVEY))
        if e > 3e-3:
            raise RuntimeError("survey pose is out of reach")
        self.move_arm(q, seconds)

    # ------------------------------------------------------------------ main
    def run(self):
        while rclpy.ok() and (self.robot is None or self.pose is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.wait(2.0)
        self.fruit = json.load(open(self.targets_file))["fruit"]
        self.removed = set()
        x0 = self.pose[0]
        self.phase("start: camera row-following, colour fruit detection (ripeness model to come)")
        if self.director:
            self.chase()
        self.survey()
        self.scout.publish(Float64MultiArray(data=[-1.2]))
        self.capture("start")

        done = attempts = 0
        while rclpy.ok() and attempts < self.picks and self.pose[0] - x0 < self.row_length:
            self.phase("drive: following the row with the front camera, scanning for fruit")
            if self.director:
                self.chase()
            self.drive(True)
            target, previous, last = None, [], None
            while rclpy.ok() and self.pose[0] - x0 < self.row_length:
                rclpy.spin_once(self, timeout_sec=0.02)
                if self.det_stamp == last:
                    continue
                last = self.det_stamp
                ahead = [c for c in self.candidates() if STOP_X[0] <= c[0] <= STOP_X[1]]
                # Stop only for a fruit seen in two consecutive frames (one-frame blobs
                # are usually a glint or a fruit at the edge of the colour rule). Compare
                # in the world frame: the rover moves between frames, the fruit does not.
                seen = [(c, self.to_world(c)) for c in ahead]
                steady = [c for c, w in seen if any(np.linalg.norm(w - p) < 0.03 for p in previous)]
                if steady:
                    target = steady
                    self.get_logger().info(f"stopping for fruit at {[list(np.round(c, 3)) for c in steady]}")
                    break
                previous = [w for _, w in seen]
            self.drive(False)
            if target is None:
                break
            self.wait(1.0)
            self.capture("arrived")
            seen = self.confirm()
            self.get_logger().info(f"{len(seen)} fruit confirmed in reach: {[list(np.round(c, 3)) for c in seen]}")
            if not seen:
                for c in target:
                    self.skip.append(self.to_world(c)[:2])
                continue
            centre = min(seen, key=lambda c: abs(c[0] - 0.03))
            self.skip.append(self.to_world(centre)[:2])
            ok = self.pick(centre)
            if ok is None:
                continue
            attempts += 1
            done += int(ok)
            self.count_pub.publish(Int32(data=done))
            self.phase(f"pick {attempts}: {'fruit in the crate' if ok else 'missed'} ({done} in the crate so far)")
            self.survey(2.5)

        self.phase(f"done: {done} of {attempts} picks verified in the crate")
        if self.director:
            x, y, _ = self.pose
            self.camera((x - 2.6, y + 2.2, 2.3), (x + 0.4, y - 0.6, 0.5))
            self.wait(2.0)
            self.capture("final")
            time.sleep(1.0)


def main():
    rclpy.init()
    node = Mission()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
