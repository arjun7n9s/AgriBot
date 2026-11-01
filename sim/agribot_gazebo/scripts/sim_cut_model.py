#!/usr/bin/env python3
"""The simulator's stand-in for what Gazebo cannot do: cut a pedicel, and see the crate.

The harvest mission only closes the blade and asks /crate/verify afterwards. This node
plays the physics and the crate sensor:

    /joint_states (the blade joint)   ->  when it closes, the fruit whose pedicel is in
                                          the throat is removed and a free copy spawned
                                          at the same pose (collision group 0x02), which
                                          the cradle catches.
    /crate/verify  agribot_interfaces/srv/VerifyPick
                                      ->  reads the last cut fruit's position back from
                                          Gazebo and says whether it lies in the crate.
    /sim/events    std_msgs/String     one line per cut, for the log and for tests

The fruit list comes from worlds/demo_row_targets.json (written with the world by
tools/sim/make_world.py). None of this is visible to the autonomy nodes.
"""
import json
import math
import os
import subprocess
import threading

import numpy as np
import rclpy
import tf2_ros
from agribot_interfaces.srv import VerifyPick
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from agribot_gazebo import cut_model

CUT_FRAME = "scara_cut_point"
BASE_FRAME = "base_footprint"


class SimCutModel(Node):
    def __init__(self, **node_args):
        super().__init__("sim_cut_model", **node_args)    # node_args: parameter_overrides (used by the tests)
        share = get_package_share_directory("agribot_gazebo")
        p = self.declare_parameter
        self.targets_file = p("targets_file", os.path.join(share, "worlds", "demo_row_targets.json")).value
        self.world_name = p("world_name", "demo_row").value
        self.blade_joint = p("blade_joint", "scara_cutter_blade_joint").value
        self.throat_radius = float(p("throat_radius", 0.03).value)
        self.service_timeout = float(p("service_timeout", 5.0).value)          # s per Gazebo service call
        self.trigger = cut_model.BladeTrigger(float(p("closed_below", -1.0).value), float(p("reopened_above", -0.3).value))
        self.crate = cut_model.CrateBox(
            centre=tuple(p("crate_centre", [-0.36, 0.02]).value), half=tuple(p("crate_half", [0.192, 0.142]).value),
            floor_z=float(p("crate_floor_z", 0.53).value), rim_z=float(p("crate_rim_z", 0.73).value))
        if not self.throat_radius > 0 or not self.service_timeout > 0:
            raise ValueError("throat_radius and service_timeout must be positive")
        with open(self.targets_file) as f:
            self.fruit = json.load(f)["fruit"]
        self.removed = set()
        self.last_released = None
        self.pose = None
        self.lock = threading.Lock()

        cbg = ReentrantCallbackGroup()
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self, spin_thread=True)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10, callback_group=cbg)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10, callback_group=cbg)
        self.events = self.create_publisher(String, "/sim/events", 10)
        self.create_service(VerifyPick, "/crate/verify", self.on_verify, callback_group=cbg)
        self.get_logger().info(f"{len(self.fruit)} fruit in the world; throat radius {self.throat_radius * 100:.0f} cm")

    def on_odom(self, msg):
        pos, o = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        self.pose = (pos.x, pos.y, yaw)

    def on_joints(self, msg):
        if self.blade_joint not in msg.name:
            return
        if self.trigger.update(msg.position[msg.name.index(self.blade_joint)]):
            self.cut()

    # ------------------------------------------------------------------- cut
    def throat_world(self):
        if self.pose is None:
            return None
        try:
            t = self.tf.lookup_transform(BASE_FRAME, CUT_FRAME, Time()).transform.translation
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"no transform to {CUT_FRAME}: {e}")
            return None
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([x + c * t.x - s * t.y, y + s * t.x + c * t.y, t.z])

    def cut(self):
        with self.lock:
            throat = self.throat_world()
            if throat is None:
                return
            fruit, dist = cut_model.fruit_in_throat(self.fruit, self.removed, throat, self.throat_radius)
            if fruit is None:
                self.event(f"blade closed on nothing: no pedicel within {self.throat_radius * 100:.0f} cm of the throat")
                return
            self.event(f"cut {fruit['name']} ({fruit['kind']}), pedicel {dist * 1000:.0f} mm from the throat")
            sdf_path = os.path.join(get_package_share_directory("agribot_field"), "models", fruit["kind"], "model.sdf")
            with open(sdf_path) as f:
                sdf = f.read().replace("0x04", "0x02")           # cut fruit: group the cradle and crate catch
            sdf = " ".join(sdf.replace("'", '"').split()).split("?>", 1)[-1].strip()
            x, y, z = fruit["knuckle"]
            qx, qy, qz, qw = cut_model.rpy_to_quaternion(*fruit["rpy"])
            free = fruit["name"] + "_cut"
            self.ign("/world/%s/create" % self.world_name, "ignition.msgs.EntityFactory",
                     f"sdf: '{sdf}', name: \"{free}\", allow_renaming: false, "
                     f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}, "
                     f"orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}")
            self.ign("/world/%s/remove" % self.world_name, "ignition.msgs.Entity", f'name: "{fruit["name"]}", type: MODEL')
            self.removed.add(fruit["name"])
            self.last_released = free

    def event(self, text):
        self.get_logger().info(text)
        self.events.publish(String(data=text))

    def ign(self, service, reqtype, req):
        out = subprocess.run(["ign", "service", "-s", service, "--reqtype", reqtype, "--reptype", "ignition.msgs.Boolean",
                              "--timeout", str(int(self.service_timeout * 1000)), "--req", req],
                             capture_output=True, text=True, timeout=self.service_timeout + 15)
        if "true" not in out.stdout:
            self.get_logger().warn(f"{service} did not confirm: {out.stdout.strip() or out.stderr.strip()}")

    # ---------------------------------------------------------------- verify
    def where(self, model):
        try:
            out = subprocess.run(["ign", "model", "-m", model, "-p"], capture_output=True, text=True, timeout=20).stdout
        except (subprocess.SubprocessError, OSError):
            return None
        return cut_model.parse_model_position(out)

    def on_verify(self, request, response):
        with self.lock:                     # a cut in progress must finish spawning first
            released = self.last_released
            p = self.where(released) if released is not None else None
        if released is None:
            response.in_crate, response.detail = False, "no fruit has been cut"
            return response
        if p is None or self.pose is None:
            response.in_crate, response.detail = False, "could not read the fruit position from Gazebo"
            return response
        local = cut_model.to_rover_frame(p, self.pose)
        response.in_crate = self.crate.contains(local)
        response.detail = f"{released} at world {np.round(p, 3).tolist()}"
        return response


def main():
    rclpy.init()
    node = SimCutModel()
    executor = MultiThreadedExecutor(num_threads=4)
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
