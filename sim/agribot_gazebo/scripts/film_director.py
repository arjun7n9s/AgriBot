#!/usr/bin/env python3
"""Camera director for filming: moves the Gazebo GUI camera as the mission changes state.

Listens to /harvest/status and /odom, drives the GUI through Gazebo's /gui/follow and
/gui/move_to/pose services, and asks capture_frames to save the robot's camera images
(on /capture) at fixed moments. The mission knows nothing about it.

Each state has a short script of (delay in seconds, shot or capture). A new state
cancels the unfinished script of the previous one.
"""
import subprocess
import threading
import time

import rclpy
from agribot_interfaces.msg import HarvestStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from agribot_gazebo.shots import look_at_quaternion

S = HarvestStatus


class Director(Node):
    def __init__(self):
        super().__init__("film_director")
        self.create_subscription(HarvestStatus, "/harvest/status", self.on_status, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.capture_pub = self.create_publisher(String, "/capture", 10)
        self.pose = (0.0, 0.0)
        self.state = None
        self.shot = 0
        self.generation = 0
        self.after_start = False

    def on_odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_status(self, msg):
        if msg.state == self.state:
            return
        self.state = msg.state
        focus = (msg.focus.x, msg.focus.y, msg.focus.z) if msg.focus_valid else None
        x, y = self.pose
        script = []
        if msg.state == S.STARTING:
            self.after_start = True
        elif msg.state == S.DRIVING:
            script.append((0.0, self.chase))
            if self.after_start:
                self.after_start = False
                script.append((0.5, lambda: self.capture("start")))
        elif msg.state == S.CONFIRMING:
            script.append((1.0, lambda: self.capture("arrived")))
        elif msg.state == S.APPROACHING and focus:
            script += [(2.9, lambda: self.camera((focus[0] + 0.52, focus[1] + 0.20, focus[2] + 0.12),
                                                 (focus[0], focus[1] + 0.02, focus[2] - 0.03))),
                       (3.1, lambda: self.capture("approach"))]
        elif msg.state == S.ENGAGING:
            script.append((2.2, lambda: self.capture("engage")))
        elif msg.state == S.CUTTING:
            script.append((3.4, lambda: self.capture("caught")))
        elif msg.state == S.DROPPING:
            script += [(0.0, lambda: self.camera((x - 1.1, y + 1.05, 1.75), (x - 0.25, y - 0.15, 0.7))),
                       (1.6, lambda: self.capture("drop"))]
        elif msg.state == S.DONE:
            script += [(0.0, lambda: self.camera((x - 2.6, y + 2.2, 2.3), (x + 0.4, y - 0.6, 0.5))),
                       (2.0, lambda: self.capture("final"))]
        if script:
            self.generation += 1
            threading.Thread(target=self.play, args=(self.generation, script), daemon=True).start()

    def play(self, generation, script):
        start = time.monotonic()
        for delay, action in script:
            while time.monotonic() - start < delay:
                if generation != self.generation:
                    return
                time.sleep(0.05)
            if generation != self.generation:
                return
            try:
                action()
            except (subprocess.SubprocessError, OSError, ValueError) as e:
                self.get_logger().warn(f"camera command failed: {e}")

    def ign(self, service, reqtype, req):
        subprocess.run(["ign", "service", "-s", service, "--reqtype", reqtype, "--reptype", "ignition.msgs.Boolean",
                        "--timeout", "5000", "--req", req], capture_output=True, text=True, timeout=20)

    def camera(self, eye, target):
        qx, qy, qz, qw = look_at_quaternion(eye, target)
        self.ign("/gui/follow", "ignition.msgs.StringMsg", 'data: ""')
        self.ign("/gui/move_to/pose", "ignition.msgs.GUICamera",
                 f"pose: {{position: {{x: {eye[0]:.3f}, y: {eye[1]:.3f}, z: {eye[2]:.3f}}}, "
                 f"orientation: {{x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}}}}}")

    def chase(self):
        self.ign("/gui/follow", "ignition.msgs.StringMsg", 'data: "agribot"')
        self.ign("/gui/follow/offset", "ignition.msgs.Vector3d", "x: -2.2, y: 1.7, z: 1.5")

    def capture(self, label):
        self.shot += 1
        self.capture_pub.publish(String(data=f"{self.shot:02d}_{label}"))


def main():
    rclpy.init()
    node = Director()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
