#!/usr/bin/env python3
"""cmd_vel -> four wheel speeds and one steering angle.

Bicycle model: steering angle = atan(wheelbase * w / v), clipped to the steering
limit (the yaw rate is then recomputed from the clipped angle). Each wheel gets
the speed its own distance from the turn centre needs. In the simulator the
wheels are stiff velocity servos; commanding all four the same speed would make
them fight every turn, which the real DC wheel motors avoid by slowing under
load. Straight driving is unaffected: all four wheels get the same speed.
Publishes to wheel_controller [fl, fr, rl, rr] and steering_controller.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

WHEEL_RADIUS = 0.19
WHEELBASE = 0.72        # assumed, see docs/work-plan.md
TRACK = 0.55            # assumed
STEER_LIMIT = math.radians(25)
# Wheel positions relative to the rear-axle centre (x forward, y left).
WHEELS = [(WHEELBASE, TRACK / 2), (WHEELBASE, -TRACK / 2), (0.0, TRACK / 2), (0.0, -TRACK / 2)]


class Drive(Node):
    def __init__(self):
        super().__init__("drive_node")
        self.wheels = self.create_publisher(Float64MultiArray, "/wheel_controller/commands", 10)
        self.steer = self.create_publisher(Float64MultiArray, "/steering_controller/commands", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.angle = 0.0

    def on_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        if abs(v) > 1e-3:
            self.angle = max(-STEER_LIMIT, min(STEER_LIMIT, math.atan(WHEELBASE * w / v)))
        w = v * math.tan(self.angle) / WHEELBASE           # yaw rate the clipped angle gives
        if abs(w) < 1e-6:
            speeds = [v] * 4
        else:
            radius = v / w                                   # signed, from the rear-axle centre
            speeds = [math.copysign(math.hypot(radius - y, x), v) * abs(w) for x, y in WHEELS]
        self.wheels.publish(Float64MultiArray(data=[s / WHEEL_RADIUS for s in speeds]))
        self.steer.publish(Float64MultiArray(data=[self.angle, self.angle]))   # left, right knuckle


def main():
    rclpy.init()
    rclpy.spin(Drive())


if __name__ == "__main__":
    main()
