#!/usr/bin/env python3
"""cmd_vel -> four equal wheel speeds and one steering angle.

The stock rover drives all four wheel motors together and steers with one
motor, so this is a bicycle model: steering angle = atan(wheelbase * w / v),
clipped to the steering limit. Publishes to wheel_controller and
steering_controller (JointGroup controllers).
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

WHEEL_RADIUS = 0.19
WHEELBASE = 0.72        # assumed, see docs/work-plan.md
STEER_LIMIT = math.radians(25)


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
        omega = v / WHEEL_RADIUS
        self.wheels.publish(Float64MultiArray(data=[omega] * 4))
        self.steer.publish(Float64MultiArray(data=[self.angle]))


def main():
    rclpy.init()
    rclpy.spin(Drive())


if __name__ == "__main__":
    main()
