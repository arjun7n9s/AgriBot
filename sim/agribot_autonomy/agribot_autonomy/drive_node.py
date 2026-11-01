"""cmd_vel to wheel speeds and steering angles.

    /cmd_vel (geometry_msgs/Twist)  ->  /wheel_controller/commands    [fl, fr, rl, rr] rad/s
                                        /steering_controller/commands [left, right] rad

The kinematics are in agribot_autonomy.steering. This node adds the safety around
them: a command watchdog (the wheels stop if /cmd_vel goes quiet), speed limits, and
refusal of non-finite commands.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from agribot_autonomy import params, steering


class DriveNode(Node):
    def __init__(self, **node_args):
        super().__init__("drive_node", **node_args)    # node_args: parameter_overrides, context (used by the tests)
        kind = params.text(self, "steering_geometry", "ackermann",
                           "'ackermann' (inner wheel turns more) or 'parallel' (both knuckles equal)")
        if kind not in ("ackermann", "parallel"):
            raise ValueError(f"steering_geometry must be 'ackermann' or 'parallel', got '{kind}'")
        self.geom = steering.Geometry(
            wheelbase=params.number(self, "wheelbase", 0.72, 0.2, 3.0, "Front to rear axle, m (assumed until measured)"),
            track=params.number(self, "track", 0.55, 0.2, 2.0, "Wheel centre to wheel centre of an axle, m (assumed)"),
            wheel_radius=params.number(self, "wheel_radius", 0.19, 0.03, 1.0, "m"),
            steer_limit=math.radians(params.number(
                self, "steer_limit_deg", 25.0, 1.0, 60.0, "Largest angle of the inner front wheel, deg")),
            ackermann=kind == "ackermann")
        self.max_speed = params.number(self, "max_speed", 1.0, 0.01, 5.0, "Command limit, m/s")
        self.max_yaw_rate = params.number(self, "max_yaw_rate", 1.5, 0.01, 6.0, "Command limit, rad/s")
        self.timeout = params.number(self, "cmd_timeout", 0.5, 0.05, 10.0,
                                     "Stop the wheels when no /cmd_vel arrives for this long, s")
        self.wheels = self.create_publisher(Float64MultiArray, "/wheel_controller/commands", 10)
        self.steer = self.create_publisher(Float64MultiArray, "/steering_controller/commands", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.angle = 0.0                    # centre (bicycle) angle currently held
        self.last_cmd = None                # clock time of the latest command
        self.stopped = True
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(f"{kind} steering, wheelbase {self.geom.wheelbase} m, track {self.geom.track} m")

    def on_cmd(self, msg):
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            self.get_logger().warn("ignoring a non-finite command", throttle_duration_sec=2.0)
            return                          # clamping would turn NaN into a full-speed command
        v = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        w = max(-self.max_yaw_rate, min(self.max_yaw_rate, msg.angular.z))
        plan = steering.plan(self.geom, v, w, self.angle)
        self.angle = plan.centre_angle
        self.last_cmd = self.get_clock().now()
        self.stopped = plan.v == 0.0
        self.publish(plan)

    def publish(self, plan):
        self.wheels.publish(Float64MultiArray(data=list(plan.wheel_omega)))
        self.steer.publish(Float64MultiArray(data=[plan.steer_left, plan.steer_right]))

    def watchdog(self):
        if self.stopped or self.last_cmd is None:
            return
        if (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.timeout:
            self.get_logger().warn("no /cmd_vel: stopping the wheels", throttle_duration_sec=5.0)
            self.stopped = True
            self.publish(steering.plan(self.geom, 0.0, 0.0, self.angle))


def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
