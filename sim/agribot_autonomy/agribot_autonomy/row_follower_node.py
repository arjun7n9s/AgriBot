"""Follow the furrow with the front camera.

    /front_camera/image   sensor_msgs/Image   in
    /row_follower/enable  std_msgs/Bool       in   start / stop following (default: stopped)
    /cmd_vel              geometry_msgs/Twist out  only while enabled
    /row_follower/state   agribot_interfaces/RowState  out  every image, enabled or not
    /row_follower/debug_image  sensor_msgs/Image       out  only if someone subscribes

The image processing is in agribot_autonomy.vision.furrow_line. If the camera goes
quiet while following, the node publishes a stop once, so the rover does not keep
driving on the last command (the drive node has its own watchdog as a second guard).
"""
import rclpy
from agribot_interfaces.msg import RowState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from agribot_autonomy import params, vision


class RowFollower(Node):
    def __init__(self, **node_args):
        super().__init__("row_follower", **node_args)    # node_args: parameter_overrides, context (used by the tests)
        self.speed = params.number(self, "speed", 0.15, 0.0, 1.0,
                                   "Forward speed while following, m/s; slow enough for the arm camera to confirm fruit")
        self.k_near = params.number(self, "k_near", 1.4, 0.0, 10.0, "Yaw rate per half-width of near offset, rad/s")
        self.k_far = params.number(self, "k_far", 1.6, 0.0, 10.0, "Yaw rate per half-width of far offset, rad/s")
        self.min_conf = params.number(self, "min_confidence", 0.35, 0.0, 1.0, "Below this the rover stops and waits")
        self.image_timeout = params.number(self, "image_timeout", 1.0, 0.1, 30.0,
                                           "Stop if no image arrives for this long while following, s")
        self.enabled = params.flag(self, "start_enabled", False, "Follow the row from the first image")
        self.bands = vision.RowParams(
            band=(params.number(self, "band_top", 0.60, 0.3, 0.9, "Top of the image band used, fraction of the height"),
                  params.number(self, "band_bottom", 0.97, 0.5, 1.0, "Bottom of the band, fraction of the height")),
            min_run=params.number(self, "min_run", 0.06, 0.01, 0.9, "Shortest furrow run, fraction of the image width"))
        if not self.bands.band[0] < self.bands.band[1]:
            raise ValueError("band_top must be smaller than band_bottom")

        self.bridge = CvBridge()
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state = self.create_publisher(RowState, "/row_follower/state", 10)
        self.debug = self.create_publisher(Image, "/row_follower/debug_image", 2)
        self.create_subscription(Image, "/front_camera/image", self.on_image, qos_profile_sensor_data)
        self.create_subscription(Bool, "/row_follower/enable", self.on_enable, 10)
        self.last_image = None
        self.halted = True                      # a stop has been published since the last motion
        self.create_timer(0.25, self.watchdog)

    def on_enable(self, msg):
        if self.enabled and not msg.data:
            self.stop()
        self.enabled = msg.data
        self.get_logger().info("following the row" if msg.data else "stopped")

    def stop(self):
        self.cmd.publish(Twist())
        self.halted = True

    def on_image(self, msg):
        self.last_image = self.get_clock().now()
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except (CvBridgeError, TypeError, ValueError) as e:      # TypeError: data shorter than the header says
            self.get_logger().warn(f"unusable image: {e}", throttle_duration_sec=5.0)
            return
        furrow = vision.furrow_line(bgr, self.bands)
        out = RowState(header=msg.header, near=furrow.near, far=furrow.far, confidence=furrow.confidence,
                       tracking=furrow.confidence >= self.min_conf)
        self.state.publish(out)
        if self.enabled:
            v, w = vision.row_command(furrow, self.speed, self.k_near, self.k_far, self.min_conf)
            twist = Twist()
            twist.linear.x, twist.angular.z = v, w
            self.cmd.publish(twist)
            self.halted = v == 0.0
        if self.debug.get_subscription_count():
            self.debug.publish(self.bridge.cv2_to_imgmsg(vision.draw_row(bgr, furrow, self.bands), "bgr8"))

    def watchdog(self):
        if not self.enabled or self.halted or self.last_image is None:
            return
        if (self.get_clock().now() - self.last_image).nanoseconds * 1e-9 > self.image_timeout:
            self.get_logger().warn("no front camera image: stopping", throttle_duration_sec=5.0)
            self.stop()


def main():
    rclpy.init()
    node = RowFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
