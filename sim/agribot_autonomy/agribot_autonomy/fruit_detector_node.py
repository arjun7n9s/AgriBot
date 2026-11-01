"""Find red tomatoes with the arm RGB-D camera and locate them in 3D.

A deliberately simple first detector: a colour rule picks red pixels, connected
blobs become fruit candidates, the depth image gives their range, and TF puts them
in the rover frame. It finds red fruit, which includes the red underside of
half-ripe ones; telling ripe from half-ripe is the job of the learned ripeness
model that comes next, and it will publish the same outputs.

    /arm_camera/image, /arm_camera/depth_image, /arm_camera/camera_info   in
    /fruit/detections   geometry_msgs/PoseArray, frame = target_frame: fruit centres
    /fruit/markers      visualization_msgs/MarkerArray, for RViz
    /fruit/debug_image  sensor_msgs/Image, colour mask, blobs and ranges

An image whose transform is not available is skipped, not guessed. The image
processing is in agribot_autonomy.vision.locate_blobs.
"""
import message_filters
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray

from agribot_autonomy import params, vision


class FruitDetector(Node):
    def __init__(self, **node_args):
        super().__init__("fruit_detector", **node_args)    # node_args: parameter_overrides, context (used by the tests)
        self.target_frame = params.text(self, "target_frame", "base_footprint", "Frame the detections are given in")
        self.p = vision.FruitParams(
            fruit_radius=params.number(self, "fruit_radius", 0.028, 0.005, 0.15,
                                       "Fruit radius, m: the depth hits the near surface"),
            min_area=params.integer(self, "min_area", 150, 1, 100000, "Smallest blob, px"),
            max_range=params.number(self, "max_range", 0.9, 0.1, 10.0, "Farthest fruit reported, m"),
            core_fraction=params.number(self, "core_fraction", 0.3, 0.05, 1.0,
                                        "Depth is read this fraction of a blob's radius around its centroid"),
            depth_percentile=params.number(self, "depth_percentile", 30.0, 0.0, 100.0,
                                           "Fallback: percentile of the whole blob's depth taken as its near surface"))
        self.tf_timeout = params.number(self, "tf_timeout", 0.1, 0.0, 5.0, "Wait for a transform at the image time, s")
        self.tf_max_age = params.number(self, "tf_max_age", 1.0, 0.0, 5.0,
                                        "Newest transform accepted when none exists at the image time, s")
        self.bridge = CvBridge()
        self.K = None
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self, spin_thread=True)   # own thread: lookups may wait
        self.create_subscription(CameraInfo, "/arm_camera/camera_info", self.on_info, qos_profile_sensor_data)
        rgb = message_filters.Subscriber(self, Image, "/arm_camera/image", qos_profile=qos_profile_sensor_data)
        depth = message_filters.Subscriber(self, Image, "/arm_camera/depth_image", qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb, depth], 4, 0.05)
        self.sync.registerCallback(self.on_images)
        self.poses = self.create_publisher(PoseArray, "/fruit/detections", 5)
        self.markers = self.create_publisher(MarkerArray, "/fruit/markers", 5)
        self.debug = self.create_publisher(Image, "/fruit/debug_image", 2)

    def on_info(self, msg):
        K = np.array(msg.k).reshape(3, 3)
        if K[0, 0] > 0 and K[1, 1] > 0:
            self.K = K

    def on_images(self, rgb_msg, depth_msg):
        if self.K is None:
            self.get_logger().warn("no camera_info yet", throttle_duration_sec=5.0)
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except (CvBridgeError, TypeError, ValueError) as e:      # TypeError: data shorter than the header says
            self.get_logger().warn(f"unusable image: {e}", throttle_duration_sec=5.0)
            return
        tf = self.lookup(rgb_msg)
        if tf is None:
            return
        r, q = tf.transform.translation, tf.transform.rotation
        try:
            rotation = vision.quat_to_matrix(q.x, q.y, q.z, q.w)
        except ValueError:
            self.get_logger().warn("invalid rotation in the transform", throttle_duration_sec=5.0)
            return
        translation = np.array([r.x, r.y, r.z])

        blobs, mask = vision.locate_blobs(bgr, depth, self.K, self.p)
        out = PoseArray()
        out.header.stamp, out.header.frame_id = rgb_msg.header.stamp, self.target_frame
        for b in blobs:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, vision.to_target_frame(
                b.camera_xyz, rotation, translation))
            pose.orientation.w = 1.0
            out.poses.append(pose)
        self.poses.publish(out)
        self.publish_markers(out)
        if self.debug.get_subscription_count():
            self.debug.publish(self.bridge.cv2_to_imgmsg(vision.draw_fruit(bgr, mask, blobs), "bgr8"))

    def lookup(self, image):
        """Transform from the image's frame to the target frame, at the image time if possible.

        A camera stamp can be a few milliseconds newer than the latest transform (TF arrives
        at the joint-state rate). Then the newest transform is used if it is at most
        tf_max_age seconds old: the arm moves slowly, so that is worth far less than a
        dropped frame. Anything older, or any other failure, skips the frame.
        """
        stamp = Time.from_msg(image.header.stamp)
        frame = image.header.frame_id
        try:
            return self.tf.lookup_transform(self.target_frame, frame, stamp, timeout=Duration(seconds=self.tf_timeout))
        except tf2_ros.ExtrapolationException:
            pass
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"no transform, frame skipped: {e}", throttle_duration_sec=5.0)
            return None
        try:
            latest = self.tf.lookup_transform(self.target_frame, frame, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"no transform, frame skipped: {e}", throttle_duration_sec=5.0)
            return None
        age = abs((stamp - Time.from_msg(latest.header.stamp)).nanoseconds) * 1e-9
        if age > self.tf_max_age:
            self.get_logger().warn(f"newest transform is {age:.2f} s from the image: frame skipped",
                                   throttle_duration_sec=5.0)
            return None
        return latest

    def publish_markers(self, poses):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for k, p in enumerate(poses.poses):
            m = Marker()
            m.header = poses.header
            m.ns, m.id, m.type, m.action = "fruit", k, Marker.SPHERE, Marker.ADD
            m.pose = p
            m.scale.x = m.scale.y = m.scale.z = 2 * self.p.fruit_radius
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.15, 0.1, 0.8
            arr.markers.append(m)
        self.markers.publish(arr)


def main():
    rclpy.init()
    node = FruitDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
