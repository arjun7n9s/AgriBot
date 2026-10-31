#!/usr/bin/env python3
"""Find red tomatoes with the arm RGB-D camera and locate them in 3D.

A deliberately simple first detector: a colour rule picks red pixels, connected
blobs become fruit candidates, the depth image gives their range, and TF puts
them in the rover's frame. It finds red fruit, which includes the red underside
of half-ripe ones: telling ripe from half-ripe is the job of the learned
ripeness model that comes next. The outputs will stay the same.

    /arm_camera/image, /arm_camera/depth_image, /arm_camera/camera_info
      ->  /fruit/detections   (geometry_msgs/PoseArray, frame base_footprint):
                               estimated fruit centres
          /fruit/markers      (visualization_msgs/MarkerArray) for RViz
          /fruit/debug_image  colour mask, blobs and ranges
"""
import cv2
import message_filters
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray

FRUIT_RADIUS = 0.028     # m: the depth hits the near surface; the centre is this far behind
MIN_AREA = 150           # px
MAX_RANGE = 0.9          # m: beyond this the arm cannot reach anyway


def red_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = ((h <= 8) | (h >= 170)) & (s >= 120) & (v >= 60)
    mask = red.astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class FruitDetector(Node):
    def __init__(self):
        super().__init__("fruit_detector")
        self.target_frame = self.declare_parameter("target_frame", "base_footprint").value
        self.bridge = CvBridge()
        self.K = None
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.create_subscription(CameraInfo, "/arm_camera/camera_info", self.on_info, 2)
        rgb = message_filters.Subscriber(self, Image, "/arm_camera/image")
        depth = message_filters.Subscriber(self, Image, "/arm_camera/depth_image")
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb, depth], 4, 0.05)
        self.sync.registerCallback(self.on_images)
        self.poses = self.create_publisher(PoseArray, "/fruit/detections", 5)
        self.markers = self.create_publisher(MarkerArray, "/fruit/markers", 5)
        self.debug = self.create_publisher(Image, "/fruit/debug_image", 2)

    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def on_images(self, rgb_msg, depth_msg):
        if self.K is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
        try:
            tf = self.tf.lookup_transform(self.target_frame, rgb_msg.header.frame_id, Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return
        R = quat_to_matrix(tf.transform.rotation)
        t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]

        mask = red_mask(bgr)
        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
        out = PoseArray()
        out.header.stamp, out.header.frame_id = rgb_msg.header.stamp, self.target_frame
        found = []
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < MIN_AREA:
                continue
            d = depth[labels == i]
            d = d[np.isfinite(d) & (d > 0.05)]
            if d.size < 20:
                continue
            z = float(np.percentile(d, 30))          # near surface of the fruit
            if z > MAX_RANGE:
                continue
            u, v = cents[i]
            ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            surface = ray * z
            centre = surface + ray / np.linalg.norm(ray) * FRUIT_RADIUS
            p = R @ centre + t
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, p)
            pose.orientation.w = 1.0
            out.poses.append(pose)
            found.append((i, u, v, z, p))
        self.poses.publish(out)
        self.publish_markers(out)
        if self.debug.get_subscription_count():
            self.debug.publish(self.bridge.cv2_to_imgmsg(self.draw(bgr, mask, stats, found), "bgr8"))

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
            m.scale.x = m.scale.y = m.scale.z = 2 * FRUIT_RADIUS
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.15, 0.1, 0.8
            arr.markers.append(m)
        self.markers.publish(arr)

    @staticmethod
    def draw(bgr, mask, stats, found):
        out = bgr.copy()
        tint = np.zeros_like(out)
        tint[mask > 0] = (255, 255, 0)
        out = cv2.addWeighted(out, 1.0, tint, 0.35, 0)
        for i, u, v, z, p in found:
            x, y, w, h = stats[i, :4]
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(out, f"{z:.2f} m", (x, max(14, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(out, f"{len(found)} red fruit", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)
        return out


def main():
    rclpy.init()
    rclpy.spin(FruitDetector())


if __name__ == "__main__":
    main()
