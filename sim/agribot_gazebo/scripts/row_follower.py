#!/usr/bin/env python3
"""Follow the furrow with the front camera.

The furrow is bare soil between two raised beds covered in black plastic mulch.
For each image row in the lower part of the frame, the soil run nearest the
image centre is taken as the furrow, and a line is fitted through the run
midpoints. Two points on that line steer the rover: the near point (bottom of
the image) mostly measures sideways offset, the far point (top of the band)
mostly measures heading. Under perspective a sideways offset also tilts the
line, so the law works on the two points rather than on the tilt.

    /front_camera/image   ->  /cmd_vel                  (while enabled)
    /row_follower/enable  (std_msgs/Bool)               start / stop following
    /row_follower/state   (std_msgs/Float32MultiArray)  [near, far, confidence]
    /row_follower/debug_image                           the mask, runs and fitted line

Near and far are in half-widths of the image (-1 left .. +1 right). No odometry
or map is used for steering.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

ROWS = (0.60, 0.97)      # band of the image used, as fractions of its height
STEP = 6                 # sample every STEP image rows
MIN_RUN = 0.06           # a furrow run must be at least this fraction of the width


def soil_mask(bgr):
    """Bare soil: brownish, not too dark (mulch) and not green (leaves)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    soil = (h >= 3) & (h <= 30) & (s >= 25) & (v >= 45)
    mask = soil.astype(np.uint8) * 255
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (15, 3)))


def furrow_line(bgr):
    """Return (near, far, confidence, points) for the furrow in the image."""
    h, w = bgr.shape[:2]
    mask = soil_mask(bgr)
    points = []
    centre = w / 2
    for y in range(int(ROWS[0] * h), int(ROWS[1] * h), STEP):
        row = mask[y] > 0
        edges = np.flatnonzero(np.diff(np.r_[0, row.astype(np.int8), 0]))
        runs = list(zip(edges[::2], edges[1::2]))
        runs = [(a, b) for a, b in runs if b - a >= MIN_RUN * w]
        if not runs:
            continue
        a, b = min(runs, key=lambda r: abs((r[0] + r[1]) / 2 - centre))
        if a <= 1 and b >= w - 1:           # whole row is soil: no beds in view here
            continue
        points.append((y, (a + b) / 2.0))
        centre = (a + b) / 2.0              # track the run upward from the bottom
    if len(points) < 5:
        return 0.0, 0.0, 0.0, points
    ys, xs = np.array(points).T
    slope, intercept = np.polyfit(ys, xs, 1)
    near = (slope * (h - 1) + intercept - w / 2) / (w / 2)
    far = (slope * ROWS[0] * h + intercept - w / 2) / (w / 2)
    residual = np.abs(xs - (slope * ys + intercept)).mean() / w
    confidence = float(np.clip(len(points) / ((ROWS[1] - ROWS[0]) * h / STEP), 0, 1) * (1 - min(1, 10 * residual)))
    return float(near), float(far), confidence, points


class RowFollower(Node):
    def __init__(self):
        super().__init__("row_follower")
        p = self.declare_parameter
        self.speed = p("speed", 0.15).value              # m/s: slow enough for the arm camera to confirm fruit
        self.k_near = p("k_near", 1.4).value              # rad/s per half-width
        self.k_far = p("k_far", 1.6).value                # rad/s per half-width
        self.min_conf = p("min_confidence", 0.35).value
        self.enabled = p("start_enabled", False).value
        self.bridge = CvBridge()
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state = self.create_publisher(Float32MultiArray, "/row_follower/state", 10)
        self.debug = self.create_publisher(Image, "/row_follower/debug_image", 2)
        self.create_subscription(Image, "/front_camera/image", self.on_image, 2)
        self.create_subscription(Bool, "/row_follower/enable", self.on_enable, 10)

    def on_enable(self, msg):
        if self.enabled and not msg.data:
            self.cmd.publish(Twist())
        self.enabled = msg.data
        self.get_logger().info("following the row" if msg.data else "stopped")

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        near, far, conf, points = furrow_line(bgr)
        self.state.publish(Float32MultiArray(data=[near, far, conf]))
        if self.enabled:
            twist = Twist()
            if conf >= self.min_conf:
                twist.linear.x = self.speed
                # Furrow to the right of the image centre (near or far > 0): turn right,
                # which is a negative yaw rate.
                twist.angular.z = float(-(self.k_near * near + self.k_far * far))
            self.cmd.publish(twist)          # low confidence: stop and wait
        if self.debug.get_subscription_count():
            self.debug.publish(self.bridge.cv2_to_imgmsg(self.draw(bgr, points, near, far, conf), "bgr8"))

    @staticmethod
    def draw(bgr, points, near, far, conf):
        out = bgr.copy()
        h, w = out.shape[:2]
        tint = np.zeros_like(out)
        tint[soil_mask(bgr) > 0] = (60, 160, 255)
        out = cv2.addWeighted(out, 1.0, tint, 0.35, 0)
        for y, x in points:
            cv2.circle(out, (int(x), int(y)), 3, (0, 255, 255), -1)
        if points:
            x0, x1 = near * w / 2 + w / 2, far * w / 2 + w / 2
            cv2.line(out, (int(x0), h - 1), (int(x1), int(ROWS[0] * h)), (0, 255, 0), 2)
        cv2.line(out, (w // 2, h - 1), (w // 2, int(ROWS[0] * h)), (255, 255, 255), 1)
        cv2.putText(out, f"near {near:+.2f}  far {far:+.2f}  conf {conf:.2f}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return out


def main():
    rclpy.init()
    rclpy.spin(RowFollower())


if __name__ == "__main__":
    main()
