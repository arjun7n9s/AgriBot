#!/usr/bin/env python3
"""Save what the robot's cameras see when asked.

Keeps the latest frame from each camera and, on every /capture message, writes
<out_dir>/cameras/<label>_<camera>.png. Depth is colour-mapped (near = warm).
Does nothing if out_dir is empty.
"""
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

TOPICS = {"front": "/front_camera/image", "scout": "/scout_camera/image",
          "arm_rgb": "/arm_camera/image", "arm_depth": "/arm_camera/depth_image"}


class Capture(Node):
    def __init__(self):
        super().__init__("capture_frames")
        self.out = self.declare_parameter("out_dir", "").value
        self.bridge = CvBridge()
        self.latest = {}
        for name, topic in TOPICS.items():
            self.create_subscription(Image, topic, lambda m, n=name: self.latest.__setitem__(n, m), 2)
        self.create_subscription(String, "/capture", self.on_capture, 10)

    def on_capture(self, msg):
        if not self.out:
            return
        folder = os.path.join(self.out, "cameras")
        os.makedirs(folder, exist_ok=True)
        for name, img in self.latest.items():
            if name == "arm_depth":
                d = self.bridge.imgmsg_to_cv2(img, "passthrough").astype(np.float32)
                d = np.nan_to_num(d, nan=3.0, posinf=3.0)
                d8 = np.clip(255 * (1 - (d - 0.05) / 1.2), 0, 255).astype(np.uint8)
                frame = cv2.applyColorMap(d8, cv2.COLORMAP_TURBO)
            else:
                frame = self.bridge.imgmsg_to_cv2(img, "bgr8")
            cv2.imwrite(os.path.join(folder, f"{msg.data}_{name}.png"), frame)
        self.get_logger().info(f"saved camera frames: {msg.data}")


def main():
    rclpy.init()
    rclpy.spin(Capture())


if __name__ == "__main__":
    main()
