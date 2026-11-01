"""A small harness for node tests: one rclpy context, a test node spinning in a thread.

Tests create the node under test with `parameter_overrides`, add it to the harness
executor, then publish inputs and wait for outputs. No Gazebo, no real clock: nodes run
on wall time.
"""
import os
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "87")            # keep clear of a running demo
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402


def overrides(mapping):
    return [Parameter(k, value=v) for k, v in mapping.items()]


class Harness:
    def __init__(self):
        rclpy.init()                    # the default context: tf2_ros listeners with a thread need it
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.node = Node("test_harness")
        self.executor.add_node(self.node)
        self.nodes = []
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def make(self, cls, **params):
        """Create a node of `cls` with these parameter overrides, attached to this harness."""
        return self.add(cls(parameter_overrides=overrides(params)))

    def remove(self, node):
        self.executor.remove_node(node)
        node.destroy_node()
        self.nodes.remove(node)

    def wait_for(self, condition, timeout=5.0, interval=0.02):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return True
            time.sleep(interval)
        return condition()

    def collect(self, msg_type, topic, qos=10):
        """Subscribe and return a list that fills with received messages."""
        got = []
        self.node.create_subscription(msg_type, topic, got.append, qos)
        return got

    def close(self):
        for n in list(self.nodes):
            self.remove(n)
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()
        self.thread.join(timeout=2)
