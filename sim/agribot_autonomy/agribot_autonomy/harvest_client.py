"""Send one Harvest goal, print its progress and exit with its outcome.

    ros2 run agribot_autonomy harvest_client --picks 3
    ros2 run agribot_autonomy harvest_client --picks 3 --distance 4.0 --wait 120

Exit code 0: the goal finished normally. 1: it was aborted, rejected or canceled.
2: no action server appeared within --wait seconds.
"""
import argparse
import sys

import rclpy
from agribot_interfaces.action import Harvest
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--picks", type=int, default=2, help="pick attempts (default 2)")
    ap.add_argument("--distance", type=float, default=0.0, help="metres of row, 0 = the mission's row_length")
    ap.add_argument("--wait", type=float, default=60.0, help="seconds to wait for the action server")
    args = ap.parse_args(remove_ros_args(argv if argv is not None else sys.argv)[1:])

    rclpy.init(args=argv)
    node = Node("harvest_client")
    client = ActionClient(node, Harvest, "harvest")
    code = 2
    try:
        if not client.wait_for_server(timeout_sec=args.wait):
            node.get_logger().error(f"no /harvest action server after {args.wait:.0f} s")
            return 2
        goal = Harvest.Goal(max_picks=args.picks, max_distance=args.distance)
        sent = client.send_goal_async(goal, feedback_callback=lambda fb: node.get_logger().info(
            f"[{fb.feedback.attempts} tried, {fb.feedback.verified} verified, {fb.feedback.distance:.2f} m] "
            f"{fb.feedback.detail}", throttle_duration_sec=2.0))
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if not handle.accepted:
            node.get_logger().error("goal rejected")
            return 1
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(node, done)
        result = done.result().result
        node.get_logger().info(f"result: success={result.success} attempted={result.attempted} "
                               f"verified={result.verified} distance={result.distance:.2f} m: {result.message}")
        code = 0 if result.success else 1
    except KeyboardInterrupt:
        code = 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
