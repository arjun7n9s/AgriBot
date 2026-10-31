"""Draw the running ROS 2 graph (nodes and the topics between them) with Graphviz.

A clean alternative to screenshotting rqt_graph: it reads the live graph through
rclpy, so it captures nothing but ROS. Hidden nodes, parameter and lifecycle
topics are left out.

    python3 tools/sim/ros_graph.py <out_stem>      # writes <out_stem>.dot/.png/.svg
"""
import subprocess
import sys
import time

import rclpy

SKIP_TOPICS = ("/parameter_events", "/rosout", "transition_event", "/capture")
SKIP_NODES = ("_ros2cli", "ros_graph", "rqt", "rviz", "launch_ros")


def main():
    stem = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("ros_graph_snapshot")
    time.sleep(2.0)                                    # let discovery settle
    names = [(n, ns) for n, ns in node.get_node_names_and_namespaces()
             if not any(s in n for s in SKIP_NODES)]
    edges, topics = set(), set()
    for n, ns in names:
        full = (ns.rstrip("/") + "/" + n)
        for t, _ in node.get_publisher_names_and_types_by_node(n, ns):
            if not any(s in t for s in SKIP_TOPICS):
                edges.add((full, t)); topics.add(t)
        for t, _ in node.get_subscriber_names_and_types_by_node(n, ns):
            if not any(s in t for s in SKIP_TOPICS):
                edges.add((t, full)); topics.add(t)
    lines = ["digraph agribot {", '  rankdir=LR; bgcolor="white"; pad=0.3; nodesep=0.18; ranksep=0.9;',
             '  node [fontname="Helvetica", fontsize=11];', '  edge [color="#8a8f98", arrowsize=0.6];']
    for n, ns in names:
        full = ns.rstrip("/") + "/" + n
        lines.append(f'  "{full}" [shape=box, style="rounded,filled", fillcolor="#e6f2e6", color="#3e8e41"];')
    for t in sorted(topics):
        lines.append(f'  "{t}" [shape=ellipse, style=filled, fillcolor="#eef3fb", color="#6f9bd1", fontsize=10];')
    for a, b in sorted(edges):
        lines.append(f'  "{a}" -> "{b}";')
    lines.append("}")
    with open(stem + ".dot", "w") as f:
        f.write("\n".join(lines))
    for fmt in ("png", "svg"):
        subprocess.run(["dot", f"-T{fmt}", "-Gdpi=110", stem + ".dot", "-o", f"{stem}.{fmt}"], check=True)
    print(f"{len(names)} nodes, {len(topics)} topics, {len(edges)} edges -> {stem}.png/.svg")


if __name__ == "__main__":
    main()
