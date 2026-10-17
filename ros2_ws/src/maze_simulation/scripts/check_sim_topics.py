#!/usr/bin/env python3
"""Verify the simulation interface contract; prints PASS/FAIL per check.

Run while maze_simulation.launch.py is running:
  ros2 run maze_simulation check_sim_topics.py [--duration 5] [--json out.json]
Checks: /scan, /odom rates and frames, /clock, /tf odom->base_footprint,
/cmd_vel message type (must be geometry_msgs/msg/TwistStamped).
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

EXPECTED_CMD_VEL_TYPE = "geometry_msgs/msg/TwistStamped"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()
    rclpy.init()
    node = Node("check_sim_topics")
    count = {"scan": 0, "odom": 0, "clock": 0, "tf": 0}
    seen = {"scan_frame": None, "odom_frame": None, "odom_child": None,
            "tf_pairs": set(), "scan_beams": 0, "scan_range_max": None}

    def on_scan(m):
        count["scan"] += 1
        seen["scan_frame"] = m.header.frame_id
        seen["scan_beams"] = len(m.ranges)
        seen["scan_range_max"] = m.range_max

    def on_odom(m):
        count["odom"] += 1
        seen["odom_frame"], seen["odom_child"] = m.header.frame_id, m.child_frame_id

    def on_tf(m):
        count["tf"] += 1
        for t in m.transforms:
            seen["tf_pairs"].add((t.header.frame_id, t.child_frame_id))

    def on_clock(_):
        count["clock"] += 1

    node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data)
    node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/tf", on_tf, 100)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    cmd_types = dict(node.get_topic_names_and_types()).get("/cmd_vel", [])
    d = args.duration
    results = {
        "scan_rate_hz": count["scan"] / d, "odom_rate_hz": count["odom"] / d,
        "clock_rate_hz": count["clock"] / d, "tf_msgs": count["tf"],
        "scan_frame": seen["scan_frame"], "scan_beams": seen["scan_beams"],
        "scan_range_max": seen["scan_range_max"],
        "odom_frame": seen["odom_frame"], "odom_child": seen["odom_child"],
        "tf_pairs": sorted(map(list, seen["tf_pairs"])), "cmd_vel_types": cmd_types,
    }
    # exactly one robot in the custom_maze world, named `burger` (main's launcher), and the right world
    import re
    import subprocess
    try:
        pose = subprocess.run(["timeout", "10", "gz", "topic", "-e", "-n", "1", "-t",
                               "/world/custom_maze/dynamic_pose/info"],
                              capture_output=True, text=True).stdout
    except FileNotFoundError:
        pose = ""
    names = re.findall(r'name: "([^"]+)"', pose)
    results["gz_entities"] = sorted(set(names))
    checks = [
        ("Gazebo world is custom_maze (/world/custom_maze/dynamic_pose/info answers)", bool(names)),
        ("exactly one robot entity 'burger' (no duplicate/turtlebot3_burger)",
         names.count("burger") == 1 and not any(n.startswith("turtlebot3") for n in names)),
        ("scan frame is base_scan", seen["scan_frame"] == "base_scan"),
        ("/scan publishing (>=3 Hz)", results["scan_rate_hz"] >= 3),
        ("/odom publishing (>=5 Hz)", results["odom_rate_hz"] >= 5),
        ("/clock publishing", results["clock_rate_hz"] > 0),
        ("odom frame is 'odom'", seen["odom_frame"] == "odom"),
        ("odom child is base_footprint", seen["odom_child"] == "base_footprint"),
        ("TF odom->base_footprint present", ("odom", "base_footprint") in seen["tf_pairs"]),
        ("/cmd_vel type is TwistStamped", EXPECTED_CMD_VEL_TYPE in cmd_types),
    ]
    ok = True
    for name, passed in checks:
        ok &= bool(passed)
        print(("PASS  " if passed else "FAIL  ") + name)
    print(json.dumps(results, indent=2))
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"results": results,
                       "checks": {n: bool(p) for n, p in checks}}, f, indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
