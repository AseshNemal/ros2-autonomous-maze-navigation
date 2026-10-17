#!/usr/bin/env python3
"""Send one Nav2 NavigateToPose goal from the command line and report progress and the result.

Used by tools/send_goal.sh (do not call ros2 CLI `action`: it is not installed on the team VM).
  send_goal.py X Y [--timeout SECONDS]      goal in world/map coordinates (metres), frame `map`
Exit status 0 only when Nav2 reports SUCCEEDED.
"""
import argparse
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("--timeout", type=float, default=120.0, help="seconds to wait for the result")
    a = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = Node("send_goal")
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=15.0):
        print("FAIL: /navigate_to_pose is not available - start Nav2 first (bash tools/demo.sh)")
        return 2
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = a.x, a.y
    goal.pose.pose.orientation.w = 1.0
    last = [0.0]

    def feedback(msg):
        now = time.monotonic()
        if now - last[0] >= 2.0:
            last[0] = now
            f = msg.feedback
            print(f"  distance remaining {f.distance_remaining:.2f} m, recoveries {f.number_of_recoveries}", flush=True)

    t0 = time.monotonic()
    send = client.send_goal_async(goal, feedback_callback=feedback)
    rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print("FAIL: goal rejected (is the initial pose set? run tools/demo.sh or set_initial_pose.py)")
        return 2
    print("goal accepted", flush=True)
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=a.timeout)
    if not res.done():
        handle.cancel_goal_async()
        print(f"FAIL: no result within {a.timeout:.0f} s, goal cancelled")
        return 1
    status = res.result().status
    name = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED"}.get(status, str(status))
    print(f"RESULT: {name} after {time.monotonic() - t0:.1f} s")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if status == GoalStatus.STATUS_SUCCEEDED else 1


if __name__ == "__main__":
    sys.exit(main())
