#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class GoalEvaluator(Node):
    def __init__(self, args):
        super().__init__("navigation_goal_evaluator")
        self.args = args
        self.finished = False
        self.recording = False
        self.started = None
        self.previous_position = None
        self.previous_stamp = None
        self.distance = 0.0
        self.samples = 0
        self.invalid_measurement = False

        self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data
        )
        self.client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

    def on_odom(self, message):
        if not self.recording:
            return

        stamp = (
            message.header.stamp.sec
            + message.header.stamp.nanosec * 1e-9
        )
        point = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )

        if not all(math.isfinite(value) for value in point):
            self.invalid_measurement = True
            return

        if self.previous_stamp is not None:
            if stamp < self.previous_stamp:
                self.invalid_measurement = True
            elif stamp == self.previous_stamp:
                return
            elif self.previous_position is not None:
                self.distance += math.hypot(
                    point[0] - self.previous_position[0],
                    point[1] - self.previous_position[1],
                )

        self.previous_position = point
        self.previous_stamp = stamp
        self.samples += 1

    def send_goal(self):
        if not self.client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("Nav2 navigate_to_pose server unavailable")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.args.x
        goal.pose.pose.position.y = self.args.y
        goal.pose.pose.orientation.z = math.sin(self.args.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.args.yaw / 2.0)

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_response)
        self.get_logger().info("Goal sent; waiting for acceptance")

    def on_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.write_result("REJECTED")
            return

        self.started = self.get_clock().now().nanoseconds
        self.recording = True
        handle.get_result_async().add_done_callback(self.on_result)
        self.get_logger().info("Goal accepted; recording odometry")

    def on_result(self, future):
        status = future.result().status
        labels = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        self.write_result(labels.get(status, f"STATUS_{status}"))

    def write_result(self, outcome):
        self.recording = False
        elapsed = ""
        if self.started is not None:
            elapsed = (
                self.get_clock().now().nanoseconds - self.started
            ) / 1e9

        valid = (
            self.samples >= 2
            and not self.invalid_measurement
            and elapsed != ""
            and elapsed >= 0
        )

        output = Path(self.args.csv).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        needs_header = not output.exists() or output.stat().st_size == 0

        with output.open("a", newline="") as stream:
            writer = csv.writer(stream)
            if needs_header:
                writer.writerow([
                    "run_id", "method", "goal_x", "goal_y", "goal_yaw",
                    "outcome", "elapsed_sim_s", "odom_distance_m",
                    "odom_samples", "metrics_valid",
                ])
            writer.writerow([
                self.args.run_id, "nav2_astar_rpp",
                self.args.x, self.args.y, self.args.yaw,
                outcome, elapsed,
                round(self.distance, 4) if valid else "",
                self.samples, valid,
            ])

        self.get_logger().info(
            f"{outcome}; metrics_valid={valid}; CSV: {output}"
        )
        self.finished = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--csv", required=True)

    arguments = rclpy.utilities.remove_ros_args()[1:]
    args = parser.parse_args(arguments)

    if not all(math.isfinite(v) for v in (args.x, args.y, args.yaw)):
        parser.error("Goal coordinates and yaw must be finite")

    rclpy.init()
    node = GoalEvaluator(args)
    try:
        node.send_goal()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().warning(
            "Recording interrupted. No completed result saved. "
            "The Nav2 goal may still be active; cancel it in RViz."
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
