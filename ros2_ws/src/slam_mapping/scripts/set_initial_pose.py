#!/usr/bin/env python3
"""Publish /initialpose repeatedly until AMCL starts publishing /amcl_pose."""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class InitialPose(Node):
    def __init__(self, x, y, yaw, timeout):
        super().__init__("set_initial_pose")
        self.x, self.y, self.yaw = x, y, yaw
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, qos)
        self.done = False
        self.tries = 0
        self.deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        self.create_timer(2.0, self.tick)

    def tick(self):
        if self.done:
            return
        if self.get_clock().now().nanoseconds > self.deadline:
            self.get_logger().error("AMCL did not confirm the initial pose in time")
            self.done = True
            return
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = self.x, self.y
        m.pose.pose.orientation.z = math.sin(self.yaw / 2)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05
        m.pose.covariance[35] = 0.05
        self.pub.publish(m)
        self.tries += 1

    def on_pose(self, _):
        if self.tries and not self.done:
            self.get_logger().info(f"AMCL pose received after {self.tries} publishes")
            self.done = True


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--x", type=float, default=0.25)
    p.add_argument("--y", type=float, default=0.0)
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument("--timeout", type=float, default=120.0)
    args = p.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = InitialPose(args.x, args.y, args.yaw, args.timeout)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
