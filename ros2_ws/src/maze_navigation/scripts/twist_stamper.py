#!/usr/bin/env python3
"""Convert Nav2's geometry_msgs/Twist (cmd_vel_nav2) to TwistStamped (/cmd_vel).

Jazzy Nav2 publishes Twist, while the Jazzy TurtleBot3 Gazebo bridge expects
TwistStamped on /cmd_vel. Run only for the Nav2 pipeline: DQN and the auto-mapper
publish /cmd_vel themselves, so never start this at the same time as them.
"""
import sys

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistStamper(Node):
    def __init__(self):
        super().__init__("twist_stamper")
        self.declare_parameter("input_topic", "cmd_vel_nav2")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("frame_id", "base_link")
        self.frame = self.get_parameter("frame_id").value
        self.pub = self.create_publisher(TwistStamped, self.get_parameter("output_topic").value, 10)
        self.create_subscription(Twist, self.get_parameter("input_topic").value, self.on_twist, 10)

    def on_twist(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
