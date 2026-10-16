#!/usr/bin/env python3
"""Dead-man safety for the simulated robot.

Gazebo's DiffDrive keeps executing the LAST /cmd_vel forever. If whatever was driving
(teleop, mapping tour, DQN, Nav2) dies or stops publishing, the robot would run away.
This node publishes ONE zero command when a non-zero command has gone stale (no new
/cmd_vel for `timeout` seconds). It never publishes while commands keep arriving.
"""
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

OWN = "cmd_vel_watchdog"


class Watchdog(Node):
    def __init__(self):
        super().__init__(OWN)
        self.timeout = self.declare_parameter("timeout", 0.5).value
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_cmd, 10)
        self.last, self.moving = 0.0, False
        self.create_timer(0.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_cmd(self, m):
        if m.header.frame_id == OWN:
            return
        t = m.twist
        self.moving = any(abs(v) > 1e-6 for v in (t.linear.x, t.linear.y, t.linear.z,
                                                    t.angular.x, t.angular.y, t.angular.z))
        self.last = self.now()

    def tick(self):
        if self.moving and self.now() - self.last > self.timeout:
            self.moving = False
            z = TwistStamped()
            z.header.stamp = self.get_clock().now().to_msg()
            z.header.frame_id = OWN
            self.pub.publish(z)
            self.get_logger().warn("no /cmd_vel for %.1f s: sent stop" % self.timeout)


def main():
    rclpy.init()
    rclpy.spin(Watchdog())


if __name__ == "__main__":
    sys.exit(main())
