#!/usr/bin/env python3
"""Wall follower (right hand, then left hand) for unattended mapping (publishes TwistStamped /cmd_vel).

Only for the mapping phase: stop it before Nav2 or DQN. Simulation only.
  ros2 run slam_mapping auto_map_drive.py --duration 400
The first half follows the right wall, the second half the left wall, so both sides of
every wall (including interior wall islands) are scanned.
Behaviour is a simple reactive heuristic; coverage must be judged from the map
(see validate_map.py). It is not a proven exploration algorithm.
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def sector_min(scan, centre_deg, half_deg):
    """Minimum valid range within centre +/- half (degrees, 0 = forward, CCW+)."""
    best = float("inf")
    n = len(scan.ranges)
    for k in range(-half_deg, half_deg + 1):
        ang = math.radians(centre_deg + k)
        idx = int(round((ang - scan.angle_min) / scan.angle_increment)) % n
        r = scan.ranges[idx]
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
            best = min(best, r)
    return best


class WallFollower(Node):
    def __init__(self, duration, target, speed, side="both"):
        super().__init__("auto_map_drive")
        self.target, self.speed = target, speed
        self.side = side
        self.total = duration
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.t0 = self.get_clock().now().nanoseconds
        self.end = self.t0 + int(duration * 1e9)
        self.finished = False

    def command(self, lin, ang):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x, m.twist.angular.z = float(lin), float(ang)
        self.pub.publish(m)

    def on_scan(self, scan):
        if self.get_clock().now().nanoseconds > self.end:
            self.command(0.0, 0.0)
            self.finished = True
            return
        elapsed = (self.get_clock().now().nanoseconds - self.t0) / 1e9
        left_hand = self.side == "left" or (self.side == "both" and elapsed > self.total / 2)
        s = -1.0 if left_hand else 1.0                   # +1: wall on the right, -1: wall on the left
        front = sector_min(scan, 0, 20)
        wall = sector_min(scan, -90 * s, 25)
        if front < 0.28:
            self.command(0.0, 0.45 * s)                   # blocked: turn away from the wall (slow: ~5 Hz lidar needs overlapping scans)
        elif wall > 0.45:
            self.command(self.speed * 0.6, -0.5 * s)     # lost wall: curve back towards it
        else:
            ang = max(-0.5, min(0.5, -3.0 * (wall - self.target) * s))
            self.command(self.speed, ang)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=600.0, help="simulated seconds")
    p.add_argument("--target", type=float, default=0.22, help="right wall distance (m)")
    p.add_argument("--speed", type=float, default=0.10)
    p.add_argument("--side", choices=["right", "left", "both"], default="both")
    a = p.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = WallFollower(a.duration, a.target, a.speed, a.side)
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
