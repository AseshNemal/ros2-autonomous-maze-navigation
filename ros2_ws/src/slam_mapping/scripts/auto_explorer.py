#!/usr/bin/env python3
"""
Autonomous Maze Explorer for TurtleBot3 Burger in ROS 2 Jazzy.
Uses real-time 360-degree LiDAR clearance sensing to navigate mazes
without getting trapped in corners or continuous rotation loops.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        # State
        self.state = 'FORWARD'  # 'FORWARD', 'ALIGNING', 'BACKUP'
        self.target_yaw_dir = 1.0  # +1 for left, -1 for right
        self.backup_ticks = 0

        # Scan data
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.has_scan = False

        # Anti-stuck tracking via odom
        self.last_x = 0.0
        self.last_y = 0.0
        self.odom_count = 0
        self.stuck_counter = 0

        self.get_logger().info('Intelligent Maze Explorer initialized.')

    def odom_callback(self, msg: Odometry):
        self.odom_count += 1
        if self.odom_count % 30 == 0:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            dist = math.hypot(x - self.last_x, y - self.last_y)

            # If commanded forward but not moving
            if self.state == 'FORWARD' and dist < 0.03:
                self.stuck_counter += 1
                if self.stuck_counter >= 3:
                    self.get_logger().warn('Stuck detected, reversing.')
                    self.state = 'BACKUP'
                    self.backup_ticks = 15
                    self.stuck_counter = 0
            else:
                self.stuck_counter = 0

            self.last_x = x
            self.last_y = y

    def scan_callback(self, msg: LaserScan):
        self.ranges = list(msg.ranges)
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.has_scan = True

    def get_sector_min(self, start_deg, end_deg):
        """Minimum obstacle distance in an angular window [start_deg, end_deg]."""
        if not self.has_scan or not self.ranges:
            return 1.0
        n = len(self.ranges)
        vals = []
        for deg in range(start_deg, end_deg + 1):
            idx = int((deg % 360) * n / 360)
            r = self.ranges[idx]
            if not math.isnan(r) and not math.isinf(r) and r >= self.range_min:
                vals.append(r)
        return min(vals) if vals else 3.5

    def get_sector_mean(self, start_deg, end_deg):
        """Mean obstacle distance in an angular window [start_deg, end_deg]."""
        if not self.has_scan or not self.ranges:
            return 1.0
        n = len(self.ranges)
        vals = []
        for deg in range(start_deg, end_deg + 1):
            idx = int((deg % 360) * n / 360)
            r = self.ranges[idx]
            if not math.isnan(r) and not math.isinf(r) and r >= self.range_min:
                vals.append(r)
        return sum(vals) / len(vals) if vals else 3.5

    def find_best_open_direction(self):
        """Find whether left (+1) or right (-1) has the largest corridor opening."""
        # Compare left side (30 to 150 deg) vs right side (210 to 330 deg)
        left_clearance = self.get_sector_mean(30, 150)
        right_clearance = self.get_sector_mean(210, 330)

        # Also check sharp left (90 deg) vs sharp right (270 deg)
        left_90 = self.get_sector_min(75, 105)
        right_270 = self.get_sector_min(255, 285)

        if left_clearance + left_90 > right_clearance + right_270:
            return 1.0  # Turn left
        else:
            return -1.0  # Turn right

    def control_loop(self):
        if not self.has_scan:
            return

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_footprint'

        linear_x = 0.0
        angular_z = 0.0

        # Sector distances
        # 0 deg is front, 90 deg is left, 270 deg is right
        front_dist = min(self.get_sector_min(0, 20), self.get_sector_min(340, 360))
        left_dist = self.get_sector_min(65, 115)
        right_dist = self.get_sector_min(245, 295)
        front_left = self.get_sector_min(20, 60)
        front_right = self.get_sector_min(300, 340)

        # Distance thresholds
        STOP_DIST = 0.28
        OPEN_DIST = 0.42
        CRITICAL_DIST = 0.18

        if self.state == 'BACKUP':
            linear_x = -0.08
            angular_z = 0.0
            self.backup_ticks -= 1
            if self.backup_ticks <= 0:
                self.target_yaw_dir = self.find_best_open_direction()
                self.state = 'ALIGNING'

        elif self.state == 'FORWARD':
            # Critical emergency stop if too close to any wall
            if front_dist < CRITICAL_DIST:
                self.state = 'BACKUP'
                self.backup_ticks = 10

            # Wall or dead end ahead: stop and align with open corridor
            elif front_dist < STOP_DIST:
                linear_x = 0.0
                self.target_yaw_dir = self.find_best_open_direction()
                self.state = 'ALIGNING'

            else:
                # Drive forward along corridor
                linear_x = 0.12 if front_dist > 0.50 else 0.07

                # Center between left and right walls
                side_diff = left_dist - right_dist
                angular_z = max(min(0.45 * side_diff, 0.35), -0.35)

                # Nudge away if approaching diagonal wall
                if front_left < 0.20:
                    angular_z = -0.3
                elif front_right < 0.20:
                    angular_z = 0.3

        elif self.state == 'ALIGNING':
            # Rotate strictly in place towards the open corridor
            linear_x = 0.0
            angular_z = 0.5 * self.target_yaw_dir

            # Keep turning until the front is clearly open
            if front_dist > OPEN_DIST and front_left > 0.25 and front_right > 0.25:
                # Path ahead is open! Resume driving forward
                self.state = 'FORWARD'

        twist.twist.linear.x = float(linear_x)
        twist.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_twist = TwistStamped()
        stop_twist.header.stamp = node.get_clock().now().to_msg()
        stop_twist.header.frame_id = 'base_footprint'
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
