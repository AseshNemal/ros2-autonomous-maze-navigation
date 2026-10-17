"""Gazebo TurtleBot3 Burger interface; never drives a physical robot.

Uses /scan and /odom feedback, /cmd_vel TwistStamped (Jazzy simulation bridge),
and Gazebo's /world/<world>/set_pose service to reset only the simulated robot.
"""
import math
import re
import subprocess
import time
import numpy as np

ACTIONS = ((0.14, 0.0), (0.10, 0.8), (0.10, -0.8), (0.0, 1.0), (0.0, -1.0))
SECTORS = 12
STATE_DIM = SECTORS + 3


def valid_ranges(ranges, max_range=3.5):
    v = np.asarray(ranges, dtype=np.float32)
    if v.size == 0:
        raise ValueError("Empty LiDAR scan")
    return np.where(np.isfinite(v) & (v > 0), np.clip(v, 0, max_range), max_range)


def get_observation(ranges, position, yaw, goal, max_range=3.5):
    scan = valid_ranges(ranges, max_range)
    sectors = np.asarray([float(np.min(s)) for s in np.array_split(scan, SECTORS)], dtype=np.float32)
    distance = math.hypot(goal[0] - position[0], goal[1] - position[1])
    desired = math.atan2(goal[1] - position[1], goal[0] - position[0])
    error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
    state = np.concatenate((sectors / max_range,
                            [min(distance / 5, 1.0), math.sin(error), math.cos(error)])).astype(np.float32)
    return state, distance, float(np.min(scan))


def compute_reward(before, after, clearance, collision=False, reached=False):
    """Dense progress reward; termination rewards override normal step costs."""
    if collision:
        return -40.0
    if reached:
        return 80.0
    return (12.0 * (before - after) - 0.05
            - (0.4 if clearance < 0.30 else 0.0))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def odom_to_world(xy, yaw, odom_ref, start):
    """Map an odom-frame pose to the world frame.

    gz DiffDrive integrates wheel motion, so /odom is NOT reset when the robot is
    teleported with set_pose. At each reset we record the odom pose (odom_ref) that
    corresponds to the known world start pose (x, y, yaw) and map relative motion
    from there.
    """
    ox, oy, oyaw = odom_ref
    sx, sy, syaw = start
    d = syaw - oyaw
    dx, dy = xy[0] - ox, xy[1] - oy
    return ((sx + math.cos(d) * dx - math.sin(d) * dy,
             sy + math.sin(d) * dx + math.cos(d) * dy),
            wrap_angle(yaw + d))


def pose_xyyaw(odom):
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return (float(p.x), float(p.y)), yaw


class GazeboDQNEnv:
    def __init__(self, world="custom_maze", goal=(-0.25, 0.75),
                 start=(0.25, 0.0, 0.0), action_duration=0.30,
                 goal_tolerance=0.22, collision_distance=0.15,
                 waypoint_tolerance=0.25, waypoint_bonus=5.0):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TwistStamped

        self.rclpy = rclpy
        self.TwistStamped = TwistStamped
        self.node = Node("dqn_gazebo_env")
        self.scan = None
        self.odom = None
        self.scan_count = 0
        self.odom_count = 0
        self.node.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.world, self.goal, self.start = world, tuple(goal), tuple(start)
        self.waypoints = []          # optional guidance: intermediate targets (world xy)
        self.wp_index = 0
        self.waypoint_tolerance, self.waypoint_bonus = waypoint_tolerance, waypoint_bonus
        self.odom_ref = None
        self.action_duration = action_duration
        self.goal_tolerance, self.collision_distance = goal_tolerance, collision_distance
        self.previous_distance = None
        self.path_length = 0.0
        self.previous_xy = None
        self.sim_start = None
        self.steps = 0

    def _on_scan(self, msg):
        self.scan = msg
        self.scan_count += 1

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_count += 1

    def _wait(self, predicate, timeout=60.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.10)
            if predicate():
                return
        raise RuntimeError("Timed out waiting for live /scan and /odom data. Check Gazebo, topic bridges and simulator pause state.")

    def _state(self):
        if self.scan is None or self.odom is None:
            raise RuntimeError("Sensor topics not ready")
        xy, yaw = self.world_pose()
        return get_observation(self.scan.ranges, xy, yaw, self.target())

    def world_pose(self):
        xy, yaw = pose_xyyaw(self.odom)
        return odom_to_world(xy, yaw, self.odom_ref, self.start)

    def target(self):
        """Current target: next waypoint when guidance is on, else the final goal."""
        if self.wp_index < len(self.waypoints):
            return self.waypoints[self.wp_index]
        return self.goal

    def publish(self, linear=0.0, angular=0.0):
        msg = self.TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self):
        for _ in range(3):
            self.publish()
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def reset(self, start=None, goal=None, waypoints=None):
        """Teleport the robot to `start` (x, y, yaw world) and begin an episode.

        goal / waypoints override the constructor values for this and later episodes.
        """
        self.stop()
        if start is not None:
            self.start = tuple(start)
        if goal is not None:
            self.goal = tuple(goal)
        if waypoints is not None:
            self.waypoints = [tuple(w) for w in waypoints]
        self.wp_index = 0
        x, y, yaw = self.start
        # No Gazebo-wide reset: other simulations and nodes remain untouched.
        req = ('name: "turtlebot3_burger" '
               f'position: {{x: {x}, y: {y}, z: 0.05}} '
               f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        try:
            result = subprocess.run(
                ["gz", "service", "-s", f"/world/{self.world}/set_pose",
                 "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "5000", "--req", req],
                capture_output=True, text=True, timeout=12, check=False)
        except (FileNotFoundError, subprocess.TimeoutExpired) as error:
            raise RuntimeError("Gazebo pose-reset service unavailable") from error
        if result.returncode or not re.search(r"data:\s*true", result.stdout):
            raise RuntimeError("Gazebo robot reset failed: " + (result.stdout + result.stderr)[-600:])
        self.stop()
        s0, o0 = self.scan_count, self.odom_count
        self._wait(lambda: self.scan_count > s0 + 1 and self.odom_count > o0 + 1)
        self.odom_ref = (*pose_xyyaw(self.odom)[0], pose_xyyaw(self.odom)[1])
        state, distance, _ = self._state()
        self.previous_distance = distance
        self.path_length = 0.0
        self.previous_xy = self.world_pose()[0]
        self.sim_start = self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9
        self.steps = 0
        return state

    def step(self, action):
        if not 0 <= int(action) < len(ACTIONS):
            raise ValueError("Invalid discrete action")
        if self.previous_distance is None:
            raise RuntimeError("Call reset before step")
        linear, angular = ACTIONS[int(action)]
        before = self.previous_distance
        s0, o0 = self.scan_count, self.odom_count
        stop_at = time.monotonic() + self.action_duration
        while time.monotonic() < stop_at:
            self.publish(linear, angular)
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        self.stop()
        self._wait(lambda: self.scan_count > s0 and self.odom_count > o0, timeout=4.0)
        xy = self.world_pose()[0]
        waypoint_hit = False
        while (self.wp_index < len(self.waypoints) and
               math.hypot(self.waypoints[self.wp_index][0] - xy[0],
                          self.waypoints[self.wp_index][1] - xy[1]) <= self.waypoint_tolerance):
            self.wp_index += 1
            waypoint_hit = True
        state, distance, clearance = self._state()   # distance is to the current target
        self.path_length += math.hypot(xy[0] - self.previous_xy[0], xy[1] - self.previous_xy[1])
        self.previous_xy = xy
        self.previous_distance = distance
        self.steps += 1
        collision = clearance < self.collision_distance
        final_distance = math.hypot(self.goal[0] - xy[0], self.goal[1] - xy[1])
        success = final_distance <= self.goal_tolerance
        terminated = collision or success
        if waypoint_hit:   # target switched: progress term would compare different targets
            before = distance
        reward = compute_reward(before, distance, clearance, collision, success)
        if waypoint_hit and not (collision or success):
            reward += self.waypoint_bonus
        elapsed = (self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9
                   - self.sim_start)
        return state, reward, terminated, {
            "success": success, "collision": collision, "distance_to_goal": final_distance,
            "waypoint_index": self.wp_index,
            "min_scan": clearance, "path_length_m": self.path_length,
            "elapsed_sim_s": max(0.0, elapsed), "steps": self.steps,
        }

    def close(self):
        self.stop()
        self.node.destroy_node()
