
import math
import subprocess
import time

import numpy as np


# ==========================================
# 1. DQN Environment Configuration
# ==========================================

NUM_SECTORS = 12
MAX_LIDAR_RANGE = 3.5
STATE_SIZE = 15

GOAL_TOLERANCE = 0.22
COLLISION_DISTANCE = 0.15


# ==========================================
# 2. Robot Actions
# ==========================================

# Each action = (linear velocity, angular velocity)
# Units: m/s and rad/s

ACTIONS = [
    (0.14, 0.0),    # 0 - Forward
    (0.10, 0.8),    # 1 - Forward-left
    (0.10, -0.8),   # 2 - Forward-right
    (0.0, 1.0),     # 3 - Rotate left
    (0.0, -1.0),    # 4 - Rotate right
]


# ==========================================
# 3. State Calculation
# ==========================================

def get_state(ranges, x, y, yaw, goal_x, goal_y):

    lidar = np.asarray(ranges, dtype=np.float32)

    if lidar.size < NUM_SECTORS:
        raise ValueError("Not enough LiDAR readings")

    # Replace invalid LiDAR readings
    lidar = np.where(
        np.isfinite(lidar) & (lidar > 0),
        lidar,
        MAX_LIDAR_RANGE
    )

    lidar = np.clip(
        lidar,
        0,
        MAX_LIDAR_RANGE
    )

    # Reduce LiDAR readings to 12 sectors
    sectors = np.array(
        [
            np.min(section)
            for section in np.array_split(
                lidar,
                NUM_SECTORS
            )
        ],
        dtype=np.float32
    )

    # Calculate distance to destination
    distance = math.hypot(
        goal_x - x,
        goal_y - y
    )

    # Calculate target heading
    target_angle = math.atan2(
        goal_y - y,
        goal_x - x
    )

    heading = target_angle - yaw

    # Normalize heading to [-pi, pi]
    heading = math.atan2(
        math.sin(heading),
        math.cos(heading)
    )

    # 12 LiDAR values + 3 goal values
    state = np.concatenate([
        sectors / MAX_LIDAR_RANGE,
        np.array(
            [
                min(distance / 5.0, 1.0),
                math.sin(heading),
                math.cos(heading)
            ],
            dtype=np.float32
        )
    ])

    return (
        state,
        distance,
        float(np.min(lidar))
    )


# ==========================================
# 4. Reward Calculation
# ==========================================

def calculate_reward(
    previous_distance,
    current_distance,
    nearest_obstacle,
    reached_goal=False,
    collision=False
):

    if collision:
        return -40.0

    if reached_goal:
        return 80.0

    # Positive if robot approaches the goal
    progress = (
        previous_distance - current_distance
    )

    reward = 12.0 * progress - 0.05

    # Penalty for moving near obstacles
    if nearest_obstacle < 0.30:
        reward -= 0.4

    return float(reward)


# ==========================================
# 5. Gazebo Environment
# ==========================================

class GazeboEnvironment:

    def __init__(
        self,
        goal=(1.0, 0.0),
        world="dqn",
        action_duration=0.3
    ):

        import rclpy

        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data

        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TwistStamped

        if action_duration <= 0:
            raise ValueError(
                "Action duration must be positive"
            )

        self.rclpy = rclpy
        self.TwistStamped = TwistStamped

        self.node = Node("dqn_environment")

        self.goal = tuple(goal)
        self.world = world
        self.action_duration = action_duration

        self.scan = None
        self.odom = None

        self.scan_count = 0
        self.odom_count = 0

        self.previous_distance = None
        self.previous_position = None

        self.path_length = 0.0
        self.steps = 0
        self.start_time = 0.0

        # Subscribe to LiDAR
        self.node.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Subscribe to odometry
        self.node.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile_sensor_data
        )

        # Publish movement commands
        self.velocity_publisher = (
            self.node.create_publisher(
                TwistStamped,
                "/cmd_vel",
                10
            )
        )

    # --------------------------------------
    # Sensor Callbacks
    # --------------------------------------

    def scan_callback(self, message):
        self.scan = message
        self.scan_count += 1

    def odom_callback(self, message):
        self.odom = message
        self.odom_count += 1

    # --------------------------------------
    # Wait for Sensor Messages
    # --------------------------------------

    def wait_for_sensors(self, timeout=15.0):

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            self.rclpy.spin_once(
                self.node,
                timeout_sec=0.1
            )

            if (
                self.scan is not None
                and self.odom is not None
            ):
                return

        raise RuntimeError(
            "Timed out waiting for /scan and /odom"
        )

    def wait_for_fresh_sensors(
        self,
        old_scan,
        old_odom,
        timeout=15.0
    ):

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            self.rclpy.spin_once(
                self.node,
                timeout_sec=0.1
            )

            if (
                self.scan_count > old_scan
                and self.odom_count > old_odom
            ):
                return

        raise RuntimeError(
            "Fresh sensor messages were not received"
        )

    # --------------------------------------
    # Robot Position
    # --------------------------------------

    def get_robot_pose(self):

        if self.odom is None:
            raise RuntimeError(
                "Odometry has not been received"
            )

        position = self.odom.pose.pose.position
        orientation = self.odom.pose.pose.orientation

        # Quaternion to yaw
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y ** 2
                + orientation.z ** 2
            )
        )

        return (
            float(position.x),
            float(position.y),
            float(yaw)
        )

    def get_sim_time(self):

        if self.odom is None:
            return 0.0

        stamp = self.odom.header.stamp

        return (
            stamp.sec
            + stamp.nanosec * 1e-9
        )

    # --------------------------------------
    # Get Robot State
    # --------------------------------------

    def observe(self):

        self.wait_for_sensors()

        x, y, yaw = self.get_robot_pose()

        return get_state(
            self.scan.ranges,
            x,
            y,
            yaw,
            self.goal[0],
            self.goal[1]
        )

    # --------------------------------------
    # Send Movement Command
    # --------------------------------------

    def send_velocity(self, linear, angular):

        message = self.TwistStamped()

        message.header.stamp = (
            self.node.get_clock().now().to_msg()
        )

        message.header.frame_id = "base_link"

        message.twist.linear.x = float(linear)
        message.twist.angular.z = float(angular)

        self.velocity_publisher.publish(message)

    # --------------------------------------
    # Stop Robot
    # --------------------------------------

    def stop(self):

        for _ in range(3):

            self.send_velocity(0.0, 0.0)

            self.rclpy.spin_once(
                self.node,
                timeout_sec=0.05
            )

    # --------------------------------------
    # Reset Robot in Gazebo
    # --------------------------------------

    def reset(self):

        self.stop()

        self.wait_for_sensors()

        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "15000",
            "--req",
            (
                'name: "turtlebot3_burger" '
                'position: {x: 0, y: 0, z: 0.05} '
                'orientation: {w: 1}'
            )
        ]

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=25,
                check=False
            )

        except subprocess.TimeoutExpired as error:
            raise RuntimeError(
                "Gazebo reset process timed out"
            ) from error

        except FileNotFoundError as error:
            raise RuntimeError(
                "Gazebo command 'gz' was not found"
            ) from error

        if (
            result.returncode != 0
            or "data: true" not in result.stdout
        ):
            raise RuntimeError(
                "Gazebo reset failed: "
                + result.stdout
                + result.stderr
            )

        # Wait for new sensor readings
        old_scan = self.scan_count
        old_odom = self.odom_count

        self.wait_for_fresh_sensors(
            old_scan,
            old_odom,
            timeout=15.0
        )

        state, distance, _ = self.observe()

        self.previous_distance = distance

        x, y, _ = self.get_robot_pose()

        self.previous_position = (x, y)

        self.path_length = 0.0
        self.steps = 0

        self.start_time = self.get_sim_time()

        return state

    # --------------------------------------
    # Execute DQN Action
    # --------------------------------------

    def step(self, action):

        action = int(action)

        if action < 0 or action >= len(ACTIONS):
            raise ValueError("Invalid robot action")

        if self.previous_distance is None:
            raise RuntimeError(
                "Call reset() before step()"
            )

        linear, angular = ACTIONS[action]

        old_scan = self.scan_count
        old_odom = self.odom_count

        # Execute action for specified duration
        deadline = (
            time.monotonic()
            + self.action_duration
        )

        try:
            while time.monotonic() < deadline:

                self.send_velocity(
                    linear,
                    angular
                )

                self.rclpy.spin_once(
                    self.node,
                    timeout_sec=0.05
                )

        finally:
            self.stop()

        # Obtain fresh LiDAR and odometry
        self.wait_for_fresh_sensors(
            old_scan,
            old_odom,
            timeout=5.0
        )

        state, distance, obstacle = self.observe()

        x, y, _ = self.get_robot_pose()

        # Calculate travelled distance
        self.path_length += math.hypot(
            x - self.previous_position[0],
            y - self.previous_position[1]
        )

        self.previous_position = (x, y)

        # Check terminal conditions
        collision = (
            obstacle < COLLISION_DISTANCE
        )

        reached_goal = (
            distance < GOAL_TOLERANCE
            and not collision
        )

        # Calculate reward
        reward = calculate_reward(
            self.previous_distance,
            distance,
            obstacle,
            reached_goal=reached_goal,
            collision=collision
        )

        self.previous_distance = distance
        self.steps += 1

        done = collision or reached_goal

        elapsed = max(
            0.0,
            self.get_sim_time() - self.start_time
        )

        info = {
            "success": reached_goal,
            "collision": collision,
            "distance": distance,
            "distance_to_goal": distance,
            "nearest_obstacle": obstacle,
            "path_length": self.path_length,
            "elapsed_sim_s": elapsed,
            "steps": self.steps
        }

        return state, reward, done, info

    # --------------------------------------
    # Close Environment
    # --------------------------------------

    def close(self):

        self.stop()
        self.node.destroy_node()

