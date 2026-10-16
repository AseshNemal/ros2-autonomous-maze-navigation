"""Launch the assignment maze in Gazebo Harmonic with a TurtleBot3 Burger.

Starts: gz sim (maze.world), robot_state_publisher, robot spawn, ros_gz_bridge.
Does NOT start SLAM, Nav2 or DQN; those have their own launch files.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_NAME = "turtlebot3_burger"  # Gazebo entity name; reset services use it


def generate_launch_description():
    sim = get_package_share_directory("maze_simulation")
    tb3 = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world = os.path.join(sim, "worlds", "maze.world")
    model_sdf = os.path.join(tb3, "models", "turtlebot3_burger", "model.sdf")
    with open(os.path.join(tb3, "urdf", "turtlebot3_burger.urdf")) as f:
        robot_description = f.read()

    args = [
        DeclareLaunchArgument("gui", default_value="true",
                              description="false = headless server only (-s), for the VM / training"),
        DeclareLaunchArgument("x_pose", default_value="-1.25",
                              description="spawn x (default: start cell (5,0))"),
        DeclareLaunchArgument("y_pose", default_value="-1.25"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_args_gui = ["-r -v2 ", world]
    gz_args_headless = ["-r -s --headless-rendering -v2 ", world]

    return LaunchDescription(args + [
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(tb3, "models")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args_gui}.items(),
            condition=IfCondition(LaunchConfiguration("gui"))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args_headless}.items(),
            condition=UnlessCondition(LaunchConfiguration("gui"))),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen",
             parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", "maze", "-name", ROBOT_NAME, "-file", model_sdf,
                        "-x", LaunchConfiguration("x_pose"), "-y", LaunchConfiguration("y_pose"),
                        "-z", "0.01", "-Y", LaunchConfiguration("yaw")]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(sim, "config", "bridge.yaml"),
                          "use_sim_time": use_sim_time}]),
    ])
