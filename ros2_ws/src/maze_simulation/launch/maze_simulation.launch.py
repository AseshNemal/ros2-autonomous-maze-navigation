"""Integration launch: the friend's custom_maze.launch.py (unchanged) + the /cmd_vel dead-man watchdog.

Starts exactly what `custom_maze.launch.py` starts (Gazebo with worlds/custom_maze.sdf, one TurtleBot3
Burger named `burger` spawned at (0.25, 0.0), the stock TurtleBot3 ROS-Gazebo bridge, robot_state_publisher)
and adds `cmd_vel_watchdog.py`, which stops the robot if whatever drives /cmd_vel dies.
Does NOT start SLAM, Nav2 or DQN; those have their own launch files.

  ros2 launch maze_simulation maze_simulation.launch.py            # Gazebo GUI
  ros2 launch maze_simulation maze_simulation.launch.py gui:=false # headless server (-s), for the VM / training
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    sim = get_package_share_directory("maze_simulation")
    world = os.path.join(sim, "worlds", "custom_maze.sdf")
    gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1", "yes")
    gz_args = f'-r "{world}"' if gui else f'-s -r --headless-rendering "{world}"'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, "launch", "custom_maze.launch.py")),
            launch_arguments={"gz_args": gz_args}.items()),
        # dead-man safety: stops the robot if /cmd_vel commands stop arriving (see script docstring)
        Node(package="maze_simulation", executable="cmd_vel_watchdog.py", output="screen",
             parameters=[{"timeout": 0.5}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="false = headless server only (-s), for the VM / training"),
        OpaqueFunction(function=launch_setup),
    ])
