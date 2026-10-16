"""SLAM Toolbox online-async mapping. Start the simulation first.

  ros2 launch maze_simulation maze_simulation.launch.py
  ros2 launch slam_mapping mapping.launch.py
Drive with teleop (stamped) or `ros2 run slam_mapping auto_map_drive.py`,
then `ros2 run slam_mapping save_map.sh <name>`.
Do not run together with Nav2 or DQN controllers (they share /cmd_vel).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("slam_mapping")
    slam_share = get_package_share_directory("slam_toolbox")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("slam_params_file", default_value=os.path.join(
            share, "config", "slam_toolbox_mapping.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "slam_params_file": LaunchConfiguration("slam_params_file"),
            }.items()),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "mapping.rviz")],
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ])
