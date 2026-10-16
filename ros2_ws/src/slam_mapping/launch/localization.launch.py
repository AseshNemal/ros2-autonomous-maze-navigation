"""Map server + AMCL localization on a saved map (no Nav2 planners).

  ros2 launch slam_mapping localization.launch.py map:=/abs/path/map.yaml \
      initial_x:=-1.25 initial_y:=-1.25
Use initial_x/y = spawn pose in the *map* frame: the ground-truth map shares the
world frame (-1.25, -1.25); a SLAM-built map starts at its own origin (0, 0).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2 = get_package_share_directory("nav2_bringup")
    nav_params = os.path.join(get_package_share_directory("maze_navigation"),
                              "config", "nav2_params.yaml")
    default_map = os.path.join(get_package_share_directory("slam_mapping"),
                               "maps", "maze_groundtruth.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=nav_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("initial_x", default_value="-1.25"),
        DeclareLaunchArgument("initial_y", default_value="-1.25"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "localization_launch.py")),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": "true", "use_composition": "False",
            }.items()),
        Node(package="slam_mapping", executable="set_initial_pose.py", output="screen",
             arguments=["--x", LaunchConfiguration("initial_x"),
                        "--y", LaunchConfiguration("initial_y"),
                        "--yaw", LaunchConfiguration("initial_yaw")],
             parameters=[{"use_sim_time": True}]),
    ])
