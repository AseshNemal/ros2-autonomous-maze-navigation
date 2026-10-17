"""Launch SLAM Toolbox in localization mode on the saved maze map."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    slam_pkg = get_package_share_directory('slam_toolbox')
    pkg = get_package_share_directory('slam_mapping')
    map_file = os.path.join(pkg, 'maps', 'maze_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'localization_launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
