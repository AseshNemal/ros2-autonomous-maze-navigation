"""Launch SLAM Toolbox and the autonomous maze explorer."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('slam_mapping')
    slam_pkg = get_package_share_directory('slam_toolbox')
    params_file = os.path.join(pkg, 'config', 'mapper_params_online_async.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_drive = LaunchConfiguration('auto_drive')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'auto_drive',
            default_value='true',
            description='Automatically drive robot through the maze without manual teleop',
        ),

        # Include official SLAM Toolbox launch (handles lifecycle CONFIGURE and ACTIVATE)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': params_file,
            }.items(),
        ),

        # Autonomous maze explorer
        Node(
            package='slam_mapping',
            executable='auto_explorer.py',
            name='auto_explorer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(auto_drive),
        ),
    ])
