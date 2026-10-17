"""Launch one genuine TurtleBot3 Burger in the closed Harmonic maze."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('maze_simulation')
    gz_pkg = get_package_share_directory('ros_gz_sim')
    tb_pkg = get_package_share_directory('turtlebot3_gazebo')
    world = os.path.join(pkg, 'worlds', 'custom_maze.sdf')
    with open(os.path.join(tb_pkg, 'urdf', 'turtlebot3_burger.urdf')) as source:
        robot_description = source.read()
    resources = os.pathsep.join(filter(None, [
        os.path.join(tb_pkg, 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))
    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value=f'-r "{world}"',
                              description='Gazebo arguments; use -s -r --headless-rendering for headless'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args'),
                              'on_exit_shutdown': 'true'}.items(),
        ),
        # create waits for /world/custom_maze/create; no fixed startup sleep.
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'custom_maze', '-name', 'burger',
                        '-allow_renaming', 'false', '-file',
                        os.path.join(tb_pkg, 'models', 'turtlebot3_burger', 'model.sdf'),
                        '-x', '0.25', '-y', '0.0', '-z', '0.01', '-Y', '0.0']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(
                 tb_pkg, 'params', 'turtlebot3_burger_bridge.yaml'), 'use_sim_time': True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_description,
                                           'use_sim_time': True}]),
    ])
