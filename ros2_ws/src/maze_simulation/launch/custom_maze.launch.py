"""Launch TurtleBot3 Burger in the custom maze simulation."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('maze_simulation')
    gz_pkg = get_package_share_directory('ros_gz_sim')
    tb_pkg = get_package_share_directory('turtlebot3_gazebo')
    world = os.path.join(pkg, 'worlds', 'custom_maze.sdf')

    bridge_config = os.path.join(pkg, 'params', 'turtlebot3_bridge.yaml')
    if not os.path.exists(bridge_config):
        bridge_config = os.path.join(tb_pkg, 'params', 'turtlebot3_burger_bridge.yaml')

    with open(os.path.join(tb_pkg, 'urdf', 'turtlebot3_burger.urdf')) as source:
        robot_description = source.read()

    resources = os.pathsep.join(filter(None, [
        os.path.join(tb_pkg, 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))

    headless = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Whether to run Gazebo in headless mode (server only)'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),

        # 1. Gazebo Server: runs physics and services headlessly
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r -s -v2 "{world}"',
                              'on_exit_shutdown': 'true'}.items(),
        ),

        # 2. Gazebo GUI Client: decoupled from server so rendering doesn't block simulation startup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2'}.items(),
            condition=UnlessCondition(headless),
        ),

        # 3. Spawner: creates TurtleBot3 Burger at corridor entrance (0.25, 0.0, 0.01) facing +X
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'custom_maze', '-name', 'burger',
                        '-allow_renaming', 'false', '-file',
                        os.path.join(tb_pkg, 'models', 'turtlebot3_burger', 'model.sdf'),
                        '-x', '0.25', '-y', '0.0', '-z', '0.01', '-Y', '0.0']),

        # 4. ROS-Gazebo parameter bridge
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=[
                 '--ros-args',
                 '-p',
                 f'config_file:={bridge_config}',
             ]),

        # 5. Robot state publisher
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_description,
                                           'use_sim_time': True}]),
    ])
