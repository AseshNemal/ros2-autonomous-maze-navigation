import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('maze_simulation')
    gz_pkg = get_package_share_directory('ros_gz_sim')
    tb_pkg = get_package_share_directory('turtlebot3_gazebo')
    world = os.path.join(pkg, 'worlds', 'custom_maze.sdf')
    # Reuse TurtleBot3's tested spawn launch, which also starts the bridge,
    # state publisher, and robot model for the Jazzy simulation packages.
    tb_spawn = os.path.join(tb_pkg, 'launch', 'spawn_turtlebot3.launch.py')
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r ' + world}.items(),
        ),
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(tb_spawn))
        ]),
    ])
