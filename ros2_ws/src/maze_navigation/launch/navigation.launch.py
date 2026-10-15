from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_share = Path(
        get_package_share_directory("maze_navigation")
    )
    nav2_share = Path(
        get_package_share_directory("nav2_bringup")
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Open RViz with the navigation layout",
        ),
        DeclareLaunchArgument(
            "map",
            description="Absolute path to the saved map YAML file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Gazebo simulation clock",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=str(
                package_share / "config" / "nav2_params.yaml"
            ),
            description="Navigation configuration file",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav2_share / "launch" / "bringup_launch.py")
            ),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "slam": "false",
                "use_localization": "true",
                "autostart": "true",
                "use_composition": "false",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                str(package_share / "rviz" / "navigation.rviz"),
            ],
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
            }],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
