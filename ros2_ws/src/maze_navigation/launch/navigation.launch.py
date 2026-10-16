from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml
from launch.actions import OpaqueFunction

def validate_inputs(context):
    # Check the map YAML and navigation configuration before starting nodes.
    paths = {}

    for argument in ("map", "params_file"):
        value = LaunchConfiguration(argument).perform(context)
        path = Path(value).expanduser()

        if not path.is_absolute():
            raise RuntimeError(
                f"{argument}: provide an absolute file path, got: {value}"
            )

        if not path.is_file():
            raise RuntimeError(f"{argument}: file not found: {path}")

        try:
            with path.open() as stream:
                content = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as error:
            raise RuntimeError(
                f"{argument}: cannot read YAML: {path}: {error}"
            ) from error

        if not isinstance(content, dict):
            raise RuntimeError(
                f"{argument}: expected a YAML mapping: {path}"
            )

        paths[argument] = (path, content)

    map_path, map_config = paths["map"]
    image = map_config.get("image")

    if not isinstance(image, str) or not image.strip():
        raise RuntimeError(f"Map YAML has no valid image entry: {map_path}")

    image_path = Path(image)
    if not image_path.is_absolute():
        image_path = map_path.parent / image_path

    if not image_path.is_file():
        raise RuntimeError(f"Map image not found: {image_path}")

    return []

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
            default_value=str(
                Path(get_package_share_directory("slam_mapping"))
                / "maps" / "maze_groundtruth.yaml"
            ),
            description="Absolute path to the saved map YAML file "
            "(default: ground-truth maze map; pass your SLAM map for the real pipeline)",
        ),
        DeclareLaunchArgument("initial_x", default_value="-1.25",
                              description="Spawn x in the MAP frame (SLAM map: 0.0)"),
        DeclareLaunchArgument("initial_y", default_value="-1.25",
                              description="Spawn y in the MAP frame (SLAM map: 0.0)"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
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
        OpaqueFunction(function=validate_inputs),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav2_share / "launch" / "bringup_launch.py")
            ),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # Nav2 evaluates these with PythonExpression: they must be
                # capitalised Python literals ("false" -> NameError, no Nav2).
                "slam": "False",
                "use_localization": "True",
                "autostart": "True",
                "use_composition": "False",
            }.items(),
        ),
        # Integration additions: Nav2 (Jazzy) emits Twist on cmd_vel_nav2; the
        # simulation expects TwistStamped on /cmd_vel.
        Node(
            package="maze_navigation",
            executable="twist_stamper.py",
            name="twist_stamper",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="slam_mapping",
            executable="set_initial_pose.py",
            output="screen",
            arguments=[
                "--x", LaunchConfiguration("initial_x"),
                "--y", LaunchConfiguration("initial_y"),
                "--yaw", LaunchConfiguration("initial_yaw"),
            ],
            parameters=[{"use_sim_time": True}],
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
