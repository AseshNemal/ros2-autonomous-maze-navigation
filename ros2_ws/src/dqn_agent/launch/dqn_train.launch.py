"""DQN TRAINING: headless maze simulation + train_dqn. Do not run Nav2 alongside.

  ros2 launch dqn_agent dqn_train.launch.py episodes:=300 curriculum_max_cells:=3
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = get_package_share_directory("maze_simulation")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("episodes", default_value="300"),
        DeclareLaunchArgument("max_steps", default_value="150"),
        DeclareLaunchArgument("curriculum_max_cells", default_value="3"),
        DeclareLaunchArgument("guidance", default_value="none", description="none | waypoints (hybrid)"),
        DeclareLaunchArgument("checkpoint", default_value="~/.ros/dqn_agent/checkpoints/dqn_maze.npz"),
        DeclareLaunchArgument("csv", default_value="~/.ros/dqn_agent/results/training.csv"),
        DeclareLaunchArgument("start_delay", default_value="20.0", description="s to wait for Gazebo"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, "launch", "maze_simulation.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gui")}.items()),
        TimerAction(period=LaunchConfiguration("start_delay"), actions=[ExecuteProcess(
            cmd=["ros2", "run", "dqn_agent", "train_dqn",
                 "--episodes", LaunchConfiguration("episodes"),
                 "--max-steps", LaunchConfiguration("max_steps"),
                 "--curriculum-max-cells", LaunchConfiguration("curriculum_max_cells"),
                 "--guidance", LaunchConfiguration("guidance"),
                 "--checkpoint", LaunchConfiguration("checkpoint"),
                 "--csv", LaunchConfiguration("csv")],
            output="screen")]),
    ])
