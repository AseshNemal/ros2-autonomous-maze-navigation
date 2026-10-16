"""DQN EVALUATION on the 3 assignment goals (greedy policy). Do not run Nav2 alongside.

  ros2 launch dqn_agent dqn_eval.launch.py checkpoint:=~/.ros/dqn_agent/checkpoints/dqn_maze.npz
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
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("checkpoint", default_value="~/.ros/dqn_agent/checkpoints/dqn_maze.npz"),
        DeclareLaunchArgument("episodes", default_value="3", description="episodes per goal"),
        DeclareLaunchArgument("max_steps", default_value="250"),
        DeclareLaunchArgument("guidance", default_value="none"),
        DeclareLaunchArgument("csv", default_value="~/.ros/dqn_agent/results/dqn_evaluation.csv"),
        DeclareLaunchArgument("start_delay", default_value="20.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, "launch", "maze_simulation.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gui")}.items()),
        TimerAction(period=LaunchConfiguration("start_delay"), actions=[ExecuteProcess(
            cmd=["ros2", "run", "dqn_agent", "evaluate_dqn",
                 "--checkpoint", LaunchConfiguration("checkpoint"),
                 "--episodes", LaunchConfiguration("episodes"),
                 "--max-steps", LaunchConfiguration("max_steps"),
                 "--guidance", LaunchConfiguration("guidance"),
                 "--csv", LaunchConfiguration("csv")],
            output="screen")]),
    ])
