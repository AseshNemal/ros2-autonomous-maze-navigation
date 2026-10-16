# Member 4 — Deep Q-Learning (DQN)

ROS 2 Jazzy, TurtleBot3 Burger and Gazebo Harmonic 8. This package trains a **Double DQN** navigation policy through simulated sensor/action interactions. It is not a pretrained navigator.

## DQN design

- **State (15 dimensions):** 12 minimum-range LiDAR sectors normalized to 3.5 m, goal distance normalized to 5 m, sine and cosine of relative goal heading.
- **Actions:** 5 discrete velocity commands: forward, forward-left, forward-right, rotate-left, rotate-right.
- **Reward:** +12 times reduction in distance to goal, -0.05 per action, -0.4 when near an obstacle, -40 on scan-based collision detection, +80 on goal arrival.
- **Network:** 15 → 64 ReLU → 64 ReLU → 5 Q-values. Adam optimizer (NumPy implementation), Huber loss, uniform replay, epsilon-greedy, target network, Double DQN bootstrap.
- **CPU compatible:** NumPy only for neural-network training; PyTorch is not required on the 4 GB ARM64 VM.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select dqn_agent
source install/setup.bash
colcon test --packages-select dqn_agent
colcon test-result --verbose
```

## Training — requires a running Gazebo DQN world

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
```

Terminal 2 (source workspace too):

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run dqn_agent train_dqn --episodes 100 --max-steps 100 --goal-x 1.0 --goal-y 0.0
```

If `ros2 run` is missing, install `ros-jazzy-ros2run` (optional) or execute `ros2_ws/install/dqn_agent/lib/dqn_agent/train_dqn` directly. Use `--episodes 2 --max-steps 10` for a smoke test only, **not** performance evidence. First ensure the goal is reachable in the selected world and Gazebo services are live.

The Gazebo Stage 1 world is named `dqn` internally (not the world filename). The trainer resets only `turtlebot3_burger` through Gazebo's `/world/dqn/set_pose` service. `/cmd_vel` uses `geometry_msgs/msg/TwistStamped`, matching the ROS Jazzy TurtleBot3 simulator.

## Evaluate

```bash
ros2 run dqn_agent evaluate_dqn --checkpoint ~/.ros/dqn_agent/checkpoints/dqn_stage1.npz --episodes 3 --goal-x 1.0 --goal-y 0.0
```

Training CSV, evaluation CSV, and checkpoints are saved under `~/.ros/dqn_agent/` by default. Do not commit trained weights, experiment artifacts or fabricated metrics as source code. Checkpoints use `.npz` and need to remain outside the repository.

## Fair comparison with Nav2

Member 3 provides a Nav2 goal evaluator. After both methods run on the **same maze**, with the **same start and goal pairs, reset protocol, timeouts, and collision rules**:

```bash
ros2 run dqn_agent compare_navigation --dqn ~/.ros/dqn_agent/results/dqn_evaluation.csv --nav2 /path/to/real/nav2_results.csv
```

The comparison tool reads actual CSVs and rejects missing experiment data. It warns if goal sets differ. Nav2's evaluator reports odometry distance, not global plan distance, and lacks full environment metadata, so verify matching world/start/tolerances manually.

## Important limitations

- Scan-based collision detection is an approximation, not Gazebo contact sensor ground truth.
- Simulation elapsed time comes from odometry timestamps; sensor data may be delayed in low-memory VMs.
- One short training run does not establish convergence, success or generalization.
- One training goal does not demonstrate three distinct goals; train/test against fixed multiple goals after Member 1's maze and Member 2's map are ready.
- The ROS simulator must be running and publishing /scan and /odom throughout every training/evaluation run.
- Do not activate Nav2 velocity control and the DQN simultaneously.

See `config/dqn_params.yaml` for reference hyperparameters.
