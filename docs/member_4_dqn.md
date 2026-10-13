# Member 4 — Deep Q-Learning & Performance Analysis

**Package:** `ros2_ws/src/dqn_agent` (`ament_python`)
**Status:** scaffold only. No environment, network, training or evaluation code exists. No model has been trained and no results exist.

## Tasks
- Build the DQN navigation component (ROS 2 node wrapping the Gazebo simulation as an RL environment).
- Define and justify the state space (e.g. downsampled LiDAR, goal distance/heading), action space (discrete velocity commands) and reward function.
- Implement training and evaluation scripts.
- Keep checkpoints out of Git (`*.pt`, `*.pth`, `*.ckpt`, `checkpoints/` are ignored).
- Evaluate collision avoidance, travel time and path length.
- Compare DQN with the Nav2 baseline from Member 3 using the same start/goal pairs.

## Expected files
| Path (inside `dqn_agent/`) | Purpose |
|---|---|
| `dqn_agent/env.py` | Gym-style environment over `/scan`, `/odom`, `/cmd_vel` and a reset mechanism |
| `dqn_agent/agent.py`, `network.py`, `replay_buffer.py` | DQN components |
| `dqn_agent/train.py`, `evaluate.py` | Entry points; register in `setup.py` `console_scripts` |
| `config/dqn_params.yaml` | Hyper-parameters and reward weights |
| `models/` | Local checkpoints (git-ignored) |
| `test/` | Unit tests that do not need Gazebo |

## Dependencies
ROS: `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`. Python: PyTorch and NumPy (and plotting libraries) to be added to the root `requirements.txt` once chosen and tested on the target platform. Needs Member 1's simulation; the baseline needs Member 3.

## Deliverables
- Documented state/action/reward design.
- Training log and learning curve from real runs.
- Evaluation table (success, collisions, time, path length) for DQN and Nav2 under identical conditions, stored in `docs/results/`.
- Where trained checkpoints are shared (external link) and which run produced them.

## Validation
1. `colcon build --packages-select dqn_agent` succeeds and `colcon test --packages-select dqn_agent` passes.
2. Training runs end to end and logs episode reward.
3. Evaluation runs a fixed set of start/goal pairs for both DQN and Nav2 and writes results to `docs/results/`.

## Remaining
Everything above.
