# Member 4 — DQN Stage 1 Trial (October 16, 2026)

These CSVs were generated from **actual TurtleBot3 Burger Gazebo Harmonic / ROS 2 Jazzy runs** on an Ubuntu ARM64 VM. They are not simulated or synthetic CSVs. They are **trial/smoke-training data, not final assignment navigation benchmarks**.

## Environment and method
- Gazebo: packaged `turtlebot3_dqn_stage1.launch.py` (internal world name `dqn`)
- Robot: TurtleBot3 Burger
- Policy: NumPy Double DQN, 15 inputs, 64+64 hidden neurons, 5 discrete actions, seed 42
- Training: 10 episodes × 20 action-step limit, 0.15 s action duration, goal (1.0, 0.0)
- Replay warmup: 100 steps; target update interval: 200 gradient steps
- Checkpoint: saved locally to `~/.ros/dqn_agent/checkpoints/dqn_stage1_trial.npz` and intentionally not committed

## Observed training results
- 10/10 episodes executed, 200 total environment actions
- 101 optimizer updates
- 0 training successes, 0 scan-based collision events
- Training steps and per-episode rewards are preserved in `member4_dqn_training_trial.csv`

## Observed deterministic evaluation
- Same DQN stage world, same goal (1.0, 0.0), 3 evaluation episodes with 20-step limit
- 0/3 successes and 0/3 scan-based collision events
- All three runs timed out, after simulated 4.04, 4.22, and 4.26 seconds
- Measured odometry path lengths: 0.1978 m, 0.1921 m, 0.2048 m
- Data are preserved in `member4_dqn_evaluation_trial.csv`

## Interpretation and remaining work
- The ROS sensor/action loop, Gazebo robot reset, DQN parameter updates, checkpoint loading, and real CSV logging **work**.
- Only 200 actions and 101 optimizer updates are far too few for credible learning/convergence evidence.
- There is **no demonstrated DQN maze completion**, no three-goal evaluation, and no valid Nav2-vs-DQN comparison yet.
- Scan-based collision detection is not a Gazebo contact sensor measurement.
- These results are for ROBOTIS DQN Stage 1, **not the team's final custom maze**; do not present them as final-maze evidence.
- Required next: longer training with an approved goal/curriculum, fixed evaluation seeds/goals, integration with the final maze, and a like-for-like Nav2 comparison.
