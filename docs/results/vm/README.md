# Results on the CURRENT maze (`custom_maze.sdf`), VM, 2026-10-17

Raw logs/CSV/JSON written by `tools/verify_on_vm.sh` and the command-line tests; do not edit by hand. Summary: `docs/testing.md`.

Notes on individual files:
- `sim_20261017-101830.log` is an earlier 7-check run; `sim_20261017-101901.log` (10 checks) supersedes it.
- `control_20261017-101926.*` is run 1 (21/22); `control_20261017-102106.*` is the rerun (22/22).
- `slam_20261017-102242.log` = run 1 (odometry-steered tour, precision 0.256); `slam_20261017-111320.log` = run 2 (SLAM-pose-steered tour, precision 0.819; the committed `maps/maze_slam.*` are from this run).
- `slam_20261017-112457.log` is an **aborted run** (Gazebo started but the tour never began; terminated by hand, FastDDS shared-memory locks were stale). It contains no result.
- `dqn_20261017-115922.log` / `dqn_train_none_*.csv` are a 3-episode **smoke test**, not training results.
- `cmdline_drive_and_goal_attempt1_*.txt`: `drive.sh` worked; its `send_goal.sh` part failed only because the `ros2 action` CLI is not installed (fixed by `tools/send_goal.py`). `cmdline_send_goal_*.txt` is the passing retest.
- `tour_odom_vs_truth_*`: odometry-based tour pose compared with Gazebo ground truth (before and after the tour change).
Results from the removed 6 x 6 maze: `../old_6x6_maze/` (historical).
- `nav2_consecutive_goals_no_reset_*.txt`: 4 goals in a row without reset (clean run). `reset_robot_wrong_pose_estimate_*.txt`: deliberately wrong pose estimate, then `tools/reset_robot.sh`, then a goal (final passing run; two earlier attempts failed because of the `set_initial_pose.py` defects fixed afterwards and are not saved).

## Final-session runs (2026-10-17 afternoon)
- `nav_20261017-165127.log` / `nav2_results_20261017-165127.csv`: scripted `verify_on_vm.sh nav` on the corrected ground-truth map, 3/3 goals, 0 recoveries (headless).
- `nav2_window_mode_corrected_map_20261017.txt`: manual window-mode (Gazebo GUI + RViz) goals, including two aborted runs while a stale Nav2 lifecycle manager was running.
- `dqn_20261017-165336.log`, `dqn_train_none_20261017-165336.csv`, `dqn_eval_none_20261017-165336.csv`: 10-episode DQN smoke test (329 transitions, 230 optimizer updates).
- `dqn_20261017-165910.log`, `dqn_train_none_20261017-165910.csv`: 60-episode pure-DQN training (1587 transitions, 1488 optimizer updates).
- **`dqn_eval_none_20261017-165910.csv` (5 of 9 "successes") is NOT valid evidence of goal reaching.** Success there was decided from wheel odometry, which drifts and keeps counting while the wheels push against a wall (`dqn_odometry_vs_truth_20261017.txt`). The same checkpoint re-evaluated with success confirmed by Gazebo's true pose: `dqn_eval_none_truthconfirmed_20261017-172400.csv` / `dqn_eval_truthconfirmed_20261017-172400.log` = **0 of 9**. The training successes in `dqn_train_none_20261017-165910.csv` (13) are odometry-based and were not re-verified.
