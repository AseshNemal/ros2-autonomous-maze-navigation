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
