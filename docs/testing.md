# Testing

## Latest verified status - CURRENT maze (`custom_maze.sdf`, 7 x 6, spawn (0.25, 0.0)); VM, 2026-10-17; evidence in `docs/results/vm/`
| Stage | Result | Evidence |
|---|---|---|
| static (`tools/verify_static.py`, no ROS) | PASS (teammate files byte-identical to origin/main; layout/goals/spawn/world-name/entity checks; 22 pytest). **WARN: maze is not Figure 1 (7 x 6 vs 6 x 6)** | run on this branch |
| build + unit tests | PASS, 23 tests | `build_20261017-101817.log` |
| sim (headless): world `custom_maze`, exactly one robot `burger`, `/scan` `/odom` `/clock` `/tf` `/cmd_vel` | PASS, 10/10 checks (/scan 5.0 Hz, /odom 49.6 Hz) | `sim_20261017-101901.log`, `sim_topics_20261017-101901.json` |
| control: teleop forward/back/left/right/stop, watchdog stop after teleop killed, Nav2 goal, only one `/cmd_vel` publisher | run 1: 21/22 (`/scan` count 2 in 3 s right after start-up); **rerun with no code change: 22/22** | `control_20261017-101926.log`, `control_20261017-102106.log` |
| Nav2 3 goals (ground-truth map, reset to spawn before each goal) | **PASS 3/3**, 1 recovery in total (table in report 4.2) | `nav_20261017-113157.log`, `nav2_results_20261017-113157.csv` |
| command-line goal (`tools/send_goal.sh G3`, `... 1.25 0.75`) | PASS: both reached (10.3 s, 26.1 s). First attempt only failed because Nav2 bring-up aborted once (the retry in `demo.sh` handles it) | `cmdline_send_goal_20261017.txt` |
| command-line driving (`tools/drive.sh`) | PASS: forward moved 0.35 m, left turned in place, back reversed, stayed at rest afterwards | `cmdline_drive_and_goal_attempt1_20261017.txt` |
| Nav2, 4 goals in a row WITHOUT reset (G1, G2, G3, G1; the way a user clicks goals) | PASS 4/4, 0 recoveries; AMCL within 0.1 m of Gazebo truth after every goal. (An earlier attempt aborted every goal because a stale copy of my own test script was also sending goals; that attempt is not evidence either way.) | `nav2_consecutive_goals_no_reset_20261017.txt` |
| "RViz and Gazebo show the robot in different places" (user report) | **Reproduced**: a wrong "2D Pose Estimate" gives AMCL (2.45, 1.99) vs truth (0.12, -0.10) and the next goal ABORTS with 15-17 recoveries. **Fixed/verified**: `tools/reset_robot.sh` -> both (0.24, 0.00)/(0.22, 0.02) and the next goal SUCCEEDED in 10.3 s, 0 recoveries. Two defects found in `set_initial_pose.py` (clock-based timeout expired under sim time; stale latched `/amcl_pose` counted as confirmation) | `reset_robot_wrong_pose_estimate_20261017.txt` |
| Nav2 bring-up timeouts (`Failed to change state for node: docking_server`, nodes "unconfigured") | Seen when stale FastDDS shared-memory files (180 in `/dev/shm`) were left by killed processes. Scripts now use `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` and clear stale files; 2 clean bring-ups afterwards | scratch logs only |
| SLAM mapping (coverage tour + slam_toolbox + `validate_map.py`) | **FAIL vs our own thresholds.** Run 1 (odometry-steered tour): coverage 0.890, precision 0.256, free-OK 0.905. Run 2 (tour steered by the SLAM pose with side-wall centring): coverage 0.971, precision **0.819**, free-OK 1.000, unknown 0.083. Thresholds were 0.85 / 0.85 / 0.95; the assignment itself sets no numeric threshold | `slam_20261017-102242.log`, `slam_20261017-111320.log`, `slam_validation_*.txt`, `slam_map_20261017-111320.png` |
| DQN | **Smoke test only (3 episodes, no learning claimed):** training loop, `burger` reset, CSV and checkpoint work on the new maze; 3/3 episodes ended in collision; `evaluate_dqn` correctly refused the 0-update checkpoint. **No training or evaluation result exists for the current maze.** | `dqn_20261017-115922.log`, `dqn_train_none_20261017-115922.csv` |
| Nav2 on the SLAM-built map | NOT TESTED | |
| remote-desktop input delay / disconnects | NOT VERIFIABLE by script | |

Tour diagnosis (SLAM): with the original odometry-steered tour the robot's true position was within 0.115 m of a wall (touching) in 189 of 371 samples, so wheel odometry diverged from Gazebo truth (`tour_odom_vs_truth_before_fix_20261017.txt`). Steering by the SLAM pose and centring between side walls changed the result from precision 0.256 to 0.819. A smoke run of the changed tour (14 waypoints, odometry only) still showed 74 of 333 samples within 0.115 m (`..._after_fix_smoke_...txt`), so wall contact is reduced but not eliminated.

## Previous 6 x 6 maze (REMOVED) - historical results, VM 2026-10-16/17; files in `docs/results/old_6x6_maze/`
| Stage | Result on the old maze (not valid for the current maze) |
|---|---|
| build + tests | 22/22 |
| sim | 7/7 |
| Nav2 3 goals | 3/3 goals, 0 recoveries (`nav2_results_20261016-214253.csv`) |
| control | 22/22 after adding `cmd_vel_watchdog` (first run 21/22: runaway after teleop killed) |
| SLAM | FAIL: coverage 0.759, precision 0.577 |
| DQN | 50 episodes, 0 successes; greedy evaluation 0/9 |

### Cloud checks without ROS/Gazebo (made on the previous 6 x 6 layout; layout-specific rows no longer apply) (Claude Cloud, 2026-10-16; details `docs/cloud_analysis.md`, raw output `docs/cloud_checks/`)
| Check | Result |
|---|---|
| `python3 tools/verify_static.py` | **PASS** (61 checks: compile, YAML/XML, world and map regeneration, package/CMake/dependency/world-name/message-type/spawn consistency, Figure-1 match, perfect-pose mapping ceiling, pytest slam_mapping 4 / maze_navigation 6 / dqn_agent 12) |
| `tools/check_figure1.py` | layout == Figure 1: 12 + 12 interior walls, 0 differences |
| Merge of `origin/main` (JayangaWeerasinghe, `7261e7b`) | merged; ament_cmake kept; his world is optional (`world:=custom_maze`) |
| SLAM map analysis (`tools/analyse_slam_map.py`) | recorded map is distorted (20 % of wall pixels axis-aligned vs 77 %); scan-matcher window narrowed - **unverified on the VM** |
| DQN surrogate (`tools/offline_dqn_surrogate.py`, kinematic model, **not Gazebo**) | old settings reproduce the VM failure; new waypoint tolerance / epsilon decay: hybrid 20/30, pure 0/30 on the three goals - **unverified on the VM** |
| Not runnable here | colcon build, Gazebo, SLAM Toolbox, Nav2, teleop, `world:=custom_maze`, DQN in Gazebo |

The older sections below were written before these runs and are kept for history.


## How to run the checks
```bash
python3 tools/verify_static.py                       # no ROS needed, ~10 s
bash tools/verify_on_vm.sh <build|sim|control|slam|nav|dqn>      # one stage; prints a heartbeat every 30 s and the stage time
QUICK=1 bash tools/verify_on_vm.sh slam              # smoke test: partial tour (12 waypoints); validation is not a pass/fail
QUICK=1 bash tools/verify_on_vm.sh dqn               # smoke test: 10 episodes (EPISODES=3 for a 40 s check)
STAGES="sim control nav" bash tools/run_all.sh       # chosen stages with a timing summary
tail -f docs/results/vm/<stage>_*.log                # follow progress from a second terminal
```
Stage lengths (real time; the simulation cannot be sped up): build ~1 min, sim 0.5, control 1.5, nav 2, slam ~7 (full tour), dqn about 8 s per episode and mode.

## Requirement status (current maze) - see also `docs/SUBMISSION_CHECKLIST.md`
| Requirement | Status |
|---|---|
| Maze world in Gazebo "as given in Figure 1" | **PARTIAL / RISK:** teammate's `custom_maze.sdf` loads and is used unmodified (VM sim 10/10), but it is 7 x 6, Figure 1 is 6 x 6 - NOT an exact match |
| TurtleBot3 Burger + LiDAR | PASS (`/scan` 5 Hz, one robot `burger`) |
| SLAM mapping | IMPLEMENTED and run on the current maze; map **below our own 0.85 precision threshold** (0.819), coverage 0.971 |
| Autonomous navigation (Nav2 A*) | PASS 3/3 goals on the current maze (ground-truth map) |
| User clicks a goal | Command-line equivalent PASS; RViz **Nav2 Goal** click NOT VERIFIED here (needs the VM desktop) |
| Deep Q-learning | IMPLEMENTED; unit tests pass; current-maze smoke test only; **no learning demonstrated** |
| Performance (time, path efficiency) | Nav2 measured (report 4.2); DQN comparison has no data |
| Report (5-7 pages), demo | draft in `docs/report.md`; PDF must be regenerated (`tools/build_report_pdf.sh`) |

Record real results in `docs/results/` with the exact command, date and config; never edit numbers by hand.
