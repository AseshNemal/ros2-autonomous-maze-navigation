# Testing

## Latest verified status (VM, 2026-10-16/17; evidence in `docs/results/vm/`)
| Stage | Result |
|---|---|
| build + tests | PASS, 22/22 tests |
| sim (headless) | PASS, 7/7 checks |
| nav (Nav2, 3 goals, ground-truth map) | PASS, 3/3 goals, 0 recoveries (`nav2_results_20261016-214253.csv`) |
| control (teleop + Nav2 goal + `/cmd_vel` publishers) | PASS, 22/22 after adding `cmd_vel_watchdog` (first run 21/22: runaway after teleop killed) |
| slam | **FAIL**: coverage 0.759, precision 0.577 (need 0.85 / 0.85); map has the right size and structure |
| dqn | stage ran; **no learning**: 50 episodes 0 successes, evaluation 0/9 |
| Nav2 on the SLAM map | NOT TESTED |
| remote-desktop input delay / disconnects | NOT VERIFIABLE by script (sim real-time factor 1.00) |

The older sections below were written before these runs and are kept for history.


## Automated, no ROS (run on the Mac) - `python3 tools/verify_static.py`
Result on this branch (run 2026-10-16): **PASS** - python compiles; 6 YAML + 5 XML files parse; package names match; `maze.world` and ground-truth map equal their regeneration from `maze_layout.yaml`; pytest: slam_mapping 3 passed, maze_navigation 6 passed, dqn_agent 11 passed.
This shows internal consistency only.

## Runtime on the Ubuntu VM - `tools/verify_on_vm.sh <stage>` (logs to `docs/results/vm/`)
| Stage | What it checks | Status |
|---|---|---|
| build | `colcon build` + `colcon test` of 4 packages | **UNVERIFIED** |
| sim | world loads, robot spawns, `/scan /odom /tf /clock` rates & frames, `/cmd_vel` type | **UNVERIFIED** |
| slam | slam_toolbox + auto mapping -> `maze_slam.yaml` -> `validate_map.py` | **UNVERIFIED** |
| nav | Nav2 lifecycle, collision_monitor present, 3 goals CSV | **UNVERIFIED** |
| dqn | training CSV + greedy evaluation CSV; `compare_navigation` | **UNVERIFIED** |

## Requirement status
| Requirement | Status |
|---|---|
| Maze world matching Figure 1 | IMPLEMENTED (geometry statically verified vs figure; Gazebo load unverified) |
| TurtleBot3 spawn, sensors | IMPLEMENTED |
| SLAM map of the maze | NOT RUN (blocker: Gazebo access) - `test_map` FAILS validation |
| Localization | IMPLEMENTED |
| Nav2 A* navigation, clicked goals | IMPLEMENTED (unverified) |
| 3-goal demo + metrics | IMPLEMENTED (script); no results |
| DQN training/eval/checkpoints | IMPLEMENTED; unit tests VERIFIED; maze runs unverified |
| DQN reaches goals | NOT DEMONSTRATED (Stage-1 prior: 0/3) |
| DQN vs Nav2 comparison | IMPLEMENTED (tool); no data |
| Report draft | IMPLEMENTED (results sections PENDING) |

Record real results in `docs/results/` with the exact command, date and config; never edit numbers by hand.
