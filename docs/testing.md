# Testing

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
