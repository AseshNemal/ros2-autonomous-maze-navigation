# Submission checklist - IE4060 Assignment 02 (state of branch `dev-integration`)

PASS = executed and passed on the Ubuntu VM on the **current maze** (`custom_maze.sdf`, 7 x 6). Evidence: `docs/results/vm/`.
NOT VERIFIED = not executed on the current maze. Results from the removed 6 x 6 maze are in `docs/results/old_6x6_maze/` and do not count.

## Assignment requirements
| # | Requirement (brief) | Status | Evidence / note |
|---|---|---|---|
| 1 | Maze world in Gazebo "as given in Figure 1" | **PARTIAL - COMPLIANCE RISK** | World loads and is used unmodified (VM sim 10/10, one robot `burger`). It is the teammate's **7 x 6** maze; Figure 1 is **6 x 6**. Not an exact match - state this in the report/VIVA. |
| 2 | Simulation world file (`.world`/SDF) in the repo | PASS | `ros2_ws/src/maze_simulation/worlds/custom_maze.sdf` (byte-identical to origin/main) |
| 3 | TurtleBot3 Burger + LiDAR | PASS | `sim_20261017-101901.log` (/scan 5 Hz, /odom 49.6 Hz, TF, /clock) |
| 4 | SLAM / mapping | **PASS (implemented), accuracy below our own threshold** | Coverage 0.971, precision 0.819 (own threshold 0.85), free-OK 1.0. The brief sets no numeric threshold. `slam_validation_20261017-111320.txt`, `slam_map_20261017-111320.png` |
| 5 | Autonomous navigation (Nav2 Dijkstra/A*) | PASS | 3/3 goals, 1 recovery total: `nav2_results_20261017-113157.csv` |
| 6 | User clicks a goal; robot goes there avoiding collisions | **PARTIAL** | RViz **Nav2 Goal** click NOT VERIFIED by script (needs the VM desktop - do it at the VIVA). Equivalent command-line goals PASS (`tools/send_goal.sh`, `cmdline_send_goal_20261017.txt`). |
| 7 | Deep Q-learning implemented | PASS (implemented) | 12 unit tests pass; training loop, reset, CSV, checkpoint ran on the current maze (3-episode smoke) |
| 8 | DQN trained / evaluated, learning shown | **NOT VERIFIED / NOT DEMONSTRATED** | No training or evaluation result on the current maze. Old maze: 0 successes in 50 episodes, 0/9 evaluation (historical). Do not claim learning. |
| 9 | Efficiency: travel time, path length, obstacle avoidance | PARTIAL | Nav2 times/distances measured (report 4.2 C2). DQN comparison: no data. |
| 10 | Safety (no runaway) | PASS | `control_20261017-102106.log` 22/22 (watchdog stops the robot after teleop is killed) |
| 11 | Final report 5-7 pages | NOT DONE (draft ready) | `docs/report.md` updated with real current-maze results; regenerate the PDF: `bash tools/build_report_pdf.sh`; check it is 5-7 pages; put the GitHub link on the cover page |
| 12 | GitHub repo with packages, launch, config | READY locally | Branch `dev-integration` committed; latest commits **not pushed** |
| 13 | VIVA: 3 different clicked goals | NOT VERIFIED here | Rehearse once on the VM desktop (commands below) |

## Known failures / risks (be upfront about them)
1. Maze is 7 x 6, not the 6 x 6 of Figure 1.
2. SLAM precision 0.819 < our own 0.85; the coverage tour still brushes walls sometimes.
3. No DQN learning result on the current maze (and none on the old one).
4. Nav2 was only run on the ground-truth map, not on the SLAM map.
5. First control run had 1 flaky check (`/scan` count right after start-up); the rerun was 22/22.

## Exact commands on Ubuntu
```bash
cd ~/ros2-autonomous-maze-navigation && git checkout dev-integration && git status      # must be clean
cd ros2_ws && colcon build --symlink-install --parallel-workers 1 && cd ..
python3 tools/verify_static.py                         # expect OVERALL: PASS (+ one WARN about Figure 1)

# 1. VIVA demo (VM desktop, not SSH). Terminal 1 (same as: bash tools/auto.sh):
bash tools/demo.sh                                     # Gazebo + RViz + Nav2; wait for ">>> Nav2 is ACTIVE"
#    click Nav2 Goal in RViz, 3 different points.  If the mouse is unreliable, Terminal 2:
bash tools/send_goal.sh G1 ; bash tools/send_goal.sh G2 ; bash tools/send_goal.sh G3

# 2. Keyboard driving instead (stop the demo first; same as: bash tools/manual.sh):
bash tools/demo.sh teleop                              # keep that terminal focused; i , j l k ; z slows down
#    or without keyboard focus: start only the simulation, then drive in steps
ros2 launch maze_simulation maze_simulation.launch.py &   # (source ROS + ros2_ws/install first)
bash tools/drive.sh forward 2 ; bash tools/drive.sh left 1 ; bash tools/drive.sh stop

# 3. Optional evidence still missing (each is a separate, long run - only if time allows)
EPISODES=50 GUIDANCE=none bash tools/verify_on_vm.sh dqn        # ~7 min training + evaluation, records the real result
bash tools/verify_on_vm.sh slam                                  # ~7 min; re-measures the SLAM map
MAP_SOURCE=slam bash tools/verify_on_vm.sh nav                   # Nav2 on the SLAM map (after the slam stage)

# 4. Report
bash tools/build_report_pdf.sh                          # -> docs/report.pdf (check 5-7 pages)
```

## Final-session changes (2026-10-17, tested statically only: `verify_static.py` PASS, 8 + 6 + 18 unit tests)
- Ground-truth map: the east outer wall was missing (fixed in `maze_tools.write_map`, map regenerated, test `test_groundtruth_vs_sdf.py`). The Nav2 results in the table were recorded before this fix.
- DQN: always-on near-wall penalty removed, truncation no longer stored as terminal, checkpoint keeps hyperparameters/epsilon, reset waits for a post-teleport scan, `-inf` LiDAR treated as close, default training pair = shortest goal.
- Report rewritten for the current maze with member names/IDs; PDF must be rebuilt on a machine with pandoc + Chromium.

## Still to run on the Ubuntu VM (nothing below is verified yet)
```bash
cd ~/ros2-autonomous-maze-navigation && git pull && cd ros2_ws && colcon build --symlink-install --parallel-workers 1 && cd ..
bash tools/verify_on_vm.sh nav                                          # 1. Nav2 3 goals on the corrected map (C2b)
EPISODES=8 GUIDANCE=none bash tools/verify_on_vm.sh dqn                 # 2. smoke: confirm updates > 0 (train_steps column)
EPISODES=60 GUIDANCE=none bash tools/verify_on_vm.sh dqn                # 3. only if 2 is OK: one bounded run
bash tools/build_report_pdf.sh                                          # 4. report PDF (check 5-7 pages)
```
Then: rehearse 3 clicked goals in RViz, take one Gazebo and one RViz screenshot, push only after approval.
