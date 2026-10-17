# Submission checklist - IE4060 Assignment 02 (state of branch `dev-integration`)

PASS = executed and passed on the Ubuntu VM on the **current maze** (`custom_maze.sdf`, 7 x 6). Evidence: `docs/results/vm/`.
NOT VERIFIED = not executed on the current maze. Results from the removed 6 x 6 maze are in `docs/results/old_6x6_maze/` and do not count.

## Assignment requirements (final state, Ubuntu VM 2026-10-17; evidence in `docs/results/vm/`)
| # | Requirement (brief) | Status | Evidence / note |
|---|---|---|---|
| 1 | Maze world in Gazebo "as given in Figure 1" | **PARTIAL - COMPLIANCE RISK** | World loads and is used unmodified (sim 10/10). It is the teammate's **7 x 6** maze; Figure 1 is **6 x 6**. Not an exact match; stated in the report. |
| 2 | Simulation world file in the repo | PASS | `ros2_ws/src/maze_simulation/worlds/custom_maze.sdf` (byte-identical to origin/main) |
| 3 | TurtleBot3 Burger + LiDAR | PASS | `sim_20261017-101901.log` |
| 4 | SLAM / mapping | **PASS (implemented), accuracy below our own threshold** | Coverage 0.971, precision 0.819 (own threshold 0.85), free-OK 1.0. `slam_validation_20261017-111320.txt` |
| 5 | Autonomous navigation (Nav2 A*) | PASS | Scripted `verify_on_vm.sh nav` on the corrected map: 3/3 goals, 0 recoveries, 38.5 / 15.9 / 10.3 s (`nav2_results_20261017-165127.csv`) |
| 6 | User clicks a goal; robot goes there avoiding collisions | **PARTIAL** | Command-line goals and window-mode goals PASS (`tools/send_goal.sh`). A scripted RViz *Nav2 Goal* click is NOT VERIFIED; rehearse 3 clicks on the VM desktop before the VIVA. |
| 7 | Deep Q-learning implemented | PASS (implemented) | 22 unit tests; 60-episode training, 1488 optimizer updates |
| 8 | DQN trained / evaluated, learning shown | **FAIL (no learning shown)** | Truth-confirmed greedy evaluation 0/9 goals (7 collisions, 2 timeouts). An earlier odometry-based 5/9 was a measurement artefact. |
| 9 | Efficiency: travel time, path length, obstacle avoidance | PARTIAL | Nav2 times/distances measured; no DQN comparison possible (0 goals). |
| 10 | Safety (no runaway) | PASS | `control_20261017-102106.log` 22/22 |
| 11 | Final report 5-7 pages | PASS (6 pages) | `docs/report.pdf`, built with the WeasyPrint fallback (pandoc/Chromium need sudo) |
| 12 | GitHub repo with packages, launch, config | READY | Branch `dev-integration`; the latest local commits are **not pushed** until approved |
| 13 | VIVA: 3 different clicked goals | NOT VERIFIED here | Rehearse once on the VM desktop |

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

## Final-session changes (2026-10-17)
- Ground-truth map east wall fixed; DQN reward, truncation, checkpoint, reset and fresh-scan fixes; DQN success now confirmed with Gazebo ground truth (odometry gave false successes); Nav2 `default_server_timeout` 20 -> 1000 ms; cleanup scripts SIGKILL stale Nav2 nodes and the watchdog.
- Tests: `python3 tools/verify_static.py` (slam_mapping, maze_navigation, dqn_agent pytest) plus `bash tools/verify_on_vm.sh build`.

## Remaining before the VIVA
1. Rehearse three clicked *Nav2 Goal* goals in RViz (do not click *2D Pose Estimate*); fallback `bash tools/send_goal.sh G1|G2|G3`.
2. Decide about the Figure 1 mismatch (7 x 6 vs 6 x 6) and about further DQN training (needs approval; not run).
3. Push `dev-integration` when approved.
