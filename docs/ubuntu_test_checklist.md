# Ubuntu test checklist (run when the VM is available)

Everything below is **unverified until you run it**. Do the steps in order; stop at the first failure and send me the
log. Each stage writes to `docs/results/vm/` (logs are now tracked by Git). Commit those files - they are the evidence.
Run only one stack at a time; the stage scripts kill leftovers themselves.

```bash
cd ~/ros2-autonomous-maze-navigation && git pull origin dev-integration
export TURTLEBOT3_MODEL=burger
```

| # | Step | Command | Pass criteria | If it fails |
|---|---|---|---|---|
| 0 | Static + cloud checks | `pip install pytest numpy pyyaml pillow; python3 tools/verify_static.py` | `OVERALL: PASS` (about 60 checks incl. Figure-1 match) | fix before anything else |
| 1 | Build + unit tests | `bash tools/verify_on_vm.sh build` | `STAGE RESULT: PASS`; 22 tests as before (`maze_simulation` must build as **ament_cmake**; the merge removed Jayanga's `setup.py`) | send `build_*.log` |
| 2 | Simulation | `bash tools/verify_on_vm.sh sim` | 7 of 7; no clock-jump / TF-extrapolation warnings in the log (`grep -ci "jump\|extrapolat" docs/results/vm/sim_*.log` = 0) | check for a stale `gz`/bridge process |
| 3 | Spawn + manual control | `bash tools/verify_on_vm.sh control` | 22 of 22 (teleop 16, Nav2 6; robot stops by itself after teleop is killed) | `control_*.json` |
| 3b | Live look (optional) | `bash tools/demo.sh teleop` | window shows the Figure-1 maze; robot starts bottom-left; keys move it; take a screenshot | |
| 4 | **SLAM** (changed config) | `bash tools/verify_on_vm.sh slam` | `RESULT: PASS` (coverage, precision >= 0.85; free >= 0.95). Previous run: 0.759 / 0.577 | see "SLAM fails" below |
| 5 | Nav2, 3 goals, ground truth | `bash tools/verify_on_vm.sh nav` | 3/3 goals, CSV in `docs/results/vm/nav2_results_*.csv` | |
| 5b | Nav2 on the SLAM map (only if step 4 passed) | `MAP_SOURCE=slam bash tools/verify_on_vm.sh nav` | 3/3 goals | |
| 6 | DQN (pure **and** hybrid, changed settings) | `EPISODES=300 bash tools/verify_on_vm.sh dqn` | stage completes; compare the two `dqn_eval_*` CSVs. Expect pure DQN to fail long goals; hybrid should reach some. About 1-2 h | `GUIDANCE=waypoints EPISODES=500 bash tools/verify_on_vm.sh dqn` |
| 7 | Comparison table | `ros2 run dqn_agent compare_navigation --help` then run with the Nav2 and DQN eval CSVs | table for the report (do not edit numbers by hand) | |
| 8 | **VIVA / screenshots** | `bash tools/demo.sh` (Gazebo + RViz); press **2D Goal Pose**, click three different points | robot reaches each; screenshot RViz with the path, Gazebo with the maze, the three CSV rows | `docs/demonstration.md`, `docs/viva.md` |
| 9 | Optional world | `ros2 launch maze_simulation maze_simulation.launch.py world:=custom_maze x_pose:=0.25 y_pose:=0.0` | window shows Jayanga's 7 x 6 arena, one Burger spawned, `/scan` `/odom` publish. Default launch (no `world:=`) must still show our maze | note result honestly in `docs/testing.md` |

## SLAM fails again - diagnose in this order (do not retune blindly)
1. `python3 tools/analyse_slam_map.py ros2_ws/src/slam_mapping/maps/maze_slam.yaml` - high "within 5 deg of an axis" (> 0.6) means the map is no longer rotated; low means heading was lost again.
2. Is odometry the problem? During a tour compare odom with Gazebo truth: `ros2 topic echo /odom --once --field pose.pose` vs `gz topic -e -n 1 -t /world/maze/dynamic_pose/info` before and after (the first VM test saw 0.04 rad per turn).
3. If heading is still lost, revert the two lines in `slam_toolbox_mapping.yaml` (old values are in the comment) or lower `--speed` / `--turn` of `tour_map_drive.py`; change one thing per run and keep every `slam_validation_*.txt`.
4. Do not report a SLAM pass unless `validate_map.py` printed `RESULT: PASS` for a map saved by `slam_toolbox` alone.

## Evidence to commit and update afterwards
* `docs/results/vm/*` (CSV, JSON, **log** files) - do not hand-edit.
* Screenshots into `docs/screenshots/` (Gazebo maze, RViz map + path, SLAM map, three goal reaches).
* Update `docs/testing.md` (status table) and `docs/report.md` sections 4.3 / 4.5 with the new numbers, then `bash tools/build_report_pdf.sh`.
* Fill student names and IDs on the report cover. Share the repository link (CourseWeb) - note the default branch `main` still has the old state; point to `dev-integration` or merge it yourself.
