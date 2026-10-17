# Member 2 - SLAM and Mapping (`slam_mapping`)

Status key: **VERIFIED** (actually run and passed) · **IMPLEMENTED** (code exists, runtime check pending) · **FAILED** · **BLOCKED** · **NOT IMPLEMENTED**.
Runtime results on the current maze (Ubuntu VM, 2026-10-17): see [testing.md](../testing.md) and `docs/results/vm/`. Summary below under Tests.

## Responsibility
SLAM Toolbox mapping, occupancy grid generation, map save/load, localization (AMCL), map validation against the maze.

## Provenance
| Item | Origin |
|---|---|
| `maps/test_map.pgm/.yaml` | **Original teammate work**: `dhanushka403`, commit `7544234`, merged via PR #1 (merge commit, history intact). |
| Everything else in `slam_mapping/` | **Emergency integration code** (commit `3d07b0c`, `feat(slam)`), AI-assisted, by the branch owner. |

## Finding about the original map (inspected, not assumed)
`test_map` is 34x45 px (1.7 m x 2.25 m), **78.6% unknown**, partial fan-shaped fragment. `validate_map.py` against the current maze (`custom_maze.sdf`) -> **FAIL** (wall coverage and precision near zero). It is not the assignment maze (probably a quick prototype scan from a different world). It is kept for history and is *not* used by any launch file.

## Contribution
- `config/slam_toolbox_mapping.yaml` - online async mapping tuned for the small maze (frames `odom/map/base_footprint`, `/scan`, 0.05 m resolution, 3.5 m max range, loop closing off, scan-matcher search window 0.2 m).
- `launch/mapping.launch.py` - wraps slam_toolbox `online_async_launch.py` + RViz.
- `launch/localization.launch.py` - nav2 map_server + AMCL with initial-pose publisher.
- `scripts/save_map.sh` - `nav2_map_server map_saver_cli`.
- `scripts/validate_map.py` - pure-Python validator (wall coverage / precision / free-cell check vs the maze read from `custom_maze.sdf`).
- `scripts/tour_map_drive.py` - coverage tour over every cell, steered by the SLAM pose; this is what produced the current-maze maps.
- `scripts/auto_map_drive.py` - right-hand wall follower for unattended mapping (TwistStamped); superseded by the tour (it cannot map interior wall islands).
- `scripts/set_initial_pose.py` - publishes `/initialpose` until AMCL confirms.
- `maps/maze_groundtruth.*` - **rasterised from `custom_maze.sdf`, not a SLAM result.** Used for the Nav2 tests. The east outer wall was missing from the earlier file (pixel-edge float tie) and is fixed; `test_groundtruth_vs_sdf.py` now guards it.
- `maps/maze_slam.*` - the SLAM Toolbox map of run 2 (precision 0.819).
- `rviz/mapping.rviz`.

## Architecture
`/scan + /odom TF` -> `slam_toolbox` (scan matching, pose graph) -> `/map` + `map->odom` TF -> `map_saver_cli` -> `.pgm/.yaml` -> `map_server` + `amcl` (particle filter, likelihood field) -> `map->odom`.
The SLAM map frame origin is the robot's spawn pose, so a SLAM map is offset from world coordinates by (0.25, 0.0); the validator, `set_initial_pose.py` and `run_three_goals.py --map-source slam` all account for this (assumes spawn yaw 0).

## Topics
Subscribes `/scan`, `/tf`, `/odom` (via TF); publishes `/map`, `/map_metadata`, `map->odom` TF; AMCL publishes `/amcl_pose`, subscribes `/initialpose`.

## Run (simulation first; **no Nav2/DQN at the same time**)
```bash
ros2 launch maze_simulation maze_simulation.launch.py gui:=false
ros2 launch slam_mapping mapping.launch.py
ros2 run slam_mapping tour_map_drive.py                    # or auto_map_drive.py --duration 420, or teleop_twist_keyboard --ros-args -p stamped:=true
export MAZE_REPO=$PWD; ros2 run slam_mapping save_map.sh maze_slam
python3 ros2_ws/src/slam_mapping/scripts/validate_map.py ros2_ws/src/slam_mapping/maps/maze_slam.yaml --map-origin-in-world 0.25 0.0
ros2 launch slam_mapping localization.launch.py map:=$PWD/ros2_ws/src/slam_mapping/maps/maze_slam.yaml initial_x:=0.0 initial_y:=0.0
```
Or staged: `tools/verify_on_vm.sh slam`.

## Tests
- VERIFIED (static, pytest): validator accepts the ground-truth map, rejects a shifted copy, rejects `test_map`; the ground-truth map matches every wall of the SDF.
- VERIFIED (Ubuntu VM, `docs/results/vm/slam_validation_*.txt`): run 1 (odometry-steered tour) coverage 0.890, precision 0.256 -> FAIL; run 2 (SLAM-pose-steered tour) coverage 0.971, **precision 0.819**, free cells OK 1.0 -> FAIL against our own 0.85 precision threshold (coverage and free-cell checks pass). The brief sets no numeric threshold. AMCL localisation on the ground-truth map works (Nav2 goals reached, see Member 3).
- NOT DONE: navigation on the SLAM map.

## Limitations
Odometry drift when the tour brushes walls smears the map (precision 0.256 in run 1; wall contact in 189 of 371 ground-truth samples), reduced but not eliminated in run 2. Validator pass = geometric agreement only. The 0.85 thresholds are our own.

## VIVA
*Why SLAM Toolbox?* Maintained, works with Nav2, async mode copes with a slow VM. *How does a particle filter localise?* AMCL samples poses, weights them by how well scan endpoints fall on occupied map cells (likelihood field), resamples. *Why validate a map?* To catch the case we found: a map that loads fine but is a different world.
