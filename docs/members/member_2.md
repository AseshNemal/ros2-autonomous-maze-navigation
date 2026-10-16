# Member 2 - SLAM and Mapping (`slam_mapping`)

Status key: **VERIFIED** (actually run and passed) · **IMPLEMENTED** (code exists, runtime check pending) · **FAILED** · **BLOCKED** · **NOT IMPLEMENTED**.
No Gazebo/Nav2/SLAM/DQN runtime test has been run from this branch yet; only the static checks in `tools/verify_static.py` (Mac) have. See [testing.md](../testing.md).

## Responsibility
SLAM Toolbox mapping, occupancy grid generation, map save/load, localization (AMCL), map validation against the maze.

## Provenance
| Item | Origin |
|---|---|
| `maps/test_map.pgm/.yaml` | **Original teammate work**: `dhanushka403`, commit `7544234`, merged via PR #1 (merge commit, history intact). |
| Everything else in `slam_mapping/` | **Emergency integration code** (commit `3d07b0c`, `feat(slam)`), AI-assisted, by the branch owner. |

## Finding about the original map (inspected, not assumed)
`test_map` is 34x45 px (1.7 m x 2.25 m), **78.6% unknown**, partial fan-shaped fragment. `validate_map.py` against the real maze layout: wall_coverage 0.061, precision 0.218 -> **FAIL**. It is not the 3 m x 3 m assignment maze (probably a quick prototype scan from a different world). It is kept for history and is *not* used by any launch file.

## Contribution
- `config/slam_toolbox_mapping.yaml` - online async mapping tuned for the small maze (frames `odom/map/base_footprint`, `/scan`, 0.05 m resolution, 3.5 m max range).
- `launch/mapping.launch.py` - wraps slam_toolbox `online_async_launch.py` + RViz.
- `launch/localization.launch.py` - nav2 map_server + AMCL with initial-pose publisher.
- `scripts/save_map.sh` - `nav2_map_server map_saver_cli`.
- `scripts/validate_map.py` - pure-Python validator (wall coverage / precision / free-cell check vs `maze_layout.yaml`).
- `scripts/auto_map_drive.py` - right-hand wall follower for unattended mapping (TwistStamped).
- `scripts/set_initial_pose.py` - publishes `/initialpose` until AMCL confirms.
- `maps/maze_groundtruth.*` - **rasterised from the layout, not a SLAM result.** Exact fallback for Nav2 testing.
- `rviz/mapping.rviz`.

## Architecture
`/scan + /odom TF` -> `slam_toolbox` (scan matching, pose graph) -> `/map` + `map->odom` TF -> `map_saver_cli` -> `.pgm/.yaml` -> `map_server` + `amcl` (particle filter, likelihood field) -> `map->odom`.
The SLAM map frame origin is the robot's spawn pose, so a SLAM map is offset from world coordinates by (-1.25,-1.25); the validator, `set_initial_pose.py` and `run_three_goals.py --map-source slam` all account for this (assumes spawn yaw 0).

## Topics
Subscribes `/scan`, `/tf`, `/odom` (via TF); publishes `/map`, `/map_metadata`, `map->odom` TF; AMCL publishes `/amcl_pose`, subscribes `/initialpose`.

## Run (simulation first; **no Nav2/DQN at the same time**)
```bash
ros2 launch maze_simulation maze_simulation.launch.py gui:=false
ros2 launch slam_mapping mapping.launch.py
ros2 run slam_mapping auto_map_drive.py --duration 420     # or: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
export MAZE_REPO=$PWD; ros2 run slam_mapping save_map.sh maze_slam
python3 ros2_ws/src/slam_mapping/scripts/validate_map.py ros2_ws/src/slam_mapping/maps/maze_slam.yaml --map-origin-in-world -1.25 -1.25
ros2 launch slam_mapping localization.launch.py map:=$PWD/ros2_ws/src/slam_mapping/maps/maze_slam.yaml initial_x:=0.0 initial_y:=0.0
```
Or staged: `tools/verify_on_vm.sh slam`.

## Tests
- VERIFIED (static, pytest): validator accepts the ground-truth map, rejects a shifted copy, rejects `test_map`.
- UNVERIFIED: slam_toolbox run, real `maze_slam` map, AMCL convergence. **Blocker:** needs Gazebo on the VM; until then no SLAM-built map exists in the repo.

## Limitations
The wall-follower is a heuristic and may not cover every cell (judge by validator `unknown_fraction`/coverage; fall back to teleop). Validator pass = geometric agreement only. slam_toolbox parameter tuning for the 0.45 m corridors is a first guess.

## VIVA
*Why SLAM Toolbox?* Maintained, works with Nav2, async mode copes with a slow VM. *How does a particle filter localise?* AMCL samples poses, weights them by how well scan endpoints fall on occupied map cells (likelihood field), resamples. *Why validate a map?* To catch the case we found: a map that loads fine but is a different world.
