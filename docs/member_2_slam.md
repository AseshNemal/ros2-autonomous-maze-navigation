# Member 2 — SLAM & Mapping

**Package:** `ros2_ws/src/slam_mapping` (`ament_cmake`)
**Status:** scaffold only. No configuration, launch file or map exists yet.

## Tasks
- Configure SLAM Toolbox for the simulated LiDAR.
- Build the maze map by driving the robot (teleop or scripted).
- Save the occupancy grid (`.pgm` + `.yaml`) and serialized pose graph if useful.
- Configure localization (SLAM Toolbox localization mode and/or AMCL handoff to Member 3).
- Provide mapping and localization launch files.

## Expected files
| Path (inside `slam_mapping/`) | Purpose |
|---|---|
| `config/mapper_params_online_async.yaml` | SLAM Toolbox parameters |
| `launch/mapping.launch.py` | Starts SLAM Toolbox with `use_sim_time:=true` |
| `launch/localization.launch.py` | Localization on a saved map |
| `maps/maze_map.pgm`, `maps/maze_map.yaml` | Saved map (real output only) |
| `rviz/mapping.rviz` | RViz layout for mapping |

## Dependencies
`slam_toolbox`, `nav2_map_server`, `rviz2`; depends at runtime on Member 1's simulation providing `/scan`, `/odom` and TF.

## Deliverables
- Reproducible mapping procedure (commands and parameters).
- Saved map generated from the real maze world; note how it was produced.
- Map screenshot in `docs/screenshots/`.
- Agreed map file location for Member 3 (single source of truth, no duplicate copies).

## Validation
1. `colcon build --packages-select slam_mapping` succeeds.
2. With simulation running, `mapping.launch.py` publishes `/map` and the `map -> odom` transform.
3. Map visibly matches the maze walls in RViz.
4. Saved map reloads through `nav2_map_server` and localization converges.

## Remaining
Everything above; blocked on Member 1's world and topic interface.
