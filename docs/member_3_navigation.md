# Member 3 — Autonomous Navigation & Path Planning

**Package:** `ros2_ws/src/maze_navigation` (`ament_cmake`)
**Status:** scaffold only. No Nav2 configuration, planner or launch file exists yet.

## Tasks
- Configure Nav2 (map server, localization, planner, controller, costmaps, behaviours).
- Provide A* or Dijkstra global planning. Check the assignment brief on whether Nav2's built-in NavFn (Dijkstra/A*) is acceptable or a custom planner is required.
- Configure local planning and obstacle avoidance.
- Support clicked goals via RViz ("2D Goal Pose").
- Demonstrate navigation to at least three selected goals.

## Expected files
| Path (inside `maze_navigation/`) | Purpose |
|---|---|
| `config/nav2_params.yaml` | Nav2 parameters |
| `launch/navigation.launch.py` | Nav2 bringup with map and `use_sim_time:=true` |
| `rviz/navigation.rviz` | RViz layout with goal tool |
| `maps/` | Only if a map copy is unavoidable; prefer the one from `slam_mapping` |
| `maze_navigation/` Python module + `scripts/` | Only if a custom planner or goal-sequence runner is written (switch to `ament_cmake_python` or `ament_python` at that point) |

## Dependencies
`nav2_bringup`, `nav2_map_server`, `nav2_amcl`, `nav2_navfn_planner`, `rviz2`. Requires Member 1's simulation and Member 2's saved map.

## Deliverables
- One-command navigation launch.
- Evidence of three successful goal runs (record start, goal, outcome, time).
- Metrics logged in a format Member 4 can reuse for the baseline comparison.
- Screenshots in `docs/screenshots/`, data in `docs/results/`.

## Validation
1. `colcon build --packages-select maze_navigation` succeeds.
2. Nav2 lifecycle nodes all reach *active*.
3. A goal clicked in RViz yields a planned path and the robot reaches it.
4. Three distinct goals are reached; obstacle avoidance is observed (describe how it was tested).

## Remaining
Everything above; blocked on Members 1 and 2.
