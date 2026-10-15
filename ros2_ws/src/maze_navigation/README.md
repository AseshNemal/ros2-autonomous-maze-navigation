# maze_navigation

Member 3: Navigation configuration for TurtleBot3 Burger using ROS 2 Jazzy and Nav2.

## Current implementation

- NavFn global planner configured to use A*.
- MPPI controller with initial forward/reverse speed limits of 0.15/-0.10 m/s.
- Initial angular speed limit of 1.0 rad/s.
- Velocity smoother limits matched to the controller.
- Navigation launch file using a supplied saved map and AMCL.
- Optional RViz window with the navigation layout.
- Simulation time enabled by default.

These are initial settings. Robot footprint, costmaps and simulation interfaces still need verification and tuning.

## Files

- `config/nav2_params.yaml`: Nav2 configuration.
- `launch/navigation.launch.py`: Nav2, AMCL and optional RViz launch.
- `rviz/navigation.rviz`: RViz navigation layout.

## Build

Run from the repository's `ros2_ws` directory:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select maze_navigation
source install/setup.bash
```

## Run

Start the team's simulation first. Then provide the absolute path to the saved map YAML from `slam_mapping`:

```bash
ros2 launch maze_navigation navigation.launch.py map:=/absolute/path/to/maze.yaml
```

The map path above is a placeholder. Replace it with the real saved map path.

To run without RViz:

```bash
ros2 launch maze_navigation navigation.launch.py map:=/absolute/path/to/maze.yaml use_rviz:=false
```

This launch starts AMCL and the map server. Do not run another AMCL instance or another map server publishing the same map topic at the same time.

## Integration requirements

- A running TurtleBot3 Burger simulation.
- Simulation clock, LiDAR and odometry data.
- A saved map and its referenced image file.
- Compatible TF frames and velocity command interfaces.
- Only one active navigation method publishing robot commands at a time.

Coordinate final topic names, TF frames and command message types with the simulation and mapping members.

## Validation completed

- YAML syntax and selected parameter values checked.
- Launch Python syntax checked.
- Package build passed.
- Launch arguments loaded successfully.
- RViz layout opened successfully.
- RViz reported a shutdown crash after Ctrl+C; shutdown behaviour needs retesting.

## Pending validation

- Robot footprint and costmap configuration.
- Simulation topic, TF and command message compatibility.
- Nav2 lifecycle activation and AMCL localization.
- Collision-free navigation to three clicked goals.
- Travel time, path length and success measurements.
- Performance tuning and comparison with the DQN component.

Full robot navigation has not been tested yet.

## Goal evaluation tool

The evaluator sends its own Nav2 goal. It does not record goals clicked in RViz.

After simulation, localization and Nav2 are ready, run from the repository's ros2_ws directory:

```bash
ros2 run maze_navigation evaluate_goal.py \
  --x 1.0 --y 1.0 --yaw 0.0 \
  --run-id goal_01 \
  --csv ../docs/results/nav2_results.csv \
  --ros-args -p use_sim_time:=true
```

The coordinates above are examples, not validated maze goals.
Replace them with agreed reachable goals. Yaw is in radians.

CSV fields include the goal, outcome, elapsed simulation time,
odometry distance, sample count and metrics_valid.

Timing begins when goal acceptance is received. Distance is estimated
from odometry samples received during the run; it is not planned path
length or a collision count.

Do not reset the simulation, send another goal or use teleoperation
during a recorded run. If interrupted with Ctrl+C, the goal may remain
active and must be canceled separately.

Validation completed: Python syntax, package build and --help.
Goal execution and CSV measurements still require runtime testing.
