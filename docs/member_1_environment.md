# Member 1 — Gazebo Environment & Robot Simulation

**Package:** `ros2_ws/src/maze_simulation` (`ament_cmake`)
**Status:** scaffold only. No world, launch file or robot configuration exists yet.

## Tasks
- Build the custom Gazebo (Harmonic) maze world from the assignment diagram.
- Spawn and configure TurtleBot3 Burger.
- Configure the LiDAR and any other sensors the other members need.
- Bridge Gazebo topics to ROS 2 (`ros_gz_bridge`).
- Provide simulation launch files and verify spawning.

## Expected files
| Path (inside `maze_simulation/`) | Purpose |
|---|---|
| `worlds/maze.sdf` | Maze world (walls, floor, lighting) |
| `models/` | Any custom SDF models used by the world |
| `launch/simulation.launch.py` | Starts Gazebo, spawns robot, starts bridge |
| `config/bridge.yaml` | `ros_gz_bridge` topic mapping |
| `rviz/simulation.rviz` | Optional RViz view for checking sensors |

Add any new directory to the `install(DIRECTORY ...)` list in `CMakeLists.txt`.

## Dependencies
Declared in `package.xml`: `ros_gz_sim`, `ros_gz_bridge`, `turtlebot3_description`, `turtlebot3_gazebo`, `robot_state_publisher`. Confirm the exact Jazzy/Harmonic package set when installing; TurtleBot3 Gazebo support differs between releases.

## Interface other members rely on (to be fixed and documented here)
Topic names and frames for `/scan`, `/odom`, `/cmd_vel`, `/tf`, `/clock`, and `use_sim_time` handling. Record the final list in `docs/integration.md` once it exists.

## Deliverables
- Maze world matching the assignment diagram (record dimensions and spawn pose).
- Launch file(s) that bring up the simulation with one command.
- Screenshots of the maze and the spawned robot in `docs/screenshots/`.

## Validation
1. `colcon build --packages-select maze_simulation` succeeds.
2. `ros2 launch maze_simulation simulation.launch.py` opens the world and spawns the robot without errors.
3. `ros2 topic echo /scan --once` shows LiDAR data; `/odom` publishes.
4. Teleoperating the robot confirms walls are collidable.

## Remaining
Everything above. None of it has been implemented or run.
