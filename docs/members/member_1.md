# Member 1 - Gazebo Simulation (`maze_simulation`)

Status key: **VERIFIED** (actually run and passed) · **IMPLEMENTED** (code exists, runtime check pending) · **FAILED** · **BLOCKED** · **NOT IMPLEMENTED**.
Runtime status on the current maze (`custom_maze.sdf`): see [testing.md](../testing.md) and `docs/results/vm/`.

## Responsibility
JayangaWeerasinghe's Gazebo Harmonic maze (`custom_maze.sdf`, an approximately 7 x 6 approximation of Figure 1, which is 6 x 6), TurtleBot3 Burger spawn, LiDAR/odometry bridging, simulation launch, consistent frames and naming.

## Provenance
**Member 1's own commits (JayangaWeerasinghe, on `main`: `541b10a`, `a26c3cf`, `7261e7b`)** are preserved in this branch's history with their authorship: the environment-versions document, test-map assets, and an optional prototype world (`worlds/custom_maze.sdf`, `generate_maze.py`, `FIGURE1_README.md`, `launch/custom_maze.launch.py`). That world is a 7 x 6 approximation of Figure 1, which is a 6 x 6 grid (`tools/check_figure1.py` reports FIGURE1 MISMATCH). **Current state:** it is now the project's only world (an earlier generated 6 x 6 world was removed), used unmodified for every current result; the world, `generate_maze.py` and `launch/custom_maze.launch.py` are Member 1's work and are kept byte-identical to `main`. The wrapper launch file, command watchdog, `maze_tools.py`, `mission.yaml` and the verification scripts are *integration code* by the branch owner (AI-assisted) and must not be presented as Member 1's work.

## Contribution
- `config/mission.yaml` - start cell and the three goal cells. The maze itself is not duplicated: main's `custom_maze.sdf` (JayangaWeerasinghe, `generate_maze.py`) is 7 columns x 6 rows of 0.5 m cells (3.5 m x 3.0 m). Main describes it as an approximation of Figure 1; compare the `maze_tools.py check` ASCII render with the figure by eye.
- `scripts/maze_tools.py` - reads the maze from `custom_maze.sdf`: ground-truth map, grid A* reference path lengths, ASCII preview. It no longer generates a world.
- `worlds/custom_maze.sdf` - main's Figure 1 world, used unmodified (16 wall models, outer 3.5x3.0 m boundary, 0.01 m thick, 0.2 m high). World name `custom_maze`. `maze_tools.py` converts it to the cell/wall graph used by the planners.
- `launch/custom_maze.launch.py` - JayangaWeerasinghe's launcher (unchanged from `main`): Gazebo with `custom_maze.sdf`, spawns ONE Burger named `burger` at (0.25, 0.0), the stock TurtleBot3 bridge (`/clock /odom /tf /joint_states /scan /imu /cmd_vel`), robot_state_publisher.
- `launch/maze_simulation.launch.py` - integration wrapper: includes `custom_maze.launch.py` (`gui:=false` -> `-s -r --headless-rendering`) and adds `cmd_vel_watchdog.py`. All demos, SLAM, Nav2, DQN and the test stages start the world through it; there is no second world or bridge definition.
- `scripts/check_sim_topics.py` - PASS/FAIL contract check (rates, frames, TF, `/cmd_vel` type).

## Architecture
```
custom_maze.sdf --gz sim--> TB3 Burger (turtlebot3_gazebo model.sdf)
                                                    |  gz topics
                                              ros_gz_bridge (stock turtlebot3_burger_bridge.yaml)
                              /scan /odom /tf /clock /imu /joint_states  <--  --> /cmd_vel (TwistStamped)
```
Robot model and URDF come from the apt `turtlebot3_gazebo` package (not copied). Entity name: `burger` (spawned by main's launcher; all reset/pose code uses it). Frames: `odom -> base_footprint -> base_link -> base_scan`. Spawn: world (0.25, 0.0), yaw 0 (main's; nearest cell (5,1)).

## Interface (contract for the other members)
| Topic | Type | Direction |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | sim -> all |
| `/odom` | `nav_msgs/Odometry` (`odom`->`base_footprint`) | sim -> all |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | sim/RSP -> all |
| `/clock` | `rosgraph_msgs/Clock` | sim -> all (`use_sim_time:=true`) |
| `/cmd_vel` | **`geometry_msgs/TwistStamped`** | controllers -> sim |
Reset service used by DQN/eval: `gz service -s /world/custom_maze/set_pose` (needs the UserCommands plugin, present in the world).

## Run
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch maze_simulation maze_simulation.launch.py            # GUI
ros2 launch maze_simulation maze_simulation.launch.py gui:=false # headless (VM / training)
ros2 run maze_simulation check_sim_topics.py --duration 8
python3 ros2_ws/src/maze_simulation/scripts/maze_tools.py check   # ASCII maze + goal reference paths (no ROS)
```
If the world is edited, regenerate the ground-truth map: `python3 .../maze_tools.py map ros2_ws/src/slam_mapping/maps/maze_groundtruth`.

## Tests
- VERIFIED (static): `tools/verify_static.py` - layout is read from the SDF, the spawn clears every wall, the committed ground-truth map matches every wall of the SDF (`slam_mapping/test/test_groundtruth_vs_sdf.py`).
- VERIFIED (Ubuntu VM, 2026-10-17, `docs/results/vm/sim_20261017-101901.log`): world loads, exactly one robot `burger`, topic contract 10 of 10 (`/scan` 5.0 Hz, `/odom` 49.6 Hz, `/clock`, TF, `/cmd_vel` is `TwistStamped`), headless run. Control checks 22 of 22 (`control_20261017-102106.log`).
- NOT a match: the maze is 7 x 6, Figure 1 is 6 x 6.

## Limitations / risks to check on the VM
- The TurtleBot3 model path, bridge names and `/cmd_vel` TwistStamped type follow the Jazzy `turtlebot3_gazebo` package; confirmed on the VM by the topic contract check, re-check with `ros2 topic info /cmd_vel`.
- Walls are 0.2 m high and the LiDAR sits at about 0.18 m, so a small pitch can make a beam miss a wall top; this did not break the runs on the VM but is fragile.
- Wall visuals are plain boxes; the brief's YouTube reference was not consulted (the figure was used). Start pose and the three goals are *our choice* - the brief does not specify them.
- Headless LiDAR needs an EGL/software renderer; if `--headless-rendering` fails on the VM, run `gui:=true` or `export LIBGL_ALWAYS_SOFTWARE=1`.

## VIVA
*Why read the maze from the SDF?* The world file is the single source: `maze_tools.py` derives the ground-truth map and the reference path lengths from it, so they cannot disagree (a test compares the map with every SDF wall). *Why 0.01 m walls / 0.5 m cells?* The 0.5 m cell size matches Figure 1; the layout does not (7 x 6 against 6 x 6); the corridors are 0.5 m wide, which drove the Nav2 inflation tuning (see Member 3). *Why TwistStamped?* The Jazzy TurtleBot3 bridge uses it; Nav2 (Jazzy) emits Twist, so Member 3 inserts a `twist_stamper`.
