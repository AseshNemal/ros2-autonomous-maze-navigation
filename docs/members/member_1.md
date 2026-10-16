# Member 1 - Gazebo Simulation (`maze_simulation`)

Status key: **VERIFIED** (actually run and passed) · **IMPLEMENTED** (code exists, runtime check pending) · **FAILED** · **BLOCKED** · **NOT IMPLEMENTED**.
No Gazebo/Nav2/SLAM/DQN runtime test has been run from this branch yet; only the static checks in `tools/verify_static.py` (Mac) have. See [testing.md](../testing.md).

## Responsibility
Custom Gazebo Harmonic maze from the assignment's Figure 1, TurtleBot3 Burger spawn, LiDAR/odometry bridging, simulation launch, consistent frames and naming.

## Provenance
**No original Member 1 commits exist on any branch.** Everything here is *emergency integration code* by the branch owner (AI-assisted), commit `82efae1` (`feat(simulation)`). It must not be presented as a teammate's original work. If Member 1 delivers, review and merge theirs separately and keep whatever is better.

## Contribution
- `config/maze_layout.yaml` - maze transcribed from Figure 1: 6x6 cells of 0.5 m (3 m x 3 m). Walls were *measured from the figure's pixel line thickness* (thick = wall); the ASCII render in `maze_tools.py check` matches the figure.
- `scripts/maze_tools.py` - single source of truth: generates `worlds/maze.world`, a ground-truth map, grid A* reference path lengths, ASCII preview.
- `worlds/maze.world` - generated SDF (29 wall models, outer 3x3 m boundary, 0.05 m thick, 0.5 m high). World name `maze`.
- `launch/maze_simulation.launch.py` - gz sim (GUI or headless `-s --headless-rendering`), robot_state_publisher, `ros_gz_sim create` of the TurtleBot3 Burger model, `ros_gz_bridge`.
- `config/bridge.yaml` - bridge for `/clock /odom /tf /joint_states /scan /imu /cmd_vel`.
- `scripts/check_sim_topics.py` - PASS/FAIL contract check (rates, frames, TF, `/cmd_vel` type).

## Architecture
```
maze_layout.yaml --maze_tools.py--> maze.world --gz sim--> TB3 Burger (turtlebot3_gazebo model.sdf)
                                                    |  gz topics
                                              ros_gz_bridge (bridge.yaml)
                              /scan /odom /tf /clock /imu /joint_states  <--  --> /cmd_vel (TwistStamped)
```
Robot model and URDF come from the apt `turtlebot3_gazebo` package (not copied). Entity name: `turtlebot3_burger`. Frames: `odom -> base_footprint -> base_link -> base_scan`. Spawn: cell (5,0) = world (-1.25, -1.25), yaw 0.

## Interface (contract for the other members)
| Topic | Type | Direction |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | sim -> all |
| `/odom` | `nav_msgs/Odometry` (`odom`->`base_footprint`) | sim -> all |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | sim/RSP -> all |
| `/clock` | `rosgraph_msgs/Clock` | sim -> all (`use_sim_time:=true`) |
| `/cmd_vel` | **`geometry_msgs/TwistStamped`** | controllers -> sim |
Reset service used by DQN/eval: `gz service -s /world/maze/set_pose` (needs the UserCommands plugin, present in the world).

## Run
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch maze_simulation maze_simulation.launch.py            # GUI
ros2 launch maze_simulation maze_simulation.launch.py gui:=false # headless (VM / training)
ros2 run maze_simulation check_sim_topics.py --duration 8
python3 ros2_ws/src/maze_simulation/scripts/maze_tools.py check   # ASCII maze + goal reference paths (no ROS)
```
Regenerate the world after editing the layout: `python3 .../maze_tools.py world .../worlds/maze.world`.

## Tests
- IMPLEMENTED/VERIFIED (static): `tools/verify_static.py` - world XML parses and equals regeneration from the layout.
- UNVERIFIED: world loads in Gazebo Harmonic; robot spawns; topic contract (`check_sim_topics.py`); sensor rates; headless rendering on the ARM64 VM.

## Limitations / risks to check on the VM
- The TurtleBot3 model path, bridge names and `/cmd_vel` TwistStamped type follow the Jazzy `turtlebot3_gazebo` package as the DQN trial saw it; re-check with `ros2 topic info /cmd_vel`.
- Wall visuals are plain boxes; the brief's YouTube reference was not consulted (the figure was used). Start pose and the three goals are *our choice* - the brief does not specify them.
- Headless LiDAR needs an EGL/software renderer; if `--headless-rendering` fails on the VM, run `gui:=true` or `export LIBGL_ALWAYS_SOFTWARE=1`.

## VIVA
*Why generate the world?* One YAML drives the world, the ground-truth map and the reference path lengths, so they cannot disagree. *Why 0.05 m walls / 0.5 m cells?* Matches Figure 1; free corridor width is 0.45 m, which drove the Nav2 inflation tuning (see Member 3). *Why TwistStamped?* The Jazzy TurtleBot3 bridge uses it; Nav2 (Jazzy) emits Twist, so Member 3 inserts a `twist_stamper`.
