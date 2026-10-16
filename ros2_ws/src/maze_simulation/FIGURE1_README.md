# Figure 1 maze simulation (ROS 2 Jazzy / Gazebo Harmonic)

The reference layout remains an approximation; compare it visually with the assignment figure.
`generate_maze.py` is the source of truth for `worlds/custom_maze.sdf`.
The 7 × 6 grid uses 0.5 m cells, 0.2 m high walls and 0.01 m thick walls.
Grid `(gx, gy)` maps to world `((gx-1)*0.5, (5-gy)*0.5)` metres.
Outer wall centre lines span x=[-0.5, 3.0], y=[-0.5, 2.5]: 3.5 × 3.0 m.
The bottom horizontal wall already existed, from (-0.5,-0.5) to (3,-0.5).
The added wall_16 closes the missing left edge from (-0.5,-0.5) to (-0.5,2.5),
with centre (-0.5,1,0.1) and size (0.01,3,0.2). All 15 original walls remain.
The 4.5 × 4.0 m ground has its top at z=0 and extends 0.5 m beyond each edge.

Burger spawns at **(0.25, 0.0, 0.01) m, yaw 0**, facing +X in the bottom corridor.
The nearest wall faces are y=±0.495, giving 0.495 m centre clearance; the
left wall face is 0.745 m away. A conservative 0.15 m robot footprint radius
leaves at least 0.345 m clearance. The stock model's wheel bottom is z=-0.01
relative to its origin, so the 0.01 m spawn height places it on the ground.
The genuine installed `turtlebot3_gazebo/models/turtlebot3_burger/model.sdf`
is spawned once using `ros_gz_sim create -world custom_maze -name burger
-allow_renaming false`. The create executable waits for the world create service;
there is no fixed delay or second spawner. Model mesh resources are added to
`GZ_SIM_RESOURCE_PATH`. A robot state publisher supplies the URDF link transforms.
The world enables Physics, UserCommands, SceneBroadcaster, Sensors and Imu systems.

## Build and launch

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
python3 ros2_ws/src/maze_simulation/generate_maze.py
gz sdf -k ros2_ws/src/maze_simulation/worlds/custom_maze.sdf
cd ros2_ws
colcon build --symlink-install --packages-select maze_simulation
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch maze_simulation custom_maze.launch.py
```

Launch deliberately selects Burger. Close other simulations first because the
stock robot uses global topics. Do not also run the stock TurtleBot3 spawner.
To load just the maze, without ROS or a robot:

```bash
gz sim -r "$(ros2 pkg prefix --share maze_simulation)/worlds/custom_maze.sdf"
```

For a headless server with GPU LiDAR rendering:

```bash
ros2 launch maze_simulation custom_maze.launch.py gz_args:="-s -r --headless-rendering $(ros2 pkg prefix --share maze_simulation)/worlds/custom_maze.sdf"
```

## ROS interface

The launch reuses the installed Jazzy `turtlebot3_burger_bridge.yaml`.

| ROS topic | ROS type | Direction / Gazebo type |
|---|---|---|
| /clock | rosgraph_msgs/msg/Clock | GZ → ROS / gz.msgs.Clock |
| /scan | sensor_msgs/msg/LaserScan | GZ → ROS / gz.msgs.LaserScan |
| /odom | nav_msgs/msg/Odometry | GZ → ROS / gz.msgs.Odometry |
| /cmd_vel | geometry_msgs/msg/TwistStamped | ROS → GZ / gz.msgs.Twist |
| /tf | tf2_msgs/msg/TFMessage | GZ → ROS / gz.msgs.Pose_V |
| /imu | sensor_msgs/msg/Imu | GZ → ROS / gz.msgs.IMU |
| /joint_states | sensor_msgs/msg/JointState | GZ → ROS / gz.msgs.Model |

Gazebo topic names match the ROS names. `/tf` carries odom → base_footprint;
robot_state_publisher adds link transforms on `/tf` and `/tf_static`.
SLAM Toolbox, Nav2 and DQN nodes should use simulation time and the appropriate
footprint/inflation for 0.5 m corridors. Twist-only producers need an adapter to
TwistStamped; preserve this interface when integrating.

## Runtime acceptance checks

In another terminal, source ROS and the workspace as above. With the full launch
running, open Gazebo's Entity Tree and confirm exactly one `burger`, plus
`ground_plane` and `wall_01` through `wall_16`. Expand Burger to inspect its links.
The following also queries the scene independently of the GUI:

```bash
gz service -s /world/custom_maze/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 5000 --req ''
ros2 topic list -t
ros2 topic info /cmd_vel -v
timeout 15s ros2 topic echo /clock --once
timeout 15s ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --qos-reliability best_effort
timeout 15s ros2 topic echo /odom nav_msgs/msg/Odometry --once
timeout 15s ros2 topic echo /imu --once --qos-reliability best_effort
timeout 15s ros2 topic echo /joint_states --once
timeout 15s ros2 topic echo /tf --once
ros2 run tf2_ros tf2_echo odom base_scan
```

Check scan ranges contain finite distances to walls and timestamps advance;
topic existence alone does not prove sensor operation. At the fresh spawn,
record odometry, send this short straight command, and explicitly stop:

```bash
ros2 topic echo /odom nav_msgs/msg/Odometry --once
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped '{header: auto, twist: {linear: {x: 0.05}, angular: {z: 0.0}}}' --rate 10 --times 20
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped '{header: auto, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}' --once
ros2 topic echo /odom nav_msgs/msg/Odometry --once
```

Expect roughly 0.1 m travel along +X, changing odometry, and visible motion
without wall contact. Run this only from the fresh spawn with no other command
publisher. Inspect the closed perimeter and corridors in the GUI as well.

## Validation in the editing environment

Gazebo's `gz sdf -k` reports Valid and the isolated Jazzy colcon package build
passes. Python/XML and geometry preservation checks pass. Runtime results are
pending: the attempted headless launch hit sandbox network/socket permission
errors (getifaddrs and UDP transport), and Gazebo could not start successfully.
Robot visibility, sensor messages and movement have not been verified here.
Run the acceptance commands above on the target machine.
