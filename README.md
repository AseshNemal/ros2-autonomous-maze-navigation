# ROS 2 Autonomous Maze Navigation

**IE4060 Robotics and Intelligent Systems — Assignment 02**

An academic robotics simulation project intended to combine **ROS 2 Jazzy**, **Gazebo**, **SLAM**, **Nav2**, classical path planning, and **Deep Q-Learning (DQN)** for autonomous navigation in a maze.



## How to run (Ubuntu 24.04 + ROS 2 Jazzy)

Open a terminal in the repo folder. There are two ways to run: an automatic test, and a live demo you can watch.

### A. Automatic test (one command, no window)

```bash
bash tools/run_all.sh
```

- On the first run it installs missing ROS packages (asks for your sudo password once).
- It then runs, in order: build + tests, simulation check, SLAM mapping, Nav2 (3 goals), DQN, and prints a PASS/FAIL summary.
- It is **headless**: no Gazebo or RViz window opens, so you will only see text. That is normal.
- Takes 30-60 minutes. Do not start another Gazebo while it runs (the VM is small).
- Logs and CSV files are saved in `docs/results/vm/`.
- Shorter test: `EPISODES=50 bash tools/run_all.sh`
- One stage only: `bash tools/verify_on_vm.sh <build|sim|slam|nav|dqn>`

### B. Live demo (see the robot, click a goal)

**One command** (VM desktop): `bash tools/demo.sh` - builds, opens Gazebo + RViz. In RViz press **2D Goal Pose** and click the map. `Ctrl+C` stops all. Variants: `bash tools/demo.sh slam` (build the map live), `bash tools/demo.sh teleop` (drive with keyboard).

Or step by step:

Use the VM's desktop (not SSH). Run this in **every** new terminal first:

```bash
cd ~/ros2-autonomous-maze-navigation/ros2_ws && colcon build --symlink-install   # only once, or after code changes
source /opt/ros/jazzy/setup.bash && source ~/ros2-autonomous-maze-navigation/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

**Terminal 1 - Gazebo (maze + robot window):**
```bash
ros2 launch maze_simulation maze_simulation.launch.py
```

**Terminal 2 - Navigation (RViz window with the map):**
```bash
ros2 launch maze_navigation navigation.launch.py \
  map:=$HOME/ros2-autonomous-maze-navigation/ros2_ws/src/slam_mapping/maps/maze_groundtruth.yaml \
  initial_x:=-1.25 initial_y:=-1.25
```

**Window layout:** press **Super+Left** on the Gazebo window and **Super+Right** on RViz to tile them side by side (see [docs/gui_review.md](docs/gui_review.md)).

**Click a goal:** in RViz press **2D Goal Pose** (top toolbar), then click on the map where you want the robot to go and drag to set the direction. The robot drives there by itself. Repeat for 3 different points (this is the VIVA demo).

**SLAM demo instead of navigation** (builds the map live): in Terminal 2 run `ros2 launch slam_mapping mapping.launch.py`, then in a Terminal 3 run `python3 ros2_ws/src/slam_mapping/scripts/tour_map_drive.py`. Save the map with `ros2 run slam_mapping save_map.sh maze_slam`.

**Alternative world (optional, prototype):** `ros2 launch maze_simulation maze_simulation.launch.py world:=custom_maze x_pose:=0.25 y_pose:=0.0` loads JayangaWeerasinghe's extra arena `worlds/custom_maze.sdf` (a 7 x 6 grid approximation; **not** Figure 1, which is 6 x 6, and not used for any reported result). `world` accepts a name in `worlds/` or a file path; the default is the assignment maze, and the robot is spawned into the world named inside the file. His own launch file also works: `ros2 launch maze_simulation custom_maze.launch.py`. The Nav2 / SLAM / DQN stages, the `tools/` scripts and the ground-truth map assume the default maze (`/world/maze/...`, start pose -1.25, -1.25), so use the alternative world for viewing and manual driving only. Not yet run in Gazebo.

**Rules:** run only ONE of Nav2 / SLAM mapping / DQN at a time (they all control the same robot), and close Gazebo (`Ctrl+C`) before starting something new.

### Problems

| Problem | Fix |
|---|---|
| `ros2: error ... invalid choice: 'run'` | `bash tools/install_deps_ubuntu.sh` |
| No window opens | You used method A (headless) or SSH. Use method B on the VM desktop. |
| Gazebo very slow / VM freezes | Give the VM more RAM (6 GB+), close other programs. |
| `ros2: command not found` / `package not found` | New terminal has no ROS. Run the 3 setup lines above, or add them once to `~/.bashrc` (`source /opt/ros/jazzy/setup.bash`, `source ~/ros2-autonomous-maze-navigation/ros2_ws/install/setup.bash`, `export TURTLEBOT3_MODEL=burger`). |
| Robot does not move | Check Gazebo is running and you only have one Nav2/SLAM/DQN running. |

More detail: [docs/demonstration.md](docs/demonstration.md), [docs/installation.md](docs/installation.md), [docs/testing.md](docs/testing.md).

## Planned capabilities

- Build and simulate a maze world in Gazebo.
- Simulate a mobile robot with a LiDAR sensor.
- Generate and save a map using SLAM.
- Navigate to user-selected goals with Nav2.
- Experiment with A* / Dijkstra path planning (subject to the assignment's required implementation details).
- Implement and evaluate DQN-based navigation.
- Document tests and a multi-goal demonstration for the viva.

## Repository structure

```text
ros2-autonomous-maze-navigation/
├── ros2_ws/
│   └── src/
│       ├── maze_simulation/   # Member 1 (ament_cmake)
│       ├── slam_mapping/      # Member 2 (ament_cmake)
│       ├── maze_navigation/   # Member 3 (ament_cmake)
│       └── dqn_agent/    # Member 4 (ament_python)
├── docs/
│   ├── member_1_environment.md
│   ├── member_2_slam.md
│   ├── member_3_navigation.md
│   ├── member_4_dqn.md
│   ├── integration.md
│   ├── results/
│   └── screenshots/
├── README.md
├── .gitignore
└── requirements.txt
```

The root-level `worlds/`, `config/`, `launch/` and `dqn/` folders are leftovers from the initial scaffold (placeholder READMEs); root `maps/` holds the teammate's prototype `test_map` (not the final maze). Package assets belong inside the packages above; these folders are slated for removal once the group agrees.

## Team division (see [docs/OWNERSHIP.md](docs/OWNERSHIP.md) for who wrote what)

| Member | Package | Scope | Guide |
|---|---|---|---|
| 1 | `maze_simulation` | Gazebo maze from Figure 1, TurtleBot3 Burger, bridge, launch | [docs/members/member_1.md](docs/members/member_1.md) |
| 2 | `slam_mapping` | SLAM Toolbox, map save/validate, AMCL localization | [docs/members/member_2.md](docs/members/member_2.md) |
| 3 | `maze_navigation` | Nav2 (A* + MPPI), RViz goals, 3-goal metrics | [docs/members/member_3.md](docs/members/member_3.md) |
| 4 | `dqn_agent` | Double DQN training/evaluation, comparison with Nav2 | [docs/members/member_4.md](docs/members/member_4.md) |

Also: [architecture](docs/architecture.md) · [integration](docs/integration.md) · [installation](docs/installation.md) · [testing & status](docs/testing.md) · [demonstration](docs/demonstration.md) · [VIVA notes](docs/viva.md) · [report draft](docs/report_draft.md).

> **Branch status:** `dev-integration` is an emergency full-project backup. It is code-complete but **runtime-unverified in Gazebo** - read [docs/testing.md](docs/testing.md) for the honest status of each requirement.

## Target environment

- Ubuntu 24.04 (ARM64 or AMD64, depending on host)
- ROS 2 Jazzy
- Gazebo Harmonic and compatible ROS-Gazebo packages
- TurtleBot3 Burger, Nav2, SLAM Toolbox and compatible packages (versions to be pinned when selected)
- Python 3 for DQN experiments; Python-only packages go in `requirements.txt` (currently empty)

**Note:** Installation and platform compatibility have not yet been verified for this repository. Gazebo, ROS 2, Nav2 and PyTorch support can vary by platform and package source.

## Getting started

```bash
tools/install_deps_ubuntu.sh                       # once, on the Ubuntu VM
source /opt/ros/jazzy/setup.bash && export TURTLEBOT3_MODEL=burger
cd ros2_ws && colcon build --symlink-install --parallel-workers 1 && source install/setup.bash
ros2 launch maze_simulation maze_simulation.launch.py       # maze + robot
ros2 launch maze_navigation navigation.launch.py            # Nav2 (click goals in RViz)
python3 ../tools/verify_static.py                            # no-ROS consistency checks (any machine)
```
One controller at a time on `/cmd_vel`: mapping, Nav2 or DQN - see [docs/architecture.md](docs/architecture.md).

## Implementation checklist (honest)

- [x] Member 1: maze world + spawn launch (IMPLEMENTED; Gazebo run pending)
- [x] Member 2: SLAM/localization workflow + map validator (IMPLEMENTED; real SLAM map pending)
- [x] Member 3: Nav2 config fixes + 3-goal runner (IMPLEMENTED; no navigation result yet)
- [x] Member 4: DQN maze adaptation (IMPLEMENTED; unit-tested; no goal-reaching evidence)
- [ ] Reproducible three-goal demonstration (needs VM run)
- [ ] Evidence, results and final report numbers

## Repository hygiene

Do not commit generated ROS build files, credentials, large training checkpoints, or rosbag captures. Use Git LFS or external storage if large binary artifacts genuinely need versioning. Include only real simulation and test evidence in reports.
