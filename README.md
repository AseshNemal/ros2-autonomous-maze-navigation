# ROS 2 Autonomous Maze Navigation

**IE4060 Robotics and Intelligent Systems — Assignment 02**

An academic robotics simulation project intended to combine **ROS 2 Jazzy**, **Gazebo**, **SLAM**, **Nav2**, classical path planning, and **Deep Q-Learning (DQN)** for autonomous navigation in a maze.



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
│       ├── nav2_navigation/   # Member 3 (ament_cmake)
│       └── dqn_navigation/    # Member 4 (ament_python)
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

The root-level `worlds/`, `maps/`, `config/`, `launch/` and `dqn/` folders are leftovers from the initial scaffold (they contain only placeholder READMEs). Package assets belong inside the packages above; these folders are slated for removal once the group agrees.

## Team division

| Member | Package | Scope | Guide |
|---|---|---|---|
| 1 | `maze_simulation` | Gazebo maze world, TurtleBot3 Burger, LiDAR, simulation launch | [docs/member_1_environment.md](docs/member_1_environment.md) |
| 2 | `slam_mapping` | SLAM Toolbox, occupancy-grid mapping, map saving, localization | [docs/member_2_slam.md](docs/member_2_slam.md) |
| 3 | `nav2_navigation` | Nav2, A*/Dijkstra planning, obstacle avoidance, RViz goals, 3-goal demo | [docs/member_3_navigation.md](docs/member_3_navigation.md) |
| 4 | `dqn_navigation` | DQN design, training/evaluation, comparison with Nav2 baseline | [docs/member_4_dqn.md](docs/member_4_dqn.md) |

Integration plan and shared interface: [docs/integration.md](docs/integration.md).

## Target environment

- Ubuntu 24.04 (ARM64 or AMD64, depending on host)
- ROS 2 Jazzy
- Gazebo Harmonic and compatible ROS-Gazebo packages
- TurtleBot3 Burger, Nav2, SLAM Toolbox and compatible packages (versions to be pinned when selected)
- Python 3 for DQN experiments; Python-only packages go in `requirements.txt` (currently empty)

**Note:** Installation and platform compatibility have not yet been verified for this repository. Gazebo, ROS 2, Nav2 and PyTorch support can vary by platform and package source.

## Getting started (after implementation exists)

Install and configure the chosen ROS 2 distribution before following these steps.

```bash
# From the repository root:
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
# or build one member's package only:
# colcon build --symlink-install --packages-select maze_simulation
source install/setup.bash
```

The four packages are valid, buildable scaffolds, but **contain no functionality yet**: there is no world, launch file, map, Nav2 configuration or DQN code. Launch and training commands will be added as each member implements their part.

## Implementation checklist

- [ ] Member 1: robot model, LiDAR and custom Gazebo maze world
- [ ] Member 2: mapping and saved SLAM map
- [ ] Member 3: Nav2 configuration, A*/Dijkstra planning and goal navigation
- [ ] Member 4: DQN environment, network, training, evaluation and Nav2 comparison
- [ ] Reproducible three-goal demonstration
- [ ] Evidence, screenshots, results and final report

## Repository hygiene

Do not commit generated ROS build files, credentials, large training checkpoints, or rosbag captures. Use Git LFS or external storage if large binary artifacts genuinely need versioning. Include only real simulation and test evidence in reports.
