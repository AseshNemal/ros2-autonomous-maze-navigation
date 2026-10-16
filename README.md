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

> **Branch status:** `feature/group-integration` is an emergency full-project backup. It is code-complete but **runtime-unverified in Gazebo** - read [docs/testing.md](docs/testing.md) for the honest status of each requirement.

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
