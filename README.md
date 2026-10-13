# ROS 2 Autonomous Maze Navigation

**Robotics Simulation — Assignment 02**

An academic robotics simulation project intended to combine **ROS 2 Jazzy**, **Gazebo**, **SLAM**, **Nav2**, classical path planning, and **Deep Q-Learning (DQN)** for autonomous navigation in a maze.



## Planned capabilities

- Build and simulate a maze world in Gazebo.
- Simulate a mobile robot with a LiDAR sensor.
- Generate and save a map using SLAM.
- Navigate to user-selected goals with Nav2.
- Experiment with A* / Dijkstra path planning (subject to the assignment's required implementation details).
- Implement and evaluate DQN-based navigation.
- Document tests and a multi-goal demonstration for the viva.

## Proposed layout

```text
ros2-autonomous-maze-navigation/
├── ros2_ws/
│   └── src/           # ROS 2 packages (to be implemented)
├── worlds/            # Gazebo maze world assets
├── maps/              # Saved SLAM maps
├── config/            # ROS 2, Nav2, SLAM and robot configuration
├── launch/            # Top-level ROS 2 launch orchestration, if needed
├── dqn/               # Reinforcement learning training/evaluation code
├── docs/              # Report, test evidence and demonstration notes
├── requirements.txt   # Python-only dependencies, as required
└── .gitignore
```

For ROS 2 packages, put launch/configuration files inside their respective packages where appropriate. The root-level folders are optional shared assets; avoid maintaining duplicate copies.

## Target environment

- Ubuntu 24.04 (ARM64 or AMD64, depending on host)
- ROS 2 Jazzy
- Gazebo Harmonic and compatible ROS-Gazebo packages
- Nav2, SLAM Toolbox and compatible robot packages (versions to be pinned when selected)
- Python 3 for DQN experiments

**Note:** Installation and platform compatibility have not yet been verified for this repository. Gazebo, ROS 2, Nav2 and PyTorch support can vary by platform and package source.

## Getting started (after implementation exists)

Install and configure the chosen ROS 2 distribution before following these steps.

```bash
# From the repository root:
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

There are **no runnable ROS 2 packages yet**; commands for launching simulation and training DQN will be added when those modules exist.

## Implementation checklist

- [ ] Robot model and LiDAR simulation
- [ ] Custom Gazebo maze world
- [ ] Mapping and saved SLAM map
- [ ] Nav2 configuration and goal navigation
- [ ] A* or Dijkstra planner integration, as required
- [ ] DQN environment, network, training, and evaluation
- [ ] Reproducible three-goal demonstration
- [ ] Evidence, screenshots, results and final report

## Repository hygiene

Do not commit generated ROS build files, credentials, large training checkpoints, or rosbag captures. Use Git LFS or external storage if large binary artifacts genuinely need versioning. Include only real simulation and test evidence in reports.
