# ROS 2 Autonomous Maze Navigation

**IE4060 Robotics and Intelligent Systems — Assignment 02**

An academic robotics simulation project that combines **ROS 2 Jazzy**, **Gazebo**, **SLAM**, **Nav2**, classical path planning, and **Deep Q-Learning (DQN)** for autonomous navigation in a maze.



## How to run (Ubuntu 24.04 + ROS 2 Jazzy)

Open a terminal in the repo folder. There are two ways to run: an automatic test, and a live demo you can watch.

### A. Automatic tests (no window), one stage at a time

```bash
python3 tools/verify_static.py                       # no ROS, ~10 s
bash tools/verify_on_vm.sh sim                       # ~0.5 min: world, one robot, /scan /odom /tf /clock /cmd_vel
bash tools/verify_on_vm.sh control                   # ~1.5 min: teleop, watchdog, Nav2 goal
bash tools/verify_on_vm.sh nav                       # ~2 min: Nav2, 3 goals -> docs/results/vm/nav2_results_*.csv
bash tools/verify_on_vm.sh slam                      # ~7 min: coverage tour + SLAM + map validation
QUICK=1 bash tools/verify_on_vm.sh dqn               # ~2 min smoke test (10 episodes); real training: EPISODES=50 bash tools/verify_on_vm.sh dqn
STAGES="sim nav" bash tools/run_all.sh               # chosen stages with a timing summary
```

- Every stage prints a heartbeat every 30 s and its total time; follow details with `tail -f docs/results/vm/<stage>_*.log`.
- Stages are headless (no window). Do not start another Gazebo while one runs. Logs and CSV files go to `docs/results/vm/`.
- Without arguments `bash tools/run_all.sh` runs every stage (long: DQN training dominates).

### The two commands to remember

```bash
bash tools/manual.sh     # REMOTE / MANUAL control: Gazebo + keyboard (keep the terminal focused: i , j l k, z = slower)
bash tools/auto.sh       # AUTOMATIC: Gazebo + RViz + Nav2 -> in RViz press "Nav2 Goal", click the map, drag
```
**In RViz use the "Nav2 Goal" button (not "2D Pose Estimate": that moves where Nav2 believes the robot is, RViz and Gazebo then show different places and goals abort with many recoveries - fix with `bash tools/reset_robot.sh`).** Clicking in the Gazebo window does nothing (it only selects objects); the robot is driven by the keyboard in `manual.sh` or by RViz goals in `auto.sh`. Close one before starting the other (both drive `/cmd_vel`).

### Mouse or keyboard not working over remote desktop? Use commands

```bash
bash tools/demo.sh                    # start Gazebo + RViz + Nav2 (leave running in terminal 1)
bash tools/send_goal.sh G1            # terminal 2: send goal 1 / G2 / G3 (maze_simulation/config/mission.yaml), or: send_goal.sh 1.25 0.75
bash tools/demo.sh teleop             # alternative: keyboard driving (terminal must be focused); stop Nav2 demo first
bash tools/drive.sh forward 2         # alternative: timed driving (forward|back|left|right|stop); needs only the simulation
                                      # (ros2 launch maze_simulation maze_simulation.launch.py), NOT Nav2 at the same time
```

### B. Live demo (see the robot, click a goal)

**One command** (VM desktop): `bash tools/demo.sh` - builds, opens Gazebo + RViz. In RViz press **Nav2 Goal** and click the map. `Ctrl+C` stops all. Variants: `bash tools/demo.sh slam` (build the map live), `bash tools/demo.sh teleop` (drive with the keyboard, see "Drive with the keyboard" below). The default mode (Nav2) does **not** accept keyboard input; it only takes clicked goals.

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
  initial_x:=0.25 initial_y:=0.0
```

**Window layout:** press **Super+Left** on the Gazebo window and **Super+Right** on RViz to tile them side by side (see [docs/gui_review.md](docs/gui_review.md)).

**Click a goal:** in RViz press **Nav2 Goal** (top toolbar), then click on the map where you want the robot to go and drag to set the direction. The robot drives there by itself. Repeat for 3 different points (this is the VIVA demo).

**Drive with the keyboard** (instead of clicking goals): stop any running demo (`Ctrl+C`), then in a terminal on the VM desktop run

```bash
cd ~/ros2-autonomous-maze-navigation
bash tools/demo.sh teleop
```

Gazebo opens and `teleop_twist_keyboard` starts in that same terminal. Keep **that terminal focused** (keys pressed in the Gazebo or RViz window do nothing) and use:

| Key | Action | Key | Action |
|---|---|---|---|
| `i` | forward | `,` | backward |
| `j` | turn left (in place) | `l` | turn right (in place) |
| `u` | forward + left | `o` | forward + right |
| `m` | backward + left | `.` | backward + right |
| `k` or space | stop | `q` / `z` | speed up / slow down (all) |
| `w` / `x` | speed up / slow down (forward speed only) | `e` / `c` | speed up / slow down (turn speed only) |

The default speed is fast for the 0.5 m corridors: press `z` a few times before driving. `Ctrl+C` quits and stops the simulation. Do not run teleop together with Nav2, SLAM mapping or DQN (all publish `/cmd_vel`).

**SLAM demo instead of navigation** (builds the map live): in Terminal 2 run `ros2 launch slam_mapping mapping.launch.py`, then in a Terminal 3 run `python3 ros2_ws/src/slam_mapping/scripts/tour_map_drive.py`. Save the map with `ros2 run slam_mapping save_map.sh maze_slam`.

**Maze:** the only world is `ros2_ws/src/maze_simulation/worlds/custom_maze.sdf` (unmodified); the planners, ground-truth map and goal cells are read from that file (`maze_tools.py`) plus `ros2_ws/src/maze_simulation/config/mission.yaml` (start and goals); see `ros2_ws/src/maze_simulation/FIGURE1_README.md`: 7 x 6 cells, 3.5 m x 3.0 m, spawn (0.25, 0.0). Its Gazebo world name is `custom_maze` (`/world/custom_maze/...`); `tools/demo.sh` uses `custom_maze.launch.py`; `maze_simulation.launch.py` (adds the headless option and a cmd_vel watchdog) loads the same world. **Note:** the assignment's Figure 1 is a 6 x 6 grid; the 7 x 6 maze is an approximation of it (see report section 2). The earlier 6 x 6 results and the cloud checks were measured on the previous layout.

**Rules:** run only ONE of Nav2 / SLAM mapping / DQN at a time (they all control the same robot), and close Gazebo (`Ctrl+C`) before starting something new.

### First time on a fresh Ubuntu 24.04 machine (three commands)
ROS 2 Jazzy must be installed first ([docs/installation.md](docs/installation.md)); git does not install it. Committing the trained model removes the need to *train* again, not the need to install ROS 2, Gazebo and the dependencies.
```bash
git clone https://github.com/AseshNemal/ros2-autonomous-maze-navigation.git
cd ros2-autonomous-maze-navigation
bash tools/install_deps_ubuntu.sh   # once: Gazebo bridge, TurtleBot3, Nav2, SLAM Toolbox, RViz (needs sudo + internet)
bash tools/setup.sh                 # 1. build the workspace and check the prerequisites and the DQN model
bash tools/auto.sh                  # 2. Gazebo window + RViz + Nav2: press "Nav2 Goal" in RViz and click the maze
# stop it with Ctrl+C, then (never both at once - they drive the same robot; each command refuses if the other is running):
bash tools/run_dqn_model.sh         # 3. the trained DQN model in the Gazebo window, NO training
```
`HEADLESS=1 bash tools/run_dqn_model.sh` runs without the Gazebo window and `EPISODES=1` gives one trial per goal (default 3). Results are new files in `~/dqn_eval_results/` (`dqn_eval_<time>.csv` and `.log`); nothing in the repository is overwritten. No hard-coded user name or home directory is used; run the commands from any clone location.

### The trained DQN model (reproducibility only - it does NOT reach the goals)
| | |
|---|---|
| File | `ros2_ws/src/dqn_agent/models/dqn_maze_none_60ep_0of9.npz` (24,488 bytes) |
| SHA-256 | `d75aa1a533809e8e7075de96058ac58d6026df4afacfdb3ff8c899956b0feaf2` |
| Network | 15 inputs (12 LiDAR sectors, goal distance, bearing sin/cos), 5 actions, hidden 64, Double DQN, NumPy |
| Training | pure DQN (`GUIDANCE=none`), 60 episodes in the 7 x 6 maze, 1,587 transitions, 1,488 optimizer updates, final epsilon 0.497 (`docs/results/vm/dqn_train_none_20261017-165910.csv`, `dqn_20261017-165910.log`; 10-episode smoke test `...165336...`) |
| Authoritative evaluation | **0 of 9 goals reached** (7 collisions, 2 timeouts), success confirmed with Gazebo's true robot pose (`dqn_eval_none_truthconfirmed_20261017-172400.csv`, `dqn_eval_truthconfirmed_20261017-172400.log`) |
| Invalid earlier result | `dqn_eval_none_20261017-165910.csv` (5 of 9) used wheel odometry for success, which drifts; it is kept only as evidence of that artefact (`dqn_odometry_vs_truth_20261017.txt`) |

- The model was **trained but did not achieve successful goal navigation**; the 0 of 9 result must not be presented as success. The checkpoint exists so the evaluation can be reproduced without retraining (`bash tools/run_dqn_model.sh`), not as a working navigation policy.
- The training run used the odometry-based success signal (before the ground-truth correction); the final evaluation used the corrected verification.
- Improving it would need more training, which has not been run. The checkpoint stores the weights, step counters and hyperparameters, **not** the replay buffer, Adam optimizer state or random-number state, so it cannot continue the original training session exactly (`--resume` restarts with an empty replay buffer).
- Provenance: written at the end of the 60-episode run (17:15, 2026-10-17); its counters (1,488 updates, 1,587 transitions, epsilon 0.4974) equal the training CSV, and the truth-confirmed evaluation started afterwards from this path. No file hash was recorded at evaluation time.

### Problems

| Problem | Fix |
|---|---|
| `ros2: error ... invalid choice: 'run'` | `bash tools/install_deps_ubuntu.sh` |
| No window opens | You used method A (headless) or SSH. Use method B on the VM desktop. |
| Gazebo very slow / VM freezes | Give the VM more RAM (6 GB+), close other programs. |
| `ros2: command not found` / `package not found` | New terminal has no ROS. Run the 3 setup lines above, or add them once to `~/.bashrc` (`source /opt/ros/jazzy/setup.bash`, `source ~/ros2-autonomous-maze-navigation/ros2_ws/install/setup.bash`, `export TURTLEBOT3_MODEL=burger`). |
| Robot does not move | Check Gazebo is running and you only have one Nav2/SLAM/DQN running. |

More detail: [docs/demonstration.md](docs/demonstration.md), [docs/installation.md](docs/installation.md), [docs/testing.md](docs/testing.md).

## Capabilities

- Maze world in Gazebo with a TurtleBot3 Burger and LiDAR.
- SLAM Toolbox mapping (map precision 0.819, below our own 0.85 target; see the report).
- Nav2 navigation to user-selected goals (3 of 3 scripted goals reached on the ground-truth map).
- Double DQN navigation, trained and evaluated: **0 of 9 goals reached**, so it does not work as a navigation policy.
- Tests and a multi-goal demonstration for the viva (`tools/verify_on_vm.sh`, `docs/demonstration.md`).

## Repository structure

```text
ros2-autonomous-maze-navigation/
├── ros2_ws/
│   └── src/
│       ├── maze_simulation/   # Member 1 (ament_cmake): world, spawn, watchdog
│       ├── slam_mapping/      # Member 2 (ament_cmake): SLAM, map tools
│       ├── maze_navigation/   # Member 3 (ament_cmake): Nav2, goal evaluator
│       └── dqn_agent/         # Member 4 (ament_python): Double DQN, trained model
├── tools/                     # setup.sh, auto.sh, manual.sh, demo.sh, run_dqn_model.sh, verify_*.{sh,py}
├── maps/                      # teammate prototype test_map (not the final maze)
├── docs/
│   ├── members/               # per-member guides
│   ├── results/               # logs and CSVs from VM runs (old_6x6_maze/ = earlier layout)
│   ├── report.md / report.pdf
│   ├── OWNERSHIP.md, architecture.md, testing.md, ...
│   └── screenshots/
├── README.md
├── .gitignore
└── requirements.txt
```

Package assets (worlds, config, launch files, models) live inside the packages above. Root `maps/` holds only the teammate's prototype `test_map` (not the final maze).

## Team division (see [docs/OWNERSHIP.md](docs/OWNERSHIP.md) for who wrote what)

| Member | Package | Scope | Guide |
|---|---|---|---|
| 1 | `maze_simulation` | Gazebo maze from Figure 1, TurtleBot3 Burger, bridge, launch | [docs/members/member_1.md](docs/members/member_1.md) |
| 2 | `slam_mapping` | SLAM Toolbox, map save/validate, AMCL localization | [docs/members/member_2.md](docs/members/member_2.md) |
| 3 | `maze_navigation` | Nav2 (A* + MPPI), RViz goals, 3-goal metrics | [docs/members/member_3.md](docs/members/member_3.md) |
| 4 | `dqn_agent` | Double DQN training/evaluation, comparison with Nav2 | [docs/members/member_4.md](docs/members/member_4.md) |

Also: [architecture](docs/architecture.md) · [integration](docs/integration.md) · [installation](docs/installation.md) · [testing & status](docs/testing.md) · [demonstration](docs/demonstration.md) · [VIVA notes](docs/viva.md) · [report](docs/report.md).

> **Status:** all four workstreams are integrated on `main` and were run on an Ubuntu 24.04 VM: Nav2 3/3, DQN 0/9, SLAM precision 0.819 (below the 0.85 target). See [docs/testing.md](docs/testing.md) and the report for the per-requirement status.

## Target environment

- Ubuntu 24.04 (ARM64 or AMD64, depending on host)
- ROS 2 Jazzy
- Gazebo Harmonic and compatible ROS-Gazebo packages
- TurtleBot3 Burger, Nav2, SLAM Toolbox and compatible packages (versions to be pinned when selected)
- Python 3 for DQN experiments; Python-only packages are listed in `requirements.txt` (numpy, PyYAML, pytest)

**Note:** validated only on one Ubuntu 24.04 ARM64 VM, not from a clean install. Gazebo, ROS 2 and Nav2 package availability can vary by platform and package source. The DQN uses NumPy (no PyTorch).

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

- [x] Member 1: maze world + spawn launch (run on the VM)
- [x] Member 2: SLAM/localization workflow + map validator (SLAM map precision 0.819, below the 0.85 target; FAIL)
- [x] Member 3: Nav2 config + 3-goal runner (3 of 3 goals reached on the ground-truth map)
- [x] Member 4: Double DQN, trained 60 episodes (0 of 9 goals reached, truth-confirmed)
- [x] Reproducible three-goal demonstration (`bash tools/auto.sh`; scripted `bash tools/verify_on_vm.sh nav`)
- [ ] SLAM map that meets the 0.85 precision target, and Nav2 run on the SLAM map
- [ ] A DQN that reaches goals (needs more training or a hybrid design; not run)

## Repository hygiene

Do not commit generated ROS build files, credentials, large training checkpoints, or rosbag captures. Use Git LFS or external storage if large binary artifacts genuinely need versioning. Include only real simulation and test evidence in reports.
