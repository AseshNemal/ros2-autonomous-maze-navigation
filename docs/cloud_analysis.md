# Cloud analysis of the known problems (no Ubuntu/Gazebo available)

> **HISTORICAL (previous 6 x 6 maze).** This document describes analysis/work done on the removed 6 x 6 layout (`maze.world`, `maze_layout.yaml`, start (-1.25, -1.25)). Its numbers, thresholds and corridor widths do not apply to the current maze (`custom_maze.sdf`, 7 x 6, spawn (0.25, 0.0)). Current results: `docs/testing.md`, `docs/results/vm/`.

Everything here was done in a ROS-less Claude Cloud container: static inspection, unit tests, and three small
Python tools in `tools/`. **None of it is Gazebo/ROS evidence.** Fixes marked *hypothesis* or *surrogate* must
be confirmed on Ubuntu (`docs/ubuntu_test_checklist.md`); raw outputs are in `docs/cloud_checks/`.

## 1. Assignment audit (`docs/Assignment_02.pdf`)

| Requirement (brief) | Status | Evidence / gap |
|---|---|---|
| Maze world in Gazebo "as given in Figure 1" | **Layout matches Figure 1 exactly (static)**; world loads (VM sim 7/7, earlier run) | `tools/check_figure1.py`: 12 + 12 interior walls measured from the PDF image, 0 differences. `maze.world` == regeneration. |
| TurtleBot3 Burger + LiDAR SLAM or equivalent mapping | **Implemented, map FAILS our own validation** | Last VM run: coverage 0.759, precision 0.577 (need 0.85). Section 3 below. |
| Autonomous navigation (Nav2 A*/custom) | **VM-verified on the ground-truth map**: 3/3 goals, 0 recoveries | `docs/results/vm/nav2_results_20261016-214253.csv`. Not run on the SLAM map. |
| Machine-learning approach (DQN) | **Implemented; no goal reached on the VM** (0/9) | Section 4: causes found by analysis; fixes validated only on a surrogate. |
| User clicks a goal | Implemented (RViz `nav2_rviz_plugins/GoalTool` present in `navigation.rviz`); live click demo not recorded | VIVA item. |
| Minimise travel time / smooth, safe | Metrics logged (time, path length, recoveries) | `run_three_goals.py` CSV; efficiency vs a cell-count reference only. |
| Deliverable: world file, packages, launch files, config in GitHub | Done (this repo, branch `dev-integration`) | `main` is NOT updated (owner decision). |
| Deliverable: 5-7 page report with maze design, method, results (time, path efficiency in different methods, challenges), improvements | `docs/report.md` + `docs/report.pdf` | DQN-vs-Nav2 comparison has only Nav2 data so far. |
| Deliverable: demo of 3 clicked goals at the VIVA | Not yet rehearsed on this branch | Checklist step 8. |

## 2. Static inspection findings (all now enforced by `tools/verify_static.py`)

Checked and consistent: default world is named `maze` and every `/world/maze/...` literal (DQN reset, `check_control.py`,
`verify_on_vm.sh`, `demo.sh`) matches; the launch spawns into the world name read from the world file (so
`world:=custom_maze` spawns `-world custom_maze`); every script is installed by its CMake file; the Gazebo bridge
expects `geometry_msgs/TwistStamped` on `/cmd_vel` and no node publishes plain `Twist` there (Nav2's `Twist` goes through
`twist_stamper.py`); map and params paths exist; the default spawn (-1.25, -1.25) clears all walls by > 0.155 m and
the custom world's spawn (0.25, 0) by > 0.155 m.

**Defects found and fixed:** `maze_simulation` and `slam_mapping` `package.xml` did not declare packages their code
imports (`rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_msgs`, `rosgraph_msgs`, `launch`, `launch_ros`,
`ament_index_python`, `python3-yaml`); VM `*.log` files were git-ignored, so logs cited by the report (e.g.
`control_20261016-235730.log`) could never be committed - `.gitignore` now keeps `docs/results/**/*.log`.
`tools/verify_static.py` now fails if a package imports an undeclared dependency.

Not changed on purpose: `slam_mapping/test/test_tour_route.py` is run by `verify_static.py` but not registered with
`ament_add_pytest_test` (that would change the 22-test count on the VM).

## 3. SLAM map validation failure

* **The validator and tour are not the cause.** `tools/offline_mapping_check.py` ray-casts a 360-beam, 0.12-3.5 m LiDAR
  along the real coverage tour with perfect poses and validates the resulting map: coverage 1.0, precision 1.0, free_ok 1.0
  (PASS). It also shows how little heading drift the validator tolerates (uncorrected, per metre driven; the tour is about
  23.5 m): 0.2 deg/m PASS (0.986 / 0.869), 0.3 deg/m FAIL (precision 0.663), 1 deg/m FAIL.
* **The committed SLAM map is distorted, not just offset.** `tools/analyse_slam_map.py` on `maze_slam.pgm` (it reproduces the
  recorded 0.759 / 0.577 exactly, so it is the last VM run): the best rigid re-alignment (yaw +-12 deg, shift +-0.2 m)
  only reaches precision 0.598; 1138 occupied pixels where the true walls need roughly half that; and only **20 %** of
  line-like wall pixels lie within 5 deg of a maze axis, against **77 %** for the ground-truth map. The upper part of the
  image (`docs/figures/fig_slam.png`) shows walls at diagonal angles: heading was lost partway through the tour.
* **Odometry is not the cause (VM evidence).** `control_teleop_20261016-235730.json`: odom vs Gazebo truth - forward +0.46/+0.46 m,
  backward -0.32/-0.32 m, left +1.03/+0.99 rad, right -1.03/-1.07 rad.
* **Probable cause:** the scan matcher. `slam_toolbox` searches around the odometry-predicted pose; the config allowed
  +-0.5 m and +-20 deg per scan (`correlation_search_space_dimension 0.5`, `coarse_search_angle_offset 0.349`), far more than
  the odometry error, and the 0.45 m corridors are self-similar, so wrong correlation peaks can win.
* **Change made (hypothesis, unverified on the VM):** window reduced to 0.2 m and 0.175 rad (about 10 deg, still ~17x the
  measured 0.04 rad odometry error per turn). No other SLAM parameter was changed. If the next VM mapping run is not
  better, revert these two lines (old values are in the YAML comment) and use the diagnosis commands in the checklist.
* **Coverage:** "incomplete coverage" is explained by the same distortion (walls drawn in the wrong place count as
  missing). The tour itself covers all 36 cells (`test_tour_route.py`) and the perfect-pose map covers 100 % of wall length.

## 4. DQN failure to learn

* **The VM run was too short to learn anything.** 50 episodes made ~700 environment steps because every episode ends in a
  collision after ~14 steps; epsilon decayed over 20 000 steps and was still 0.967 at the end. A kinematic surrogate of the
  environment (`tools/offline_dqn_surrogate.py`; real agent, reward, observation, collision rule, ray-cast LiDAR on the real
  maze walls, unicycle motion) reproduces the VM run (50 episodes, pure DQN, old settings: 0 successes, all collisions,
  `docs/cloud_checks/dqn_surrogate_A_*`).
* **Corner cutting.** In hybrid (waypoint) mode the target switched when the robot was within 0.25 m of a cell centre, i.e.
  on *entering* the 0.5 m cell, so every turn was a diagonal into the corner. Surrogate diagnostic: 1-cell paths 92 %
  success but 2-cell 29 % and 3-cell 14 %, with collisions concentrated on the second waypoint. Tolerance 0.15 m: 2-cell
  ~87 %, 3-cell ~92 % (0.07 and 0.10 m give similar results; 0.15 m keeps margin for real odometry noise).
* **Changes made:** `waypoint_tolerance` 0.25 -> 0.15 m (now `environment.waypoint_tolerance_m` in `dqn_params.yaml`,
  passed by train and evaluate); `epsilon_decay_steps` 20 000 -> 3 000 plus a `--epsilon-decay` option; the VM `dqn` stage now
  trains and evaluates pure DQN **and** hybrid, with separate checkpoints and CSVs. Reward, network and action set were not
  changed (a 1 m observation range and gentler turning were tried on the surrogate and made no difference).
* **Surrogate results (1500 episodes, curriculum 1-3 cells, greedy evaluation on the three assignment goals, 10 trials each):**

| Variant | Goals (0,5) / (2,0) / (3,3) | Total |
|---|---|---|
| hybrid, new settings | 5/10, 8/10, 7/10 | **20/30** |
| hybrid, old 0.25 m tolerance (and 3000-step epsilon) | 0/10, 0/10, 0/10 | 0/30 |
| pure DQN, new settings | 0/10, 0/10, 0/10 | 0/30 |

  A pure DQN sees only local LiDAR and a straight-line goal bearing, so it cannot plan around walls over 5-8 cells; this
  is a limit of the method, not a bug. **These are surrogate numbers and must not be reported as Gazebo results.** The
  surrogate has no wheel dynamics, no real LiDAR timing and no Gazebo contact; real performance will differ.

## 5. Clock / TF warnings and Nav2 start-up

Documented VM cause (report section 5.2): a stale `ros_gz_bridge` from a previous stage republished `/clock`, `/tf`, `/scan`
(58 clock-jump warnings, TF extrapolation, dropped scans), fixed by `kill_stack` in `verify_on_vm.sh` (0 / 0 / 0 afterwards).
Static review found no second `/tf` or `/clock` source in the launch files: one bridge (`bridge.yaml`), one
`robot_state_publisher`, `use_sim_time` true for SLAM, Nav2, RViz, `twist_stamper`, `set_initial_pose`. The command
watchdog and the DQN node run on wall time and only compare their own timestamps, so they are unaffected. Nav2 start-up fixes
already on the branch: capitalised `"False"/"True"` launch arguments, `bond_timeout` 30 s, wait for `bt_navigator` active.
Nothing further could be derived from source; re-check the logs for these warnings on the next VM run.

## 6. Reproduce the cloud checks

```bash
pip install pytest numpy pyyaml pillow        # test-only, not project dependencies
python3 tools/verify_static.py                # OVERALL: PASS
python3 tools/check_figure1.py
python3 tools/offline_mapping_check.py --yaw-drift 0.3
python3 tools/analyse_slam_map.py
python3 tools/offline_dqn_surrogate.py --episodes 1500 --epsilon-decay 3000 --guidance waypoints --wp-tol 0.15 --eval-episodes 10
```
