# Member 3 - Autonomous Navigation (`maze_navigation`)

Status key: **VERIFIED** (actually run and passed) · **IMPLEMENTED** (code exists, runtime check pending) · **FAILED** · **BLOCKED** · **NOT IMPLEMENTED**.
Runtime results on the current maze (Ubuntu VM, 2026-10-17): see [testing.md](../testing.md) and `docs/results/vm/`. Summary below under Tests.

## Responsibility
Nav2 integration, A* global planning, costmaps, local obstacle avoidance, RViz goal clicking, three-goal evaluation with metrics.

## Provenance
| Item | Origin |
|---|---|
| Nav2 params, `navigation.launch.py`, `navigation.rviz`, `evaluate_goal.py`, README, `INTEGRATION_CHECKLIST.md` | **Original teammate work**: `RavinduYashas`, 7 commits `744dc77..e9bcffe`, merged via PR #2 (merge commit, intact). |
| Fixes and additions listed below | **Emergency integration code** (commit `4d9c74f`, `fix(navigation)`), AI-assisted. Original lines are marked `# integration fix:` where changed. |

## Original design (kept)
NavFn planner with `use_astar: true` (A*), MPPI controller (DiffDrive; since **replaced by Regulated Pure Pursuit**, because MPPI stalled with zero velocity in the 0.5 m corridors), AMCL localization, costmaps, collision monitor, RViz layout, validated launch inputs, odometry-based goal evaluator (`evaluate_goal.py`, unchanged).

## Fixes found by review (the Nav2 bring-up and 3-goal run on the VM are listed under Tests)
1. `robot_radius 0.22` + `inflation_radius 0.7` (both costmaps): the maze's corridors are only 0.5 m wide, so every corridor cell would be inscribed-lethal and **no path could be planned**. Now local radius 0.11 (Burger ~0.105), global planning radius 0.08 (so a start close to a wall stays plannable; the local costmap and collision monitor keep the real clearance), inflation 0.25, scaling 8. A static test asserts the robot fits.
2. `/cmd_vel` type conflict: Jazzy Nav2 publishes `Twist`; the simulation expects `TwistStamped`. Collision-monitor output moved to `cmd_vel_nav2`; new `twist_stamper.py` republishes as `/cmd_vel` (TwistStamped). **Assumes collision_monitor is in the Jazzy bringup chain - confirm with `ros2 node list`** (`verify_on_vm.sh nav` prints it).
3. Planner `tolerance 0.5 -> 0.1` (0.5 m could accept a goal in the neighbouring cell through a wall); goal tolerance 0.15, yaw tolerance relaxed (clicked goals have no heading); AMCL laser range 3.5; controller **Regulated Pure Pursuit at 0.15 m/s** (replacing MPPI).
4. `navigation.launch.py` defaults to the ground-truth map and starts `set_initial_pose.py` (AMCL needs a pose); args `initial_x/y/yaw`.
5. `run_three_goals.py`: sends the 3 assignment goals, logs outcome, sim time, odom path length, recoveries, grid-A* reference length and efficiency to CSV. `--reset-to-start` teleports the robot back to the start per goal so DQN can be compared on identical pairs.

## Topics / actions
`navigate_to_pose` (action, `nav2_msgs/NavigateToPose`), `/goal_pose` (RViz), `/initialpose`, `/amcl_pose`, `/map`, `/scan`, `/odom`, `cmd_vel_nav` -> `cmd_vel_smoothed` -> `cmd_vel_nav2` (Twist) -> `/cmd_vel` (TwistStamped).

## Run
```bash
ros2 launch maze_simulation maze_simulation.launch.py gui:=false     # terminal 1 (or gui:=true)
ros2 launch maze_navigation navigation.launch.py                    # terminal 2: ground-truth map + RViz
#   SLAM map: map:=<abs>/maze_slam.yaml initial_x:=0.0 initial_y:=0.0
# RViz: click "Nav2 Goal" (this is the clicked-goal requirement), or:
ros2 run maze_navigation run_three_goals.py --csv ~/nav2_results.csv --reset-to-start
```
Or staged: `tools/verify_on_vm.sh nav` (`MAP_SOURCE=slam` for the SLAM map).

## Tests
- VERIFIED (static, pytest): corridor-fit, A* enabled, tolerance, cmd_vel chain, goals reachable, sources compile.
- Original teammate offline checks (their `INTEGRATION_CHECKLIST.md`): synthetic logger/preflight checks - theirs, not re-run by us.
- VERIFIED (Ubuntu VM, 2026-10-17, ground-truth map **before the east-wall map fix**): Nav2 active, `nav2_results_20261017-113157.csv` - 3 of 3 goals reached (39.45 s / 5.53 m, 16.53 s / 2.29 m, 10.30 s / 0.88 m; 1 recovery in total). Four consecutive goals without reset also succeeded; command-line goals (`tools/send_goal.sh`) succeeded (10.3 s, 26.1 s).
- Known pitfall (reproduced): a wrong RViz *2D Pose Estimate* leaves AMCL metres from the true pose and the next goal fails; `tools/reset_robot.sh` restores both.
- PENDING: Nav2 re-run on the corrected ground-truth map; a scripted RViz *Nav2 Goal* click (do it on the VM desktop); Nav2 on the SLAM map (never run).

## Limitations
Nav2's NavFn A* satisfies the brief's "Nav2 stack (Dijkstra/A*)" option; no custom planner written. Reference lengths are grid-centreline estimates, so the efficiency ratio is only indicative (the robot starts at the spawn, not a cell centre, and stops within the goal tolerance). Tested only on the ground-truth map.

## VIVA
*A* vs Dijkstra?* Same optimality; A* adds a heuristic to expand fewer nodes (`use_astar`). *Global vs local planner?* NavFn plans on the global costmap; Regulated Pure Pursuit follows the global path with a lookahead point and slows near obstacles and sharp curvature on the local costmap (the collision monitor is the last safety layer). *Why inflate?* Planning treats the robot as a point; inflation encodes its radius - too much and narrow corridors vanish (our fix #1).
