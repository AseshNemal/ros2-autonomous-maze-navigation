# Architecture

```
                         custom_maze.sdf  (Figure 1, 7x6 x 0.5 m, from main)
                  +-------------+--------------+------------------+
                  v             v              v                  v
            custom_maze.sdf   maze_groundtruth   reference A*    DQN curriculum/
          (maze_simulation)  (slam_mapping)    lengths         waypoints
                  |
   Gazebo Harmonic + TurtleBot3 Burger  --ros_gz_bridge-->  /scan /odom /tf /clock
                                                                  /cmd_vel (TwistStamped)

 Classical pipeline:  /scan,/odom -> slam_toolbox -> /map -> map_saver -> maze_slam.yaml
                      maze_slam.yaml -> map_server + AMCL (map->odom) -> Nav2
                      Nav2: NavFn(A*) + MPPI + costmaps -> cmd_vel_nav2 (Twist)
                            -> twist_stamper -> /cmd_vel (TwistStamped) -> Gazebo
 RL pipeline:         /scan,/odom -> DQN env (odom->world calibrated) -> Q-net action
                      -> /cmd_vel (TwistStamped) -> Gazebo -> reward/replay/Double-DQN update
```

## Shared contract
- World `custom_maze` (main's `worlds/custom_maze.sdf`, unmodified), entity `turtlebot3_burger`, frames `map`, `odom`, `base_footprint`, `base_link`, `base_scan`.
- Everything uses `use_sim_time:=true`.
- `/cmd_vel` is **TwistStamped** (Jazzy TurtleBot3 bridge). Exactly one controller publishes it at a time: auto-mapper **or** Nav2 (via twist_stamper) **or** DQN.
- Start (0.25, 0.0), yaw 0 (main's spawn; nearest cell 5,1). Goals (cells): G1 (0,3) = (1.25, 2.25); G2 (3,3) = (1.25, 0.75); G3 (3,0) = (-0.25, 0.75). The maze spans x -0.5..3.0, y -0.5..2.5 (centre (1.25, 1.0)).
- Grid-A* reference lengths from the start: G1 11 cells = 5.5 m (7 turns), G2 6 cells = 3.0 m (3 turns), G3 3 cells = 1.5 m (2 turns).
- Map frame: ground-truth map shares world coordinates; a SLAM map's origin is the spawn pose (offset (0.25, 0.0)).

## Launch procedures (one controller at a time)
| Purpose | Commands |
|---|---|
| Mapping | `maze_simulation` -> `slam_mapping mapping.launch.py` -> teleop/auto_map_drive -> `save_map.sh` |
| Classical navigation | `maze_simulation` -> `maze_navigation navigation.launch.py` -> RViz goal / `run_three_goals.py` |
| DQN training | `dqn_agent dqn_train.launch.py` (starts headless sim itself) |
| DQN evaluation | `dqn_agent dqn_eval.launch.py` |
