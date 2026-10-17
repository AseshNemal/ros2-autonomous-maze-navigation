# Demonstration procedure (VIVA / video)

**This is a script, not proof of a run.** Capture the evidence listed; do not present anything as working until you have seen it work. Everything below is UNVERIFIED until run. Use one Gazebo instance at a time.

| # | Segment | Commands | Expected | Capture | Known limits |
|---|---|---|---|---|---|
| 1 | Custom maze | `ros2 launch maze_simulation maze_simulation.launch.py` | 3.5 x 3.0 m maze matching Figure 1 (show the figure next to it), Burger in the bottom corridor | screenshot + figure side by side | Layout from our figure transcription; start/goals are our choice |
| 2 | Sensors | `ros2 run maze_simulation check_sim_topics.py` ; `ros2 topic hz /scan` | 7 PASS lines | terminal | - |
| 3 | SLAM | `ros2 launch slam_mapping mapping.launch.py`; `ros2 run slam_mapping auto_map_drive.py` (or teleop) | map grows in RViz | RViz recording | wall-follower may miss cells |
| 4 | Map saved + validated | `save_map.sh maze_slam`; `validate_map.py ... --map-origin-in-world 0.25 -0.25` | `RESULT: PASS` | terminal + map image | pass = geometric agreement |
| 5 | Navigation, 3 clicked goals | stop SLAM; `ros2 launch maze_navigation navigation.launch.py map:=<maze_slam.yaml> initial_x:=0.0 initial_y:=0.0`; click 3 goals with *2D Goal Pose* | robot reaches each | screen recording | fall back to the ground-truth map (default) if SLAM map is poor, and say so |
| 6 | Nav2 metrics | `ros2 run maze_navigation run_three_goals.py --csv ~/nav2.csv --reset-to-start` | CSV with time/length/efficiency | CSV in `docs/results/` | reference length is a grid estimate |
| 7 | DQN training | stop Nav2; `ros2 launch dqn_agent dqn_train.launch.py episodes:=300` | CSV, epsilon decay, reward curve | training CSV, plot | pure DQN may not learn; try `guidance:=waypoints` and say it is a hybrid |
| 8 | DQN evaluation | `ros2 launch dqn_agent dqn_eval.launch.py` | greedy runs on 3 goals | eval CSV, screen recording | report failures as they are |
| 9 | Comparison | `ros2 run dqn_agent compare_navigation --dqn ... --nav2 ...` | success rate/time/path per method | terminal | valid only for identical pairs (`--reset-to-start`) |
| 10 | Contributions | each member shows their package + `docs/members/member_N.md` | - | - | - |

Order matters: never leave Nav2, DQN or the auto-mapper running together (they all drive `/cmd_vel`). Pre-run everything once before recording.
