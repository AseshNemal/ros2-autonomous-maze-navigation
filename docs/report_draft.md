# Final report draft (5-7 pages) - matches the brief's four required sections

> Every number in "Results" must come from a real run recorded in `docs/results/`. Sections marked PENDING have no data yet. Do not fill them with estimates.

## 1. Maze design
The maze is main's 7x6 Figure 1 layout (0.5 m cells, 3.5 m x 3.0 m, `generate_maze.py`). Implementation: `maze_layout.yaml` -> generated SDF world (0.01 m thick, 0.2 m high walls); narrowest corridor 0.5 m. Start (0.25,-0.25); three goals with grid-optimal routes of 5.5 m, 3.0 m and 1.5 m (7, 3 and 2 turns). Include the ASCII/figure comparison. Earlier measurements were in the previous 6x6 layout and must be re-run.

## 2. Navigation method
SLAM Toolbox (async) builds the map; AMCL localises; Nav2 plans globally with NavFn A* and follows with the MPPI controller over inflated costmaps (robot radius 0.11 m, inflation 0.25 m). Goals are clicked in RViz (`/goal_pose`). Machine learning: Double DQN (15-D state: 12 LiDAR sectors + goal distance/bearing; 5 discrete actions; reward shaping; replay; target network). Explain the optional waypoint-guided hybrid and why pure DQN lacks global information.

## 3. Results - PENDING (no Gazebo runs yet)
Tables to fill from CSVs: per goal and method (Nav2 / DQN pure / DQN waypoint-guided): success, simulated time, path length, efficiency (grid reference / travelled), recoveries/collisions; SLAM map validation (`validate_map.py` output); DQN learning curve. Challenges already documented: test_map invalid; inflation vs corridor width; stamped velocity; stale odometry after reset. Prior DQN Stage-1 trial (different world, 200 steps): 0/3 successes - report as a smoke test only.

## 4. Possible improvements
Tune MPPI for narrow corridors; custom A* with turn penalties; frontier exploration instead of wall following; PyTorch DQN/PER/dueling, longer training with randomised starts; LiDAR history in the state; sim-to-real discussion; Gazebo contact sensor for collisions.
