# Final report draft (5-7 pages) - matches the brief's four required sections

> Every number in "Results" must come from a real run recorded in `docs/results/`. Sections marked PENDING have no data yet. Do not fill them with estimates.

## 1. Maze design
Figure 1 is a 6x6 grid of 0.5 m cells (3 m x 3 m). Walls were extracted from the figure by measuring line thickness; thick lines are walls. Implementation: `maze_layout.yaml` -> generated SDF world (0.05 m thick, 0.5 m high walls); the free corridor is 0.45 m, which is only ~2.5x the TurtleBot3 Burger width (0.178 m) and drives costmap tuning. Start (-1.25,-1.25); three goals chosen with grid-optimal routes of 2.5 m, 3.5 m and 8.0 m (3, 4 and 12 turns) so difficulty varies. Include the ASCII/figure comparison.

## 2. Navigation method
SLAM Toolbox (async) builds the map; AMCL localises; Nav2 plans globally with NavFn A* and follows with the MPPI controller over inflated costmaps (robot radius 0.11 m, inflation 0.25 m). Goals are clicked in RViz (`/goal_pose`). Machine learning: Double DQN (15-D state: 12 LiDAR sectors + goal distance/bearing; 5 discrete actions; reward shaping; replay; target network). Explain the optional waypoint-guided hybrid and why pure DQN lacks global information.

## 3. Results - PENDING (no Gazebo runs yet)
Tables to fill from CSVs: per goal and method (Nav2 / DQN pure / DQN waypoint-guided): success, simulated time, path length, efficiency (grid reference / travelled), recoveries/collisions; SLAM map validation (`validate_map.py` output); DQN learning curve. Challenges already documented: test_map invalid; inflation vs corridor width; stamped velocity; stale odometry after reset. Prior DQN Stage-1 trial (different world, 200 steps): 0/3 successes - report as a smoke test only.

## 4. Possible improvements
Tune MPPI for narrow corridors; custom A* with turn penalties; frontier exploration instead of wall following; PyTorch DQN/PER/dueling, longer training with randomised starts; LiDAR history in the state; sim-to-real discussion; Gazebo contact sensor for collisions.
