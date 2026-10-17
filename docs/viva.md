# VIVA notes (group)

Per-member answers are at the bottom of each `docs/members/member_N.md`. Cross-cutting questions:

- **Pipeline end to end?** Gazebo -> LiDAR/odom -> SLAM -> map -> AMCL -> Nav2 (A* + MPPI) -> `/cmd_vel`; RL branch: LiDAR + goal -> DQN -> `/cmd_vel`.
- **Why TwistStamped / twist_stamper?** Jazzy TurtleBot3 bridge uses stamped; Nav2 Jazzy emits plain Twist.
- **What is your fairest DQN-vs-Nav2 comparison?** Same maze, same start->goal pairs (reset-to-start), same timeout and goal tolerance, success rate + sim time + odometry path length vs grid reference. State that Nav2 has a map and DQN does not.
- **What is not proven?** Anything in `testing.md` marked UNVERIFIED/NOT DEMONSTRATED. Say it before being asked.
- **What did you find wrong in the inherited work?** (a) test_map is not the maze, (b) Nav2 inflation made corridors impassable, (c) Twist/TwistStamped mismatch, (d) DQN odom stale after teleport reset. Each has a fix and a static test.
- **Which parts are the team's and which are emergency code?** `OWNERSHIP.md`.
