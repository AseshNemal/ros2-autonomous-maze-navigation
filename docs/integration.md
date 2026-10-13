# Integration

All four packages live in one ROS 2 workspace (`ros2_ws/`) and are built together but developed independently.

## Dependency order
```
maze_simulation  ->  slam_mapping  ->  maze_navigation
        \                                   \
         ------------------------------->  dqn_agent (also compares with maze_navigation)
```

## Shared interface (TO BE AGREED — not yet defined)
Members must fix and record here before integrating:
- Topics: `/scan`, `/odom`, `/cmd_vel`, `/map`, `/tf`, `/clock`
- Frames: `map`, `odom`, `base_footprint`/`base_link`, LiDAR frame
- `use_sim_time` convention (`true` for every launch)
- Robot spawn pose and the fixed start/goal pairs used for evaluation
- Map file location (owned by `slam_mapping`; others reference it, not copy it)
- `TURTLEBOT3_MODEL=burger`

## Intended workflow
1. Member 1 delivers a launchable world; others pull and run it.
2. Member 2 maps the world and commits the real map files.
3. Member 3 runs Nav2 on that map and logs three-goal results.
4. Member 4 trains in the same world and compares against Member 3's results using identical start/goal pairs.
5. Final: one documented end-to-end run, with screenshots and results in `docs/`.

## Branching suggestion
One branch per package (`feature/maze_simulation`, ...), merged into `main` by pull request. Each member should mainly edit their own package, avoiding cross-package conflicts.

## Integration status
Nothing has been integrated or tested yet.
