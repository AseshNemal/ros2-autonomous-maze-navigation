# Maps

| File | What it is | Valid for the final maze? |
|---|---|---|
| `test_map.pgm/.yaml` | Prototype fragment from teammate (Member 2, commit 7544234). 34x45 px = 1.7 x 2.25 m, 78.6% unknown. | **No** - partial fan scan, not the 3 x 3 m maze (`validate_map.py`: wall_coverage 0.061 -> FAIL). Kept for history only. |
| `ros2_ws/src/slam_mapping/maps/maze_groundtruth.*` | Occupancy grid rasterised from `maze_layout.yaml` by `maze_tools.py map`. **Not a SLAM output.** | Yes, as an exact fallback map for Nav2/AMCL. |
| `ros2_ws/src/slam_mapping/maps/maze_slam.*` | Real SLAM Toolbox map. **Does not exist until generated in Gazebo** (see docs/members/member_2.md). | Pending (UNVERIFIED). |
