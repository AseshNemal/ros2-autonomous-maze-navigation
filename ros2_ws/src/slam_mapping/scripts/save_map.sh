#!/usr/bin/env bash
# Save the live /map to <dir>/<name>.pgm/.yaml (default dir: package maps/ in the source tree if
# MAZE_REPO is set, else ~/maze_maps). Usage: save_map.sh <name> [dir]
set -euo pipefail
name="${1:?usage: save_map.sh <name> [dir]}"
dir="${2:-${MAZE_REPO:+$MAZE_REPO/ros2_ws/src/slam_mapping/maps}}"
dir="${dir:-$HOME/maze_maps}"
mkdir -p "$dir"
"$(ros2 pkg prefix nav2_map_server)/lib/nav2_map_server/map_saver_cli" -f "$dir/$name" \
  --ros-args -p save_map_timeout:=10000.0 -p use_sim_time:=true
echo "Saved: $dir/$name.yaml"
echo "Validate: python3 validate_map.py $dir/$name.yaml --map-origin-in-world -1.25 -1.25"
