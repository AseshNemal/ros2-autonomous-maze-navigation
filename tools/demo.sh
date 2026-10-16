#!/usr/bin/env bash
# One command live demo: builds, opens Gazebo (maze + TurtleBot3) and Nav2 + RViz.
# Then in RViz press "2D Goal Pose" and click on the map. Ctrl+C stops everything.
# Usage: bash tools/demo.sh [teleop|slam]   (default = navigation with the ground-truth map)
REPO="$(cd "$(dirname "$0")/.." && pwd)"
export TURTLEBOT3_MODEL=burger MAZE_REPO="$REPO"
set +u; source /opt/ros/jazzy/setup.bash
cd "$REPO/ros2_ws" && colcon build --symlink-install --parallel-workers 1 || exit 1
source install/setup.bash
pkill -f "gz sim" 2>/dev/null
trap 'kill $(jobs -p) 2>/dev/null; pkill -f "gz sim" 2>/dev/null' EXIT
ros2 launch maze_simulation maze_simulation.launch.py &
for _ in $(seq 1 60); do ros2 topic list 2>/dev/null | grep -qx /scan && break; sleep 2; done
case "${1:-nav}" in
  slam) ros2 launch slam_mapping mapping.launch.py &
        sleep 10; ros2 run slam_mapping auto_map_drive.py --duration 420
        bash src/slam_mapping/scripts/save_map.sh maze_slam ;;
  teleop) ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true ;;
  *) ros2 launch maze_navigation navigation.launch.py \
       map:="$REPO/ros2_ws/src/slam_mapping/maps/maze_groundtruth.yaml" initial_x:=-1.25 initial_y:=-1.25 ;;
esac
