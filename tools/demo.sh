#!/usr/bin/env bash
# One command live demo: builds, opens Gazebo (maze + TurtleBot3) and Nav2 + RViz.
# Then in RViz press "Nav2 Goal" and click on the map. Ctrl+C stops everything.
# Usage: bash tools/demo.sh [teleop|slam]   (default = navigation with the ground-truth map)
# never run on top of a test run (two programs driving one robot / killing each other's Gazebo)
if pgrep -f "tools/verify_on_vm.sh|tools/run_all.sh" >/dev/null; then
  echo "A test run (run_all.sh / verify_on_vm.sh) is in progress. Wait for it to finish, then run the demo."; exit 1; fi
REPO="$(cd "$(dirname "$0")/.." && pwd)"
export TURTLEBOT3_MODEL=burger MAZE_REPO="$REPO"
set +u; source /opt/ros/jazzy/setup.bash
# FastDDS shared memory leaves stale lock files when processes are killed and then makes Nav2 bring-up time out;
# use plain UDP (always available, plenty fast for this simulation).
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
cd "$REPO/ros2_ws" && colcon build --symlink-install --parallel-workers 1 || exit 1
source install/setup.bash
kill_stack() { pkill -f "ros2 launch" 2>/dev/null; pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null
  pkill -f robot_state_publisher 2>/dev/null; pkill -f rviz2 2>/dev/null; pkill -f "nav2_|opennav" 2>/dev/null
  pkill -f "twist_stamper|cmd_vel_watchdog|set_initial_pose|teleop_twist_keyboard|tour_map_drive" 2>/dev/null; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null; sleep 2; pkill -9 -f "gz sim|parameter_bridge|lib/nav2_|opennav" 2>/dev/null; true; }
kill_stack   # a leaked bridge from an earlier run causes TF/clock jumps
trap 'kill $(jobs -p) 2>/dev/null; kill_stack' EXIT
ros2 launch maze_simulation maze_simulation.launch.py &   # wraps main's custom_maze.launch.py + cmd_vel watchdog
for _ in $(seq 1 60); do ros2 topic list 2>/dev/null | grep -qx /scan && break; sleep 2; done
MODE="${1:-nav}"; MODE="${MODE//[^a-z]/}"   # tolerate a stray comma/period when typing
# Gazebo can come up paused (the play button shows); make sure the simulation is running
gz service -s /world/custom_maze/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false' >/dev/null 2>&1
case "$MODE" in
  slam) ros2 launch slam_mapping mapping.launch.py &
        sleep 10; python3 src/slam_mapping/scripts/tour_map_drive.py
        bash src/slam_mapping/scripts/save_map.sh maze_slam ;;
  teleop) ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true ;;
  *) ros2 launch maze_navigation navigation.launch.py \
       map:="$REPO/ros2_ws/src/slam_mapping/maps/maze_groundtruth.yaml" initial_x:=0.25 initial_y:=0.0 &
     NAV=$!
     # wait until Nav2 is really active; if the lifecycle manager gave up, start it again
     for i in $(seq 1 90); do
       ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "active \[3\]" && { echo ">>> Nav2 is ACTIVE: click 'Nav2 Goal' in RViz"; break; }
       if [ "$i" = 45 ] || [ "$i" = 70 ]; then
         echo ">>> Nav2 not active yet - sending startup"
         ros2 service call /lifecycle_manager_localization/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" >/dev/null 2>&1
         ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" >/dev/null 2>&1
       fi
       sleep 2
     done
     wait $NAV ;;
esac
