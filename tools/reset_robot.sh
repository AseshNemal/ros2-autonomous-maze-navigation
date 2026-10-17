#!/usr/bin/env bash
# RESET: put the robot back at the spawn pose in Gazebo AND tell Nav2/AMCL it is there.
# Use it when RViz and Gazebo show the robot in different places (for example after a wrong
# "2D Pose Estimate" click, after Gazebo's reset button, or after driving with drive.sh / manual.sh).
#
#   bash tools/reset_robot.sh
#
# Needs the simulation (and Nav2 if you want localization reset) to be running.
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
set +u; source /opt/ros/jazzy/setup.bash; [ -f "$REPO/ros2_ws/install/setup.bash" ] && source "$REPO/ros2_ws/install/setup.bash"; set -u
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
read -r X Y < <(python3 - "$REPO" <<'EOF'
import sys
sys.path.insert(0, sys.argv[1] + "/ros2_ws/src/maze_simulation/scripts")
import maze_tools as mt
print(*mt.spawn_xy(mt.load()))
EOF
)
echo "Teleporting the robot to the spawn pose ($X, $Y) in Gazebo ..."
gz service -s /world/custom_maze/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: \"burger\" position {x: $X y: $Y z: 0.01} orientation {w: 1}" | tr -d '\n'; echo
sleep 2
echo "Setting the AMCL initial pose to the same place ..."
python3 "$REPO/ros2_ws/src/slam_mapping/scripts/set_initial_pose.py" --x "$X" --y "$Y" --yaw 0 --timeout 20 \
  --ros-args -p use_sim_time:=true \
  || echo "(Nav2 is not running: only Gazebo was reset)"
echo "done: Gazebo and RViz should now show the robot at the same place"
