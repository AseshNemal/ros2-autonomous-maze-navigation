#!/usr/bin/env bash
# Command-line alternative to clicking "2D Goal Pose" in RViz (use when the remote desktop mouse is unreliable).
# Needs the simulation and Nav2 to be running (bash tools/demo.sh, or the sim + navigation launch files).
#
#   bash tools/send_goal.sh G1            # goal cell 1 from config/mission.yaml
#   bash tools/send_goal.sh G2
#   bash tools/send_goal.sh G3
#   bash tools/send_goal.sh 1.25 0.75     # any world x y (metres) inside the maze
#
# Prints the distance remaining every 2 s and the final result (SUCCEEDED / ABORTED / CANCELED); exit 0 only on SUCCEEDED.
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
set +u; source /opt/ros/jazzy/setup.bash; [ -f "$REPO/ros2_ws/install/setup.bash" ] && source "$REPO/ros2_ws/install/setup.bash"; set -u
if [ $# -eq 1 ]; then
  read -r X Y < <(python3 - "$1" "$REPO" <<'EOF'
import sys
sys.path.insert(0, sys.argv[2] + "/ros2_ws/src/maze_simulation/scripts")
import maze_tools as mt
L = mt.load()
i = int(sys.argv[1].lstrip("Gg")) - 1
x, y = mt.cell_center(L, *L["goal_cells"][i])
print(x, y)
EOF
)
elif [ $# -eq 2 ]; then X=$1; Y=$2
else sed -n 2,12p "$0"; exit 2; fi
echo "Sending Nav2 goal x=$X y=$Y (frame map) ..."
exec python3 "$REPO/tools/send_goal.py" "$X" "$Y" "${@:3}"
