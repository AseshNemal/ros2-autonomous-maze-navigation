#!/usr/bin/env bash
# First-time setup after cloning (Ubuntu 24.04 + ROS 2 Jazzy already installed):
#   bash tools/setup.sh
# Checks the prerequisites, builds the ROS workspace and checks that the committed DQN model loads.
# Missing ROS packages: bash tools/install_deps_ubuntu.sh   (needs sudo, run once)
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
MODEL="$REPO/ros2_ws/src/dqn_agent/models/dqn_maze_none_60ep_0of9.npz"
fail() { echo "SETUP FAILED: $*" >&2; exit 1; }

[ -f /opt/ros/jazzy/setup.bash ] || fail "ROS 2 Jazzy not found in /opt/ros/jazzy (install it first: docs/installation.md)"
set +u; source /opt/ros/jazzy/setup.bash; set -u
for c in colcon ros2 gz python3; do
  command -v "$c" >/dev/null || fail "'$c' not found. Run once: bash tools/install_deps_ubuntu.sh"
done
for p in turtlebot3_gazebo ros_gz_sim nav2_bringup slam_toolbox rviz2; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 || fail "ROS package '$p' missing. Run once: bash tools/install_deps_ubuntu.sh"
done
python3 -c "import numpy, yaml" 2>/dev/null || fail "python3-numpy / python3-yaml missing: sudo apt install python3-numpy python3-yaml"
export TURTLEBOT3_MODEL=burger

echo ">>> building the workspace (one worker: small VMs run out of memory with more)"
( cd "$REPO/ros2_ws" && colcon build --symlink-install --parallel-workers 1 ) || fail "colcon build failed"
set +u; source "$REPO/ros2_ws/install/setup.bash"; set -u

echo ">>> checking the committed DQN model"
python3 - "$MODEL" "$REPO" <<'EOF' || fail "the committed DQN model does not load"
import sys
sys.path.insert(0, sys.argv[2] + "/ros2_ws/src/dqn_agent")
from dqn_agent.agent import DQNAgent
a = DQNAgent.load(sys.argv[1])
assert (a.state_dim, a.action_dim, a.train_steps, a.env_steps) == (15, 5, 1488, 1587), "unexpected checkpoint contents"
print(f"model OK: state {a.state_dim}, actions {a.action_dim}, {a.train_steps} optimizer updates, {a.env_steps} transitions")
EOF

cat <<EOF

SETUP OK. Next (one command each, in a terminal at $REPO):
  bash tools/auto.sh              # Gazebo + RViz + Nav2: click "Nav2 Goal" in RViz
  bash tools/run_dqn_model.sh     # trained DQN model, no training (do NOT run both at once)
EOF
