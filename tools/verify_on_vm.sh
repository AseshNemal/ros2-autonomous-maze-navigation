#!/usr/bin/env bash
# Staged runtime verification on Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic.
# Usage: tools/verify_on_vm.sh <stage>   stages: build | sim | slam | nav | dqn
# Each stage logs to docs/results/vm/<stage>_<timestamp>.log and records PASS/FAIL.
# Run ONE Gazebo at a time (VM is small). Nothing here is claimed as verified until
# you have run it and committed the logs.
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO/ros2_ws"
OUT="$REPO/docs/results/vm"; mkdir -p "$OUT"
TS="$(date +%Y%m%d-%H%M%S)"
STAGE="${1:?stage: build|sim|slam|nav|dqn}"
LOG="$OUT/${STAGE}_$TS.log"
export TURTLEBOT3_MODEL=burger MAZE_REPO="$REPO"
set +u
source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u
PIDS=()
cleanup() { for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done; pkill -f "gz sim" 2>/dev/null; true; }
trap cleanup EXIT
bg() { "$@" >>"$LOG" 2>&1 & PIDS+=($!); }
say() { echo "[$(date +%T)] $*" | tee -a "$LOG"; }
wait_topic() { for _ in $(seq 1 "${2:-60}"); do ros2 topic list 2>/dev/null | grep -qx "$1" && return 0; sleep 2; done; return 1; }

case "$STAGE" in
build)
  cd "$WS" && colcon build --symlink-install --parallel-workers 1 2>&1 | tee -a "$LOG"
  colcon test --parallel-workers 1 2>&1 | tee -a "$LOG"; colcon test-result --verbose 2>&1 | tee -a "$LOG" ;;
sim)
  bg ros2 launch maze_simulation maze_simulation.launch.py gui:=false
  wait_topic /scan 90 || { say "FAIL: /scan never appeared"; exit 1; }
  python3 "$WS/src/maze_simulation/scripts/check_sim_topics.py" --duration 8 --json "$OUT/sim_topics_$TS.json" | tee -a "$LOG" ;;
slam)
  bg ros2 launch maze_simulation maze_simulation.launch.py gui:=false
  wait_topic /scan 90 || { say "FAIL: no /scan"; exit 1; }
  bg ros2 launch slam_mapping mapping.launch.py use_rviz:=false
  sleep 10
  python3 "$WS/src/slam_mapping/scripts/auto_map_drive.py" --duration "${MAP_SECONDS:-420}" 2>&1 | tee -a "$LOG"
  bash "$WS/src/slam_mapping/scripts/save_map.sh" maze_slam "$WS/src/slam_mapping/maps" 2>&1 | tee -a "$LOG"
  python3 "$WS/src/slam_mapping/scripts/validate_map.py" "$WS/src/slam_mapping/maps/maze_slam.yaml" \
      --map-origin-in-world -1.25 -1.25 2>&1 | tee -a "$OUT/slam_validation_$TS.txt" | tee -a "$LOG" ;;
nav)
  MAPSRC="${MAP_SOURCE:-world}"   # world = ground-truth map, slam = maze_slam.yaml
  if [ "$MAPSRC" = slam ]; then MAP="$WS/src/slam_mapping/maps/maze_slam.yaml"; IX=0.0; IY=0.0
  else MAP="$WS/src/slam_mapping/maps/maze_groundtruth.yaml"; IX=-1.25; IY=-1.25; fi
  bg ros2 launch maze_simulation maze_simulation.launch.py gui:=false
  wait_topic /scan 90 || { say "FAIL: no /scan"; exit 1; }
  bg ros2 launch maze_navigation navigation.launch.py use_rviz:=false map:="$MAP" initial_x:=$IX initial_y:=$IY
  sleep 45
  ros2 lifecycle get /bt_navigator 2>&1 | tee -a "$LOG"
  ros2 node list 2>&1 | tee -a "$LOG"        # confirm collision_monitor is present (cmd_vel chain)
  ros2 topic info /cmd_vel 2>&1 | tee -a "$LOG"
  ros2 run maze_navigation run_three_goals.py --csv "$OUT/nav2_results_$TS.csv" \
      --map-source "$MAPSRC" --reset-to-start 2>&1 | tee -a "$LOG" ;;
dqn)
  bg ros2 launch maze_simulation maze_simulation.launch.py gui:=false
  wait_topic /scan 90 || { say "FAIL: no /scan"; exit 1; }
  ros2 run dqn_agent train_dqn --episodes "${EPISODES:-300}" --curriculum-max-cells 3 \
      --guidance "${GUIDANCE:-none}" --csv "$OUT/dqn_train_$TS.csv" 2>&1 | tee -a "$LOG"
  ros2 run dqn_agent evaluate_dqn --checkpoint ~/.ros/dqn_agent/checkpoints/dqn_maze.npz \
      --guidance "${GUIDANCE:-none}" --csv "$OUT/dqn_eval_$TS.csv" 2>&1 | tee -a "$LOG" ;;
*) echo "unknown stage"; exit 2 ;;
esac
say "stage $STAGE finished; log: $LOG"
