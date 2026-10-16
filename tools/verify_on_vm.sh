#!/usr/bin/env bash
# Staged runtime verification on Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic.
# Usage: tools/verify_on_vm.sh <stage>   stages: build | sim | slam | nav | dqn
# Each stage logs to docs/results/vm/<stage>_<timestamp>.log, prints "STAGE RESULT: PASS|FAIL"
# and exits non-zero on ANY failed check (a failed validation can never look successful).
# Run ONE Gazebo at a time (VM is small). Nothing here is claimed as verified until
# you have run it and committed the logs.
set -uo pipefail
# never run on top of the live demo: it would kill your Gazebo/RViz (set FORCE=1 to override)
if [ "${FORCE:-0}" != 1 ] && pgrep -f "tools/demo.sh" >/dev/null; then
  echo "tools/demo.sh is running. Close it (Ctrl+C) first, or run with FORCE=1."; exit 3; fi
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
# Kill EVERYTHING from a previous run. A leaked ros_gz_bridge keeps republishing /clock,/tf,/scan
# into the next Gazebo -> time jumps, dropped scans, TF extrapolation errors.
kill_stack() {
  pkill -f "ros2 launch" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null; pkill -f "gz_tools_vendor" 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null; pkill -f robot_state_publisher 2>/dev/null; pkill -f rviz2 2>/dev/null
  pkill -f "slam_toolbox|nav2_|lib/nav2|opennav|auto_map_drive|twist_stamper|set_initial_pose" 2>/dev/null
  sleep 3
  pkill -9 -f "gz sim|parameter_bridge|robot_state_publisher" 2>/dev/null; true
}
[ "$STAGE" = build ] || kill_stack
PIDS=()
cleanup() { for p in "${PIDS[@]:-}"; do kill -TERM -- "-$p" 2>/dev/null; done; [ "$STAGE" = build ] || kill_stack; }
trap cleanup EXIT
bg() { setsid "$@" >>"$LOG" 2>&1 & PIDS+=($!); }
RC=0
fail() { say "FAIL: $*"; RC=1; }
# run a command, tee its output to the log, and keep ITS exit status (not tee's)
run() { "$@" 2>&1 | tee -a "$LOG"; return "${PIPESTATUS[0]}"; }
say() { echo "[$(date +%T)] $*" | tee -a "$LOG"; }
# wait until /scan is actually delivering messages (topic listed != data flowing on a slow VM)
wait_scan() { for _ in $(seq 1 45); do timeout 8 ros2 topic echo --once /scan >/dev/null 2>&1 && return 0; sleep 1; done; return 1; }
wait_topic() { for _ in $(seq 1 "${2:-60}"); do ros2 topic list 2>/dev/null | grep -qx "$1" && return 0; sleep 2; done; return 1; }

case "$STAGE" in
build)
  cd "$WS" || exit 1
  run colcon build --symlink-install --parallel-workers 1 || fail "colcon build"
  run colcon test --parallel-workers 1 || fail "colcon test"
  run colcon test-result --verbose || fail "colcon test-result (tests failed)" ;;
sim)
  bg ros2 launch maze_simulation maze_simulation.launch.py gui:=false
  wait_scan || { fail "/scan never appeared"; }
  [ "$RC" = 0 ] && { run python3 "$WS/src/maze_simulation/scripts/check_sim_topics.py" --duration 8 \
      --json "$OUT/sim_topics_$TS.json" || fail "check_sim_topics reported failures"; } ;;
slam)
  bg ros2 launch maze_simulation maze_simulation.launch.py gui:=false
  wait_scan || fail "no /scan"
  if [ "$RC" = 0 ]; then
    bg ros2 launch slam_mapping mapping.launch.py use_rviz:=false
    sleep 10
    run timeout "${MAP_TIMEOUT:-900}" python3 "$WS/src/slam_mapping/scripts/tour_map_drive.py" || fail "tour_map_drive (coverage tour did not finish)"
    MAPYAML="$WS/src/slam_mapping/maps/maze_slam.yaml"; rm -f "$MAPYAML" "${MAPYAML%.yaml}.pgm"
    # the saved map must come from slam_toolbox ONLY (a stray map_server/Nav2 would save the wrong map)
    PUBS=$(ros2 topic info /map -v 2>/dev/null | grep "Node name:" | awk '{print $3}' | sort -u | tr '\n' ' ')
    say "/map publishers: $PUBS"
    [ "$PUBS" = "slam_toolbox " ] || fail "/map is not published by slam_toolbox alone (found: $PUBS) - stop other ROS stacks"
    run bash "$WS/src/slam_mapping/scripts/save_map.sh" maze_slam "$WS/src/slam_mapping/maps" || fail "save_map"
    [ -f "$MAPYAML" ] || fail "maze_slam.yaml was not written"
    if [ -f "$MAPYAML" ]; then
      python3 "$WS/src/slam_mapping/scripts/validate_map.py" "$MAPYAML" --map-origin-in-world -1.25 -1.25 \
        2>&1 | tee -a "$OUT/slam_validation_$TS.txt" | tee -a "$LOG"
      [ "${PIPESTATUS[0]}" = 0 ] || fail "map validation (see slam_validation_$TS.txt)"
    fi
  fi ;;
nav)
  MAPSRC="${MAP_SOURCE:-world}"   # world = ground-truth map, slam = maze_slam.yaml
  if [ "$MAPSRC" = slam ]; then MAP="$WS/src/slam_mapping/maps/maze_slam.yaml"; IX=0.0; IY=0.0
  else MAP="$WS/src/slam_mapping/maps/maze_groundtruth.yaml"; IX=-1.25; IY=-1.25; fi
  bg ros2 launch maze_simulation maze_simulation.launch.py gui:=false
  wait_scan || fail "no /scan"
  if [ "$RC" = 0 ]; then
    bg ros2 launch maze_navigation navigation.launch.py use_rviz:=false map:="$MAP" initial_x:=$IX initial_y:=$IY
    # wait (max ~4 min) until Nav2 is really active instead of a blind sleep
    UP=0
    for _ in $(seq 1 40); do
      ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "active \[3\]" &&
      ros2 topic list --include-hidden-topics 2>/dev/null | grep -qx /navigate_to_pose/_action/status && { UP=1; break; }
      sleep 2
    done
    run ros2 lifecycle get /bt_navigator
    run ros2 node list
    run ros2 topic info /cmd_vel
    if [ "$UP" = 1 ]; then say "Nav2 active, /navigate_to_pose available"
      ros2 node list 2>/dev/null | grep -q collision_monitor || say "WARN: collision_monitor node not found"
      run ros2 run maze_navigation run_three_goals.py --csv "$OUT/nav2_results_$TS.csv" \
          --map-source "$MAPSRC" --reset-to-start || fail "run_three_goals: not all goals reached"
    else fail "Nav2 never became active (bt_navigator / /navigate_to_pose)"; fi
  fi ;;
dqn)
  bg ros2 launch maze_simulation maze_simulation.launch.py gui:=false
  wait_scan || fail "no /scan"
  if [ "$RC" = 0 ]; then
    run ros2 run dqn_agent train_dqn --episodes "${EPISODES:-300}" --curriculum-max-cells 3 \
        --guidance "${GUIDANCE:-none}" --csv "$OUT/dqn_train_$TS.csv" || fail "train_dqn"
    run ros2 run dqn_agent evaluate_dqn --checkpoint ~/.ros/dqn_agent/checkpoints/dqn_maze.npz \
        --guidance "${GUIDANCE:-none}" --csv "$OUT/dqn_eval_$TS.csv" || fail "evaluate_dqn"
  fi ;;
*) echo "unknown stage"; exit 2 ;;
esac
[ "$RC" = 0 ] && say "STAGE RESULT: PASS ($STAGE); log: $LOG" || say "STAGE RESULT: FAIL ($STAGE); log: $LOG"
exit "$RC"
