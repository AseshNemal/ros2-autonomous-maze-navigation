#!/usr/bin/env bash
# Run the committed, already trained DQN model in the 7x6 Gazebo maze. NO training happens here.
#
#   bash tools/run_dqn_model.sh                  # Gazebo window + evaluation (3 trials per assignment goal)
#   HEADLESS=1 bash tools/run_dqn_model.sh       # no Gazebo window
#   EPISODES=1 bash tools/run_dqn_model.sh       # trials per goal (default 3)
#   OUT_DIR=/some/dir bash tools/run_dqn_model.sh
#
# Results (new files every run, nothing in the repo is overwritten): ~/dqn_eval_results/dqn_eval_<time>.csv / .log
# The model reached 0 of 9 goals when it was evaluated (see README): this reproduces that evaluation, it is not a
# working navigation policy. Run tools/setup.sh once first. Refuses to run while Nav2 is running (both drive /cmd_vel).
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
MODEL="$REPO/ros2_ws/src/dqn_agent/models/dqn_maze_none_60ep_0of9.npz"
[ -f "$MODEL" ] || { echo "model not found: $MODEL (git pull / checkout dev-integration)"; exit 1; }
# Nav2 and the DQN both publish /cmd_vel: never run them together.
if pgrep -f "lib/nav2_|tools/[d]emo.sh|tools/[a]uto.sh|[t]rain_dqn|[e]valuate_dqn" >/dev/null; then
  echo "Nav2 (tools/auto.sh) or another DQN run is already running. Stop it first (Ctrl+C in its terminal), then run this again."
  exit 3
fi

[ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy not found (docs/installation.md)"; exit 1; }
[ -f "$REPO/ros2_ws/install/setup.bash" ] || { echo "workspace not built: run  bash tools/setup.sh  first"; exit 1; }

set +u; source /opt/ros/jazzy/setup.bash; source "$REPO/ros2_ws/install/setup.bash"; set -u
export TURTLEBOT3_MODEL=burger FASTDDS_BUILTIN_TRANSPORTS=UDPv4
TS="$(date +%Y%m%d-%H%M%S)"
OUT="${OUT_DIR:-$HOME/dqn_eval_results}"
mkdir -p "$OUT" || exit 1

SIM=""
if pgrep -f "[g]z sim" >/dev/null; then
  echo ">>> using the Gazebo simulation that is already running (it will be left running)"
else
  GUI=true; [ -n "${HEADLESS:-}" ] && GUI=false
  echo ">>> starting the Gazebo maze (gui:=$GUI), log: $OUT/simulation_$TS.log"
  # own process group (setsid): "ros2 launch" ignores SIGINT as a plain background job and leaves Gazebo running,
  # so on exit we stop exactly this group and nothing else
  setsid ros2 launch maze_simulation maze_simulation.launch.py gui:=$GUI > "$OUT/simulation_$TS.log" 2>&1 &
  SIM=$!
fi
cleanup() {
  if [ -n "$SIM" ]; then
    kill -TERM -- "-$SIM" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8; do kill -0 -- "-$SIM" 2>/dev/null || break; sleep 1; done
    kill -KILL -- "-$SIM" 2>/dev/null
    wait "$SIM" 2>/dev/null
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

echo ">>> waiting for the simulation (/scan) ..."
UP=0
for _ in $(seq 1 60); do timeout 8 ros2 topic echo --once /scan >/dev/null 2>&1 && { UP=1; break; }; sleep 2; done
[ "$UP" = 1 ] || { echo "the simulation did not start (see $OUT/simulation_$TS.log)"; exit 1; }
# Gazebo can come up paused; make sure it is running
gz service -s /world/custom_maze/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'pause: false' >/dev/null 2>&1

echo ">>> evaluating $MODEL"
ros2 run dqn_agent evaluate_dqn --checkpoint "$MODEL" --episodes "${EPISODES:-3}" --guidance none \
  --csv "$OUT/dqn_eval_$TS.csv" 2>&1 | grep --line-buffered -v libEGL | tee "$OUT/dqn_eval_$TS.log"
RC=${PIPESTATUS[0]}

if [ -f "$OUT/dqn_eval_$TS.csv" ]; then
  python3 - "$OUT/dqn_eval_$TS.csv" <<'EOF'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
n = len(rows)
ok = sum(r["success"] == "True" for r in rows)
col = sum(r["collision"] == "True" for r in rows)
print(f"\nRESULT: {ok} of {n} trials reached the goal, {col} collisions, {n - ok - col} timeouts")
EOF
fi
echo "CSV: $OUT/dqn_eval_$TS.csv"
echo "log: $OUT/dqn_eval_$TS.log"
exit "$RC"
