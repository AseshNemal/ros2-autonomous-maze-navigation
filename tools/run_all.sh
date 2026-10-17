#!/usr/bin/env bash
# Runs the verification stages in order and prints a summary with the time each one took.
# Continues past a failed stage so you see all results. Logs: docs/results/vm/
#
#   bash tools/run_all.sh                          # every stage (long: DQN training dominates)
#   STAGES="sim control" bash tools/run_all.sh     # only some stages  (build sim control slam nav dqn)
#   QUICK=1 bash tools/run_all.sh                  # short smoke test of every stage (partial SLAM tour, 10 DQN episodes)
#   bash tools/verify_on_vm.sh nav                 # one stage on its own
# Stage lengths (real time, simulation runs at 1x): build ~1 min, sim ~0.5, control ~1.5, nav ~3,
# slam ~7 (full tour; QUICK ~2), dqn ~ episodes x 8 s per mode (QUICK ~3).
cd "$(dirname "$0")/.."
# install dependencies on first run (skipped once ros2 run exists)
set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null; set -u
ros2 run -h >/dev/null 2>&1 || bash tools/install_deps_ubuntu.sh
STAGES="${STAGES:-build sim control slam nav dqn}"
N=$(echo $STAGES | wc -w); I=0
declare -A RES TIME
for s in $STAGES; do
  I=$((I + 1)); T=$SECONDS
  echo "================ STAGE $I/$N: $s ================  (log: docs/results/vm/${s}_*.log)"
  if bash tools/verify_on_vm.sh "$s"; then RES[$s]=PASS; else RES[$s]=FAIL; fi
  TIME[$s]=$((SECONDS - T))
done
echo; echo "================ SUMMARY ================"
for s in $STAGES; do printf "%-8s %-5s %5s s\n" "$s" "${RES[$s]}" "${TIME[$s]}"; done
echo "total $SECONDS s"
echo "(per-check PASS/FAIL lines are in the stage logs under docs/results/vm/)"
