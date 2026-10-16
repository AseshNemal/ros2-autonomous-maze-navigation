#!/usr/bin/env bash
# Runs every verification stage in order and prints a summary. One command for the whole project.
# Continues past a failed stage so you see all results. Logs: docs/results/vm/
cd "$(dirname "$0")/.."
# install dependencies on first run (skipped once ros2 run exists)
set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null; set -u
ros2 run -h >/dev/null 2>&1 || bash tools/install_deps_ubuntu.sh
declare -A RES
for s in build sim slam nav dqn; do
  echo "================ STAGE: $s ================"
  # nav needs the SLAM map only if the slam stage produced one; otherwise use the ground-truth map
  if bash tools/verify_on_vm.sh "$s"; then RES[$s]=done; else RES[$s]=ERROR; fi
done
echo; echo "================ SUMMARY (check PASS/FAIL lines in docs/results/vm/*.log) ================"
for s in build sim slam nav dqn; do echo "$s: ${RES[$s]}"; done
grep -hE "^(PASS|FAIL)" docs/results/vm/*.log docs/results/vm/*.txt 2>/dev/null | sort | uniq -c | sort -rn | head -40
