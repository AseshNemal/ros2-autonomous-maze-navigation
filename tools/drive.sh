#!/usr/bin/env bash
# Command-line alternative to keyboard teleop (use when remote-desktop keyboard focus is unreliable).
# Needs the simulation to be running; do NOT run it together with Nav2, SLAM tour or DQN (same /cmd_vel).
#
#   bash tools/drive.sh forward 2     # drive forward for 2 s (default 1 s), then stop
#   bash tools/drive.sh back 1
#   bash tools/drive.sh left 1        # turn left in place
#   bash tools/drive.sh right 1
#   bash tools/drive.sh stop
set -uo pipefail
set +u; source /opt/ros/jazzy/setup.bash; set -u
CMD="${1:-}"; SECS="${2:-1}"
case "$CMD" in
  forward) LX=0.15; AZ=0.0 ;;
  back)    LX=-0.15; AZ=0.0 ;;
  left)    LX=0.0; AZ=0.5 ;;
  right)   LX=0.0; AZ=-0.5 ;;
  stop)    LX=0.0; AZ=0.0; SECS=0.3 ;;
  *) sed -n 2,11p "$0"; exit 2 ;;
esac
N=$(python3 -c "print(max(1, int(float('$SECS') * 10)))")
pub() { ros2 topic pub --times "$1" --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
        "{header: {frame_id: base_link}, twist: {linear: {x: $2}, angular: {z: $3}}}" >/dev/null; }
echo "$CMD for $SECS s ..."
pub "$N" "$LX" "$AZ"
pub 3 0.0 0.0          # always finish with an explicit stop
echo "stopped"
