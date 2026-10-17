#!/usr/bin/env bash
# AUTOMATIC NAVIGATION: Gazebo window + RViz + Nav2. One command:
#
#   bash tools/auto.sh
#
# Wait for ">>> Nav2 is ACTIVE", then in the RViz window press "Nav2 Goal" (top toolbar), click a point on
# the map and drag to set the direction: the robot drives there by itself. Clicking in the Gazebo window does nothing.
# No mouse? In a second terminal:  bash tools/send_goal.sh G1   (or G2, G3, or:  bash tools/send_goal.sh 1.25 0.75)
# Do NOT click "2D Pose Estimate": it moves where Nav2 believes the robot is (RViz and Gazebo then disagree and goals abort).
# If that happened:  bash tools/reset_robot.sh   (puts the robot and Nav2 back at the spawn pose)
# Ctrl+C stops everything.
exec bash "$(dirname "$0")/demo.sh"
