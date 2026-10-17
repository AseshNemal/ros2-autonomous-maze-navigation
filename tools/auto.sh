#!/usr/bin/env bash
# AUTOMATIC NAVIGATION: Gazebo window + RViz + Nav2. One command:
#
#   bash tools/auto.sh
#
# Wait for ">>> Nav2 is ACTIVE", then in the RViz window press "2D Goal Pose" (top toolbar), click a point on
# the map and drag to set the direction: the robot drives there by itself. Clicking in the Gazebo window does nothing.
# No mouse? In a second terminal:  bash tools/send_goal.sh G1   (or G2, G3, or:  bash tools/send_goal.sh 1.25 0.75)
# Ctrl+C stops everything.
exec bash "$(dirname "$0")/demo.sh"
