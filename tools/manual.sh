#!/usr/bin/env bash
# MANUAL / REMOTE CONTROL: Gazebo window + keyboard driving. One command:
#
#   bash tools/manual.sh
#
# Keep THIS terminal focused (keys typed in the Gazebo window do nothing) and press:
#   i forward   , backward   j turn left   l turn right   k or space stop
#   u/o forward+left/right   m/. backward+left/right   z slow down   q speed up   Ctrl+C quits
# Press z a few times first: the default speed is fast for this maze.
exec bash "$(dirname "$0")/demo.sh" teleop
