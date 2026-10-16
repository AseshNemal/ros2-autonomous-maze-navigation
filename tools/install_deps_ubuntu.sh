#!/usr/bin/env bash
# Ubuntu 24.04 + ROS 2 Jazzy dependencies for all four packages (run once on the VM).
set -euo pipefail
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-description \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-mppi-controller \
  ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rviz2 ros-jazzy-ros2run ros-jazzy-ros2node ros-jazzy-ros2lifecycle ros-jazzy-ros2param ros-jazzy-ros2service python3-colcon-common-extensions python3-numpy python3-yaml python3-pytest
grep -q TURTLEBOT3_MODEL ~/.bashrc || echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
