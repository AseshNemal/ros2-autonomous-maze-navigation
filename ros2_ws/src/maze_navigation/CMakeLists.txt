cmake_minimum_required(VERSION 3.8)
project(maze_navigation)

find_package(ament_cmake REQUIRED)

# Resource-only package for now: no nodes or libraries are built yet.
install(DIRECTORY
  launch
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/evaluate_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
