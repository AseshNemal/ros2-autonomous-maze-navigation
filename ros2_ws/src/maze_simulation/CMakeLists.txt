cmake_minimum_required(VERSION 3.8)
project(maze_simulation)

find_package(ament_cmake REQUIRED)

# Resource-only package for now: no nodes or libraries are built yet.
install(DIRECTORY
  launch
  worlds
  models
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
