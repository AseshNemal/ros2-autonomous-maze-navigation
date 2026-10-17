cmake_minimum_required(VERSION 3.8)
project(slam_mapping)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/auto_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
