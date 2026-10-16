cmake_minimum_required(VERSION 3.8)
project(maze_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/maze_tools.py scripts/check_sim_topics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
