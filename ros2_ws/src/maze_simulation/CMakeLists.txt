cmake_minimum_required(VERSION 3.8)
project(maze_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)
install(FILES FIGURE1_README.md generate_maze.py
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/maze_tools.py scripts/check_sim_topics.py scripts/check_control.py scripts/cmd_vel_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
