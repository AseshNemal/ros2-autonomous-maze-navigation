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
  scripts/run_three_goals.py
  scripts/twist_stamper.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config test/test_config.py)
endif()

ament_package()
