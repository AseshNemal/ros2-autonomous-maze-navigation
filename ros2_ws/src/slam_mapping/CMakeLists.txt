cmake_minimum_required(VERSION 3.8)
project(slam_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/save_map.sh
  scripts/validate_map.py
  scripts/auto_map_drive.py
  scripts/set_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_validate_map test/test_validate_map.py)
endif()

ament_package()
