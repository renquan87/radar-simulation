cmake_minimum_required(VERSION 3.5)
project(radar_sim)

find_package(ament_cmake REQUIRED)

# Install launch files, config, rviz, and worlds
install(DIRECTORY
  launch
  config
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
