cmake_minimum_required(VERSION 3.5)

project(icai_crl_bringup_sim)

find_package(ament_cmake REQUIRED)
find_package(icai_crl_description REQUIRED)
find_package(icai_crl_gazebo REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()