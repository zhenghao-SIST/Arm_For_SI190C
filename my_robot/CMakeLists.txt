cmake_minimum_required(VERSION 3.8)
project(my_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY urdf meshes launch rviz models worlds config
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes into the model directory so Gazebo can resolve
# model://my_robot/meshes/... URIs in model.sdf
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/models/${PROJECT_NAME}/meshes
)

# Install Python scripts
install(PROGRAMS
  scripts/aruco_detector.py
  scripts/generate_aruco.py
  scripts/joint_command_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Gazebo custom plugins
add_subdirectory(gz_plugins)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
