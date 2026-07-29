cmake_minimum_required(VERSION 3.16)
project(direct_joint_position_plugin)

# Gazebo Harmonic (gz-sim8) packages
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)

set(PLUGIN_NAME DirectJointPositionPlugin)

add_library(${PLUGIN_NAME} SHARED ${PLUGIN_NAME}.cpp)
target_link_libraries(${PLUGIN_NAME}
  PUBLIC
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-plugin2::gz-plugin2
)
target_include_directories(${PLUGIN_NAME}
  PUBLIC
    ${gz-sim8_INCLUDE_DIRS}
)

# Install the plugin
install(TARGETS ${PLUGIN_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Tell Gazebo where to find our plugin
install(DIRECTORY DESTINATION share/gz/gz-sim-plugins)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/direct_joint_position_plugin.xml"
  "<plugin filename=\"lib${PLUGIN_NAME}.so\" name=\"my::DirectJointPositionPlugin\"/>")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/direct_joint_position_plugin.xml"
  DESTINATION share/gz/gz-sim-plugins)
