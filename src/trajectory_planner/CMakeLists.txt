cmake_minimum_required(VERSION 3.18)
project(trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(demo_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(trajectory_planner_node SHARED
  src/trajectory_planner_node.cpp
)

target_include_directories(trajectory_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(trajectory_planner_node PUBLIC cxx_std_17)

target_link_libraries(trajectory_planner_node PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${demo_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(trajectory_planner_node
  PLUGIN "TrajectoryPlanner"
  EXECUTABLE "trajectory_planner"
)

ament_export_targets(export_trajectory_planner_node)

install(
  TARGETS trajectory_planner_node
  EXPORT export_trajectory_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()