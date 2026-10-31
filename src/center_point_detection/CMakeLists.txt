cmake_minimum_required(VERSION 3.18)
project(center_point_detection)

cmake_policy(SET CMP0167 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(demo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(Boost CONFIG REQUIRED)

include_directories(include)

add_library(center_point_detection_node SHARED
  src/center_point_detection_node.cpp
  src/lane_center_detector.cpp
  src/dbscan.cpp
)

target_link_libraries(center_point_detection_node PUBLIC
  Boost::headers
  ${demo_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  rclcpp_components::component
  sensor_msgs::sensor_msgs_library
)

target_include_directories(center_point_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(center_point_detection_node PUBLIC cxx_std_17) # Require C99 and C++17

rclcpp_components_register_node(center_point_detection_node
  PLUGIN "CenterPointDetection"
  EXECUTABLE "center_point_detection"
)

ament_export_targets(export_center_point_detection_node)

install(
  TARGETS center_point_detection_node
  EXPORT export_center_point_detection_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS center_point_detection_node
  DESTINATION lib/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
