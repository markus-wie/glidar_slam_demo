cmake_minimum_required(VERSION 3.18)

project(image_processing LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-pedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(demo_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(NODE_NAME "image_processing_node")

include_directories(include)

add_library(${NODE_NAME} SHARED
  src/image_processing_node.cpp
)

set_target_properties(${NODE_NAME} PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${NODE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${NODE_NAME} PUBLIC
  ${demo_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
)

rclcpp_components_register_node(${NODE_NAME} PLUGIN "image_processing::ImageProcessingNode" EXECUTABLE "image_processing")

ament_export_targets(export_${NODE_NAME})

install(
  TARGETS ${NODE_NAME}
  EXPORT export_${NODE_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${NODE_NAME}
  DESTINATION lib/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
