cmake_minimum_required(VERSION 3.20)
project(mission_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mission_controller src/mission_controller.cpp)
target_include_directories(mission_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(mission_controller PUBLIC c_std_99 cxx_std_17)

target_link_libraries(mission_controller PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

install(TARGETS mission_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
