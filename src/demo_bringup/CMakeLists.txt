cmake_minimum_required(VERSION 3.18)
project(demo_bringup)

# suppress unused variable warning for non-compiling packages
set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
    params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
