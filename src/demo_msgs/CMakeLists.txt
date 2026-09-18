cmake_minimum_required(VERSION 3.18)
project(demo_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(messages
  "msg/Line.msg"
  "msg/LineSegment.msg"
  "msg/LSDandImage.msg"
  "msg/LanePoints.msg"
  "msg/LanePoint.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${messages}
  DEPENDENCIES std_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime rosidl_default_generators std_msgs sensor_msgs)

ament_package()
