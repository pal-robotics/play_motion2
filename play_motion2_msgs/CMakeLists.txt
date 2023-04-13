cmake_minimum_required(VERSION 3.5)
project(play_motion2_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlayMotion2.action"
  "srv/IsMotionReady.srv"
  "srv/ListMotions.srv"
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
