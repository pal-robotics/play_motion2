cmake_minimum_required(VERSION 3.8)
project(play_motion2_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Motion.msg"
  "srv/AddMotion.srv"
  "srv/GetMotionInfo.srv"
  "srv/IsMotionReady.srv"
  "srv/IsJointListReady.srv"
  "srv/ListMotions.srv"
  "srv/RemoveMotion.srv"
  "srv/SaveMotion.srv"
  "action/PlayMotion2.action"
  "action/PlayMotion2Raw.action"
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
