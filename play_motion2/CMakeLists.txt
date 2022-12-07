cmake_minimum_required(VERSION 3.5)
project(play_motion2)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)

find_package(controller_manager_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(play_motion2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(dependencies
  controller_manager_msgs
  lifecycle_msgs
  play_motion2_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  trajectory_msgs
)

include_directories(include)

# play_motion2
add_library(play_motion2 SHARED
  src/play_motion2/play_motion2_helpers.cpp
  src/play_motion2/play_motion2.cpp
)

ament_target_dependencies(play_motion2 ${dependencies})

install(TARGETS play_motion2
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# play_motion2_node
add_executable(play_motion2_node
  src/play_motion2_main.cpp
)

ament_target_dependencies(play_motion2_node ${dependencies})

target_link_libraries(play_motion2_node
  play_motion2
)
install(TARGETS play_motion2_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  find_package(controller_manager_msgs REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(pluginlib REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # test configuration files
  install(
    FILES
      test/config.yaml
      test/play_motion2_config.yaml
      test/rrbot/controller_manager.yaml
      test/rrbot/controllers.launch.py
      test/rrbot/joint_state_broadcaster.launch.py
      test/rrbot/joint_state_broadcaster.yaml
      test/rrbot/joint_trajectory_controller.launch.py
      test/rrbot/joint_trajectory_controller.yaml
      test/rrbot/rrbot.launch.py
      test/rrbot/rrbot.rviz
      test/rrbot/rrbot.xacro
    DESTINATION share/${PROJECT_NAME}/test
  )

  # rrbot_system
  add_library(rrbot_system SHARED
    test/rrbot/rrbot_system.cpp
  )
  target_include_directories(rrbot_system PRIVATE
    include
  )
  ament_target_dependencies(rrbot_system
    hardware_interface
    pluginlib
    rclcpp
  )
  pluginlib_export_plugin_description_file(hardware_interface test/rrbot/rrbot_system.xml)
  install(
    TARGETS rrbot_system
    DESTINATION lib
  )

  # play_motion2_helpers_test
  ament_add_gtest(play_motion2_helpers_test
    test/play_motion2_helpers_test.cpp
  )
  ament_target_dependencies(play_motion2_helpers_test ${test_dependencies})
  target_link_libraries(play_motion2_helpers_test
    play_motion2
  )

  # play_motion2_test
  ament_add_gtest(play_motion2_test
    test/play_motion2_test.cpp
  )
  ament_target_dependencies(play_motion2_test
    ament_cmake_gtest
    ament_index_cpp
  )
  target_link_libraries(play_motion2_test
    play_motion2
  )

  # play_motion2_node_test
  ament_add_gtest_executable(play_motion2_node_test
    test/play_motion2_node_test.cpp
  )
  ament_target_dependencies(play_motion2_node_test
    controller_manager_msgs
    launch_testing_ament_cmake
    play_motion2_msgs
    rclcpp
    rclcpp_action
  )

  install(
    TARGETS play_motion2_node_test
    DESTINATION share/${PROJECT_NAME}/test
  )

  add_launch_test(
    test/play_motion2_node_test.launch.py
  )

endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
