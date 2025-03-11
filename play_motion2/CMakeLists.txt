cmake_minimum_required(VERSION 3.5)
project(play_motion2)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# play_motion2
ament_auto_add_library(play_motion2 SHARED
  src/play_motion2/motion_loader.cpp
  src/play_motion2/motion_planner.cpp
  src/play_motion2/play_motion2.cpp
)

# play_motion2 client
ament_auto_add_library(client
  src/play_motion2/client.cpp
)

# play_motion2_node
ament_auto_add_executable(play_motion2_node
  src/play_motion2_main.cpp
)

# run_motion
ament_auto_add_executable(run_motion
  src/run_motion.cpp
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  # rrbot_system
  ament_auto_add_library(rrbot_system SHARED
    test/rrbot/rrbot_system.cpp
  )
  # need to target because they are test dependencies
  ament_target_dependencies(rrbot_system
    hardware_interface
    pluginlib
  )
  pluginlib_export_plugin_description_file(hardware_interface test/rrbot/rrbot_system.xml)

  # passthrough_controller
  generate_parameter_library(passthrough_controller_parameters
    test/controllers/src/passthrough_controller_parameters.yaml
  )
  add_library(passthrough_controller SHARED
    test/controllers/src/passthrough_controller.cpp
  )

  add_compile_definitions(HARDWARE_INTERFACE_VERSION_MAJOR=${hardware_interface_VERSION_MAJOR})
  add_compile_definitions(GENERATE_PARAMETER_LIBRARY_VERSION_MINOR=${generate_parameter_library_VERSION_MINOR})
  target_link_libraries(passthrough_controller PUBLIC passthrough_controller_parameters)
  ament_target_dependencies(passthrough_controller PUBLIC controller_interface realtime_tools)
  pluginlib_export_plugin_description_file(controller_interface test/controllers/passthrough_controller.xml)

  install(
    TARGETS passthrough_controller
    DESTINATION lib
  )

  # motion_loader_test
  ament_auto_add_gtest(motion_loader_test
    test/motion_loader_test.cpp
  )

  # play_motion2_test
  ament_auto_add_gtest(play_motion2_test
    test/play_motion2_test.cpp
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
    sensor_msgs
  )

  install(
    TARGETS play_motion2_node_test
    DESTINATION lib/${PROJECT_NAME}
  )

  add_launch_test(
    test/play_motion2_node_test.launch.py
  )
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
