cmake_minimum_required(VERSION 3.8)
project(play_motion2)

add_compile_options(-Wno-cpp)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()
add_compile_definitions(MOVEIT_VERSION_MAJOR=${moveit_ros_planning_interface_VERSION_MAJOR})
add_compile_definitions(MOVEIT_VERSION_MINOR=${moveit_ros_planning_interface_VERSION_MINOR})
add_compile_definitions(RCLCPP_VERSION_MAJOR=${rclcpp_VERSION_MAJOR})

ament_auto_add_executable(play_motion2_node
  src/play_motion2_main.cpp
  src/play_motion2/play_motion2_executor.cpp
  src/play_motion2/play_motion2_mgr.cpp
  src/utils/motion_planner.cpp
  src/utils/motion_loader.cpp
)

# play_motion2 client
ament_auto_add_library(client SHARED
  src/play_motion2/client.cpp
)

# run_motion
ament_auto_add_executable(run_motion
  src/run_motion.cpp
)

# Play Motion 2 Client Python
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  # rrbot_system
  ament_auto_add_library(rrbot_system SHARED
    test/rrbot/rrbot_system.cpp
  )
  # need to target because they are test dependencies
  target_link_libraries(rrbot_system
    hardware_interface::hardware_interface
    hardware_interface::mock_components
    pluginlib::pluginlib
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
  target_link_libraries(passthrough_controller
    passthrough_controller_parameters
    controller_interface::controller_interface
    realtime_tools::realtime_tools
    realtime_tools::thread_priority
  )
  pluginlib_export_plugin_description_file(controller_interface test/controllers/passthrough_controller.xml)

  install(
    TARGETS passthrough_controller
    DESTINATION lib
  )

  # motion_loader_test
  ament_auto_add_gtest(motion_loader_test
    test/motion_loader_test.cpp
    src/utils/motion_loader.cpp
  )

  # play_motion2_node_test
  ament_add_gtest_executable(play_motion2_node_test
    test/play_motion2_node_test.cpp
  )
  target_link_libraries(play_motion2_node_test
    ${controller_manager_msgs_TARGETS}
    ${play_motion2_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    sensor_msgs::sensor_msgs_library
  )

  install(
    TARGETS play_motion2_node_test
    DESTINATION lib/${PROJECT_NAME}
  )

  add_launch_test(
    test/play_motion2_node_test.launch.py
  )

  # Python tests
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_client.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

if(ament_cmake_auto_VERSION VERSION_LESS "2.6.0")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE config launch)
else()
  ament_auto_package(INSTALL_TO_SHARE config launch)
endif()
