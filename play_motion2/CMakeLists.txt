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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  set(test_dependencies
    ament_cmake_gtest
    ament_index_cpp
  )

  # test configuration files
  install(
    FILES
      test/config.yaml
      test/play_motion2_config.yaml
    DESTINATION share/${PROJECT_NAME}/test
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
  ament_target_dependencies(play_motion2_test ${test_dependencies})
  target_link_libraries(play_motion2_test
    play_motion2
  )

endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
