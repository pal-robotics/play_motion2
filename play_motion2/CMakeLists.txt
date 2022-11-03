cmake_minimum_required(VERSION 3.5)
project(play_motion2)

find_package(ament_cmake REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)

find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(play_motion2_msgs REQUIRED)

set(dependencies
  rclcpp
  trajectory_msgs
  play_motion2_msgs
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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
#  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
