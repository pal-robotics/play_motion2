cmake_minimum_required(VERSION 3.5)
project(play_motion2)

find_package(ament_cmake REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)

find_package(rclcpp REQUIRED)

set(dependencies
  rclcpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/play_motion2/play_motion2.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(play_motion2_main
  src/play_motion2_main.cpp
)

ament_target_dependencies(play_motion2_main ${dependencies})
target_link_libraries(play_motion2_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  play_motion2_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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
