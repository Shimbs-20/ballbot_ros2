cmake_minimum_required(VERSION 3.8)
project(ballbot_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

include_directories(
  include
)

add_executable(mapping_with_known_poses src/mapping_with_known_poses.cpp)
ament_target_dependencies(mapping_with_known_poses rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs tf2)

install(PROGRAMS
  ${PROJECT_NAME}/mapping_with_known_poses.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mapping_with_known_poses
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  maps
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install (DIRECTORY include
  DESTINATION include
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
