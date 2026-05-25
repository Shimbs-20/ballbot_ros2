cmake_minimum_required(VERSION 3.8)
project(ballbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_python_install_package(${PROJECT_NAME})

include_directories(
  include
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs Eigen3)

add_executable(twist_relay src/twist_relay.cpp)
ament_target_dependencies(twist_relay rclcpp geometry_msgs std_msgs)

add_executable(lqr src/lqr.cpp)

ament_target_dependencies(lqr rclcpp sensor_msgs geometry_msgs std_msgs nav_msgs tf2_ros tf2_geometry_msgs
)


install (DIRECTORY include
  DESTINATION include
)


install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  ${PROJECT_NAME}/simple_controller.py
  ${PROJECT_NAME}/twist_relay.py
  ${PROJECT_NAME}/ballbot_omnni.py
  ${PROJECT_NAME}/fake_odom.py
  ${PROJECT_NAME}/stm32_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  simple_controller
  twist_relay
  lqr
  DESTINATION lib/${PROJECT_NAME}
  )


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_simple_controller test/test_simple_controller.py
    TIMEOUT 60
  )

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
