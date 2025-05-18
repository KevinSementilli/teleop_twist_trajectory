cmake_minimum_required(VERSION 3.8)
project(teleop_twist_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(trajectory_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
)

add_executable(teleop_twist_trajectory 
  src/twist_trajectory.cpp
  src/main.cpp
)

ament_target_dependencies(
  teleop_twist_trajectory 
  rclcpp
  kdl_parser
  orocos_kdl
  trajectory_msgs
  geometry_msgs
)

install(TARGETS
  teleop_twist_trajectory
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
