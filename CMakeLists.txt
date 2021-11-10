cmake_minimum_required(VERSION 3.5)
project(lpslam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lpslam_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

SET(LPSLAM_BUILD_ROS2 ON CACHE BOOL "build with ROS2 support")
SET(LPSLAM_BUILD_OPENVSLAM ON CACHE BOOL "enable")
SET(LPSLAM_BUILD_OPENVSLAM_PANGOLIN ON CACHE BOOL "enable")
add_subdirectory(lpslam)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## Build 
add_executable(lpslam_node
  src/LpSlamNode.cpp
)

target_link_libraries(lpslam_node
  ${OPENZEN_TARGET_NAME}
  lpslam
)

ament_target_dependencies(lpslam_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  lpslam_interfaces)

install(TARGETS lpslam_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS lpslam
  DESTINATION lib/${PROJECT_NAME})

ament_package()
