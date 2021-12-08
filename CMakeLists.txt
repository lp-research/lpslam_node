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
find_package(yaml_cpp_vendor REQUIRED)
if (USE_OPENVSLAM_DIRECTLY)
  find_package(message_filters REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(cv_bridge REQUIRED)
  if (NOT LPSLAM_BUILD_OPENVSLAM)
    # This means that openvslam is built separately
    find_package(openvslam REQUIRED)
  endif()
else()
  find_package(lpslam_interfaces REQUIRED)
endif()

SET(LPSLAM_BUILD_ROS2 ON CACHE BOOL "build with ROS2 support")
SET(LPSLAM_BUILD_LPSLAM ON CACHE BOOL "Build LPSLAM module")
SET(LPSLAM_BUILD_OPENVSLAM ON CACHE BOOL "enable")
SET(LPSLAM_BUILD_OPENVSLAM_PANGOLIN ON CACHE BOOL "enable")
SET(USE_PANGOLIN_VIEWER OFF CACHE BOOL "Use Pangolin Viewer")
SET(USE_OPENVSLAM_DIRECTLY OFF CACHE BOOL "Build with direct usage of OpenVSLAM (w/o lpslam module)")

if (LPSLAM_BUILD_LPSLAM)
  add_subdirectory(lpslam)
endif()

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
SET(LPSPAM_NODE_SRC src/LpBaseNode.cpp)
if(USE_OPENVSLAM_DIRECTLY)
  LIST(APPEND LPSPAM_NODE_SRC src/OpenVSLAMNode.cpp)
else()
  LIST(APPEND LPSPAM_NODE_SRC src/LpSlamNode.cpp)
endif()
LIST(APPEND LPSPAM_NODE_SRC src/main.cpp)

add_executable(lpslam_node
  ${LPSPAM_NODE_SRC}
)

SET(LPSPAM_NODE_LIBS
  ${OPENZEN_TARGET_NAME}
)

if(USE_OPENVSLAM_DIRECTLY)
  LIST(APPEND LPSPAM_NODE_LIBS openvslam)
  target_include_directories(lpslam_node
    PRIVATE
    lpslam/src/Interface)

  if(USE_PANGOLIN_VIEWER)
    LIST(APPEND LPSPAM_NODE_LIBS pangolin_viewer)
    if (NOT LPSLAM_BUILD_LPSLAM)
      # If build lpslam with conan, it will add pangolin dependency.
      # Otherwise, adding it manually
      LIST(APPEND LPSPAM_NODE_LIBS pangolin)
    endif()
  endif()
else()
  LIST(APPEND LPSPAM_NODE_LIBS lpslam)
endif()

target_link_libraries(lpslam_node
  ${LPSPAM_NODE_LIBS}
)

SET(LPSPAM_NODE_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml_cpp_vendor
)

if(USE_OPENVSLAM_DIRECTLY)
  LIST(APPEND LPSPAM_NODE_DEPS message_filters builtin_interfaces cv_bridge)
  target_compile_definitions(lpslam_node PRIVATE -DUSE_OPENVSLAM_DIRECTLY)
else()
  LIST(APPEND LPSPAM_NODE_DEPS lpslam_interfaces)
endif()

if(USE_PANGOLIN_VIEWER)
  target_compile_definitions(lpslam_node PRIVATE -DUSE_PANGOLIN_VIEWER)
endif()

ament_target_dependencies(lpslam_node
  ${LPSPAM_NODE_DEPS})

install(TARGETS lpslam_node
  DESTINATION lib/${PROJECT_NAME})
if (LPSLAM_BUILD_LPSLAM)
  install(TARGETS lpslam
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
