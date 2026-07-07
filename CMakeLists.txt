cmake_minimum_required(VERSION 3.8)
project(docksimulation_Hypmotion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(cv_bridge REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)

# ============================================================
# ArUco detector node
# Chỉ detect marker và publish ID + ảnh đánh dấu
# ============================================================

add_executable(
  aruco_detector
  src/aruco_detector.cpp
)

ament_target_dependencies(
  aruco_detector
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_include_directories(
  aruco_detector
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  aruco_detector
  ${OpenCV_LIBS}
)

# ============================================================
# ArUco pose node
# Tính vị trí, hướng, publish PoseArray, MarkerArray và TF
# ============================================================

add_executable(
  aruco_pose_node
  src/aruco_pose_node.cpp
)

ament_target_dependencies(
  aruco_pose_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  tf2
  tf2_ros
)

target_include_directories(
  aruco_pose_node
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  aruco_pose_node
  ${OpenCV_LIBS}
)

# ============================================================
# Install executables
# ============================================================

install(
  TARGETS
    aruco_detector
    aruco_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Install resources
# ============================================================

install(
  DIRECTORY
    launch
    config
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Testing
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()