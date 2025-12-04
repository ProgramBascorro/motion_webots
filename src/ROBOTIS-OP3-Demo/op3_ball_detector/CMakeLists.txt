cmake_minimum_required(VERSION 3.8)
project(op3_ball_detector)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Find OpenCV
find_package(OpenCV 4.9 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 4 REQUIRED)
endif()

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ============ C++ Implementation ============

# YOLO detector library
add_library(yolo_detector_lib SHARED
  src/yolo_detector.cpp
)

ament_target_dependencies(yolo_detector_lib
  OpenCV
)

target_link_libraries(yolo_detector_lib
  ${OpenCV_LIBRARIES}
)

# Ball detector library
add_library(ball_detector_lib SHARED
  src/ball_detector.cpp
)

ament_target_dependencies(ball_detector_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
  ament_index_cpp
)

target_link_libraries(ball_detector_lib
  yolo_detector_lib
  ${OpenCV_LIBRARIES}
)

# Ball detector node executable
add_executable(ball_detector_node
  src/ball_detector_node.cpp
)

ament_target_dependencies(ball_detector_node
  rclcpp
)

target_link_libraries(ball_detector_node
  ball_detector_lib
)

# Install C++ targets
install(TARGETS
  yolo_detector_lib
  ball_detector_lib
  ball_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(yolo_detector_lib ball_detector_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
  ament_index_cpp
)

ament_package()