cmake_minimum_required(VERSION 3.5)
project(open_cr_module)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_math REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${robotis_controller_msgs_INCLUDE_DIRS}
  ${robotis_framework_common_INCLUDE_DIRS}
  ${robotis_device_INCLUDE_DIRS}
  ${robotis_math_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME} SHARED src/open_cr_module.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  dynamixel_sdk
  robotis_controller_msgs
  robotis_framework_common
  robotis_device
  robotis_math
  Eigen3
)

# Add executable node
add_executable(open_cr_node src/open_cr_node.cpp)
ament_target_dependencies(open_cr_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  dynamixel_sdk
  robotis_controller_msgs
  robotis_framework_common
  robotis_device
  robotis_math
  Eigen3
)
target_link_libraries(open_cr_node ${PROJECT_NAME})

# Add reader node
add_executable(open_cr_reader src/open_cr_reader.cpp)
ament_target_dependencies(open_cr_reader
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Install
install(TARGETS ${PROJECT_NAME} open_cr_node open_cr_reader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs sensor_msgs dynamixel_sdk robotis_controller_msgs robotis_framework_common robotis_device robotis_math Eigen3)

# Ament package
ament_package()
