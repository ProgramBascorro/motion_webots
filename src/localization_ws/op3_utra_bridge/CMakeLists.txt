cmake_minimum_required(VERSION 3.8)
project(op3_utra_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(soccer_msgs REQUIRED)
find_package(op3_kinematics_dynamics REQUIRED)

# Include directories
include_directories(
  include
)

# C++ Odometry Bridge Node
add_executable(odom_bridge_node 
  src/odom_bridge_node.cpp
)

ament_target_dependencies(odom_bridge_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  op3_kinematics_dynamics
)

# Install C++ executables
install(TARGETS
  odom_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts from scripts/ directory
install(PROGRAMS
  scripts/robot_state_publisher.py
  scripts/imu_bridge.py
  scripts/odom_path_publisher.py
  scripts/odom_to_tf.py
  scripts/op3_static_transforms.py
  scripts/odom_publisher_static.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (if exists)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install RViz config files
install(DIRECTORY 
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()