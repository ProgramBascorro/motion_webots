cmake_minimum_required(VERSION 3.8)
project(tools_fit)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_math REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(op3_kinematics_dynamics REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${robotis_math_INCLUDE_DIRS}
  ${robotis_framework_common_INCLUDE_DIRS}
  ${op3_kinematics_dynamics_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_executable(fit_walking_ready fit_walking_ready.cpp)
ament_target_dependencies(fit_walking_ready
  rclcpp robotis_math robotis_framework_common op3_kinematics_dynamics)
install(TARGETS fit_walking_ready DESTINATION lib/${PROJECT_NAME})
ament_package()
