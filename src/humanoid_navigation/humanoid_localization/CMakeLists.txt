cmake_minimum_required(VERSION 3.8)
project(humanoid_localization)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================
# Dependencies
# ============================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(orocos_kdl_vendor REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)

find_package(OpenMP REQUIRED)

# ============================
# Optional SBPL
# ============================
find_package(SBPL QUIET)
if(SBPL_FOUND)
  add_compile_definitions(USE_SBPL)
  message(STATUS "SBPL found: building with SBPL functionality")
else()
  message(STATUS "SBPL not found: building without SBPL functionality")
endif()

set(PKG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ============================
# Libraries
# ============================

add_library(transform_utils src/transform_utils.cpp)
target_include_directories(transform_utils PUBLIC
  $<BUILD_INTERFACE:${PKG_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(transform_utils
  tf2 tf2_ros tf2_kdl tf2_eigen tf2_geometry_msgs
)

add_library(mapmodel src/MapModel.cpp)
target_include_directories(mapmodel PUBLIC
  $<BUILD_INTERFACE:${PKG_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mapmodel
  rclcpp sensor_msgs tf2 Eigen3
  octomap octomap_msgs
  pcl_conversions
)
target_link_libraries(mapmodel
  octomap
  octomath
)

add_library(motionmodel src/MotionModel.cpp)
target_include_directories(motionmodel PUBLIC
  $<BUILD_INTERFACE:${PKG_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motionmodel
  tf2 tf2_ros tf2_kdl
  orocos_kdl_vendor
  sensor_msgs
  pcl_ros pcl_conversions
)

add_library(observationmodel src/ObservationModel.cpp)
target_include_directories(observationmodel PUBLIC
  $<BUILD_INTERFACE:${PKG_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(observationmodel
  rclcpp tf2 tf2_ros
  pcl_ros
  octomap octomap_msgs
)
target_link_libraries(observationmodel
  octomap
  octomath
)

add_library(raycastingmodel src/RaycastingModel.cpp)
target_include_directories(raycastingmodel PUBLIC
  $<BUILD_INTERFACE:${PKG_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(raycastingmodel
  rclcpp tf2 tf2_ros
  octomap octomap_msgs octomap_ros
)
target_link_libraries(raycastingmodel
  observationmodel
  transform_utils
  octomap
  octomath
  OpenMP::OpenMP_CXX
)

add_library(dynamicedt3d
  src/dynamicEDT3D/dynamicEDT3D.cpp
  src/dynamicEDT3D/dynamicEDTOctomap.cpp
)
target_include_directories(dynamicedt3d PUBLIC
  $<BUILD_INTERFACE:${PKG_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dynamicedt3d octomap)
target_link_libraries(dynamicedt3d
  octomap
  octomath
)

add_library(endpointmodel src/EndpointModel.cpp)
target_include_directories(endpointmodel PUBLIC
  $<BUILD_INTERFACE:${PKG_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(endpointmodel
  visualization_msgs
  pcl_ros
  octomap octomap_msgs
)
target_link_libraries(endpointmodel
  dynamicedt3d
  transform_utils
  octomap
  octomath
)

# ============================
# Main localization library
# ============================
add_library(humanoidlocalization SHARED
  src/HumanoidLocalization.cpp
)
target_include_directories(humanoidlocalization PUBLIC
  $<BUILD_INTERFACE:${PKG_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(humanoidlocalization
  rclcpp geometry_msgs nav_msgs rcl_interfaces
  sensor_msgs std_srvs visualization_msgs
  tf2 tf2_ros tf2_kdl tf2_eigen tf2_geometry_msgs
  message_filters Eigen3
  pcl_ros pcl_conversions
  octomap octomap_msgs octomap_ros
)
target_link_libraries(humanoidlocalization
  mapmodel
  motionmodel
  raycastingmodel
  endpointmodel
  dynamicedt3d
  transform_utils
  octomap
  octomath
  OpenMP::OpenMP_CXX
)

if(SBPL_FOUND)
  target_include_directories(humanoidlocalization PUBLIC ${SBPL_INCLUDE_DIRS})
  target_link_libraries(humanoidlocalization ${SBPL_LIBRARIES})
endif()

# ============================
# Executable
# ============================
add_executable(localization_node src/localization_node.cpp)
target_include_directories(localization_node PRIVATE
  $<BUILD_INTERFACE:${PKG_INCLUDE_DIR}>
)
ament_target_dependencies(localization_node rclcpp)
target_link_libraries(localization_node
  humanoidlocalization
)

# ============================
# Install
# ============================
install(TARGETS
  transform_utils
  mapmodel
  motionmodel
  observationmodel
  raycastingmodel
  dynamicedt3d
  endpointmodel
  humanoidlocalization
  localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ============================
# Export
# ============================
ament_export_include_directories(include)
ament_export_libraries(humanoidlocalization)
ament_export_dependencies(
  rclcpp geometry_msgs nav_msgs rcl_interfaces
  sensor_msgs std_srvs visualization_msgs
  tf2 tf2_ros tf2_kdl tf2_eigen tf2_geometry_msgs
  message_filters Eigen3
  pcl_ros pcl_conversions
  octomap octomap_msgs octomap_ros
)

ament_package()
