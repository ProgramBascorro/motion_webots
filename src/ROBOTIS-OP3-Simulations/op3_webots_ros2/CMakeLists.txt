cmake_minimum_required(VERSION 3.8)
project(op3_webots_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)

find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)


add_executable(op3_extern_controller 
  src/op3_extern_controller.cpp 
  src/extern_controller_main.cpp
)

target_link_libraries(op3_extern_controller ${YAML_CPP_LIBRARIES})

ament_target_dependencies(op3_extern_controller
  ament_index_cpp
  rclcpp
  webots_ros2_driver
  nav_msgs
)

install(TARGETS
  op3_extern_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  resource
  images
  protos
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()