cmake_minimum_required(VERSION 3.8)
project(op3_yolo_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(soccer_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(yolo_detector
  src/yolo_detector.cpp
  src/yolo_detector_node.cpp
)

ament_target_dependencies(yolo_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  soccer_msgs
  ament_index_cpp
)

target_link_libraries(yolo_detector ${OpenCV_LIBRARIES})

install(TARGETS
  yolo_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
