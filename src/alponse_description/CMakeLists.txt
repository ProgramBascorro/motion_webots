cmake_minimum_required(VERSION 3.8)
project(alponse_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  meshes
  mujoco
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/view_alponse_mujoco
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
