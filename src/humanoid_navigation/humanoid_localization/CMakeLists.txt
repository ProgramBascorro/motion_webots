cmake_minimum_required(VERSION 3.8)
project(humanoid_localization)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED) # Pastikan sudah diinstal: sudo apt install ros-humble-octomap-ros
find_package(Eigen3 REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
message(STATUS "message_filters include dirs: ${message_filters_INCLUDE_DIRS}")
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(DYNAMICEDT3D_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Try to find SBPL
find_package(SBPL)

if(SBPL_FOUND)
  add_definitions(-DUSE_SBPL)
  message(STATUS "SBPL found, building with SBPL functionality")
  # SBPL include dirs ditambahkan via target_include_directories nanti
else()
  message(WARNING "SBPL not found, building without SBPL functionality")
  set(SBPL_LIBRARIES "")
  set(SBPL_INCLUDE_DIRS "") # Set variabel kosong jika tidak ditemukan
endif()

# Find OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  # OpenMP include dirs ditambahkan via target_include_directories nanti
  # OpenMP libraries ditambahkan via target_link_libraries nanti
endif()

# --- Hapus atau komentari blok include_directories global ---
# include_directories(
#  include
#  ${SBPL_INCLUDE_DIRS}
#  ... (dan seterusnya) ...
# )

# --- Add Libraries ---
add_library(transform_utils src/transform_utils.cpp)
add_library(mapmodel src/MapModel.cpp)
add_library(motionmodel src/MotionModel.cpp)
add_library(observationmodel src/ObservationModel.cpp)
add_library(raycastingmodel src/RaycastingModel.cpp)
add_library(endpointmodel src/EndpointModel.cpp)
add_library(dynamicedt3d
  src/dynamicEDT3D/dynamicEDT3D.cpp
  src/dynamicEDT3D/dynamicEDTOctomap.cpp
)
# Library utama, tambahkan semua file sumbernya
add_library(humanoidlocalization SHARED # Gunakan SHARED jika ini library utama
  src/HumanoidLocalization.cpp
  # Tambahkan file .cpp lain jika ada yang bagian dari library ini
)

# --- Target Include Directories ---
# Tambahkan include path untuk setiap library jika diperlukan
target_include_directories(transform_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Include lokal
  $<INSTALL_INTERFACE:include>
  ${tf2_eigen_INCLUDE_DIRS} # Contoh: Jika transform_utils.cpp butuh Eigen
  ${tf2_kdl_INCLUDE_DIRS}   # Contoh: Jika transform_utils.cpp butuh KDL
)

target_include_directories(mapmodel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${octomap_INCLUDE_DIRS}
  ${octomap_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

target_include_directories(motionmodel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_kdl_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

target_include_directories(observationmodel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${octomap_INCLUDE_DIRS}
  ${octomap_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

target_include_directories(raycastingmodel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${octomap_INCLUDE_DIRS}
  ${OpenMP_CXX_INCLUDE_DIRS} # Jika OpenMP punya include dirs
)

target_include_directories(dynamicedt3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Untuk header dynamicedt3d
  $<INSTALL_INTERFACE:include>
  ${octomap_INCLUDE_DIRS}
)

target_include_directories(endpointmodel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${octomap_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${octomap_msgs_INCLUDE_DIRS}
)

# Include directories untuk library utama humanoidlocalization
target_include_directories(humanoidlocalization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SBPL_INCLUDE_DIRS} # Dari find_package(SBPL)
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
  ${octomap_msgs_INCLUDE_DIRS}
  ${octomap_ros_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${DYNAMICEDT3D_INCLUDE_DIRS} # Dari set() di atas
  ${visualization_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS} # <-- Penting
  ${OpenMP_CXX_INCLUDE_DIRS} # Dari find_package(OpenMP)
  ${orocos_kdl_INCLUDE_DIRS} # Ditambahkan jika diperlukan oleh header
  ${tf2_kdl_INCLUDE_DIRS}    # Ditambahkan jika diperlukan oleh header
  ${tf2_eigen_INCLUDE_DIRS}   # Ditambahkan jika diperlukan oleh header
  ${sensor_msgs_INCLUDE_DIRS} # Ditambahkan jika diperlukan oleh header
)

# --- Target Link Libraries ---
# Link library kecil ke dependensi eksternalnya
target_link_libraries(transform_utils PUBLIC
  ${tf2_eigen_LIBRARIES}
  ${tf2_kdl_LIBRARIES}
)

target_link_libraries(mapmodel PUBLIC
  ${octomap_LIBRARIES}
  ${octomap_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
)

target_link_libraries(motionmodel PUBLIC
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_kdl_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${pcl_ros_LIBRARIES}
)

target_link_libraries(observationmodel PUBLIC
  ${octomap_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${pcl_ros_LIBRARIES}
)

target_link_libraries(raycastingmodel PUBLIC
  observationmodel # Link ke library internal
  transform_utils  # Link ke library internal
  ${OpenMP_CXX_LIBRARIES}
  ${octomap_LIBRARIES}
)

target_link_libraries(dynamicedt3d PUBLIC
  ${octomap_LIBRARIES}
)

target_link_libraries(endpointmodel PUBLIC
  dynamicedt3d     # Link ke library internal
  transform_utils  # Link ke library internal
  ${octomap_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
)

# Link library utama humanoidlocalization
target_link_libraries(humanoidlocalization PUBLIC # Gunakan PUBLIC agar node yang link ke sini dapat librarynya
  # Library internal
  mapmodel
  motionmodel
  raycastingmodel
  endpointmodel
  dynamicedt3d
  transform_utils # Ditambahkan jika belum ada

  # Library eksternal
  ${octomap_LIBRARIES}
  $<$<BOOL:${SBPL_FOUND}>:${SBPL_LIBRARIES}> # Kondisional link SBPL
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${octomap_msgs_LIBRARIES}
  ${octomap_ros_LIBRARIES} # Ditambahkan
  ${rcl_interfaces_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_kdl_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${pcl_conversions_LIBRARIES} # Ditambahkan
  ${std_srvs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${tf2_eigen_LIBRARIES}
  ${message_filters_LIBRARIES} # <-- Penting
  ${visualization_msgs_LIBRARIES} # Ditambahkan
  ${orocos_kdl_LIBRARIES} # Ditambahkan
)

# --- Executable ---
add_executable(localization_node
  src/localization_node.cpp
  src/HumanoidLocalization.cpp
)

# Include directories untuk executable (hanya yang dibutuhkan langsung oleh node)
target_include_directories(localization_node PRIVATE
   # Tidak perlu include path dari dependensi humanoidlocalization karena sudah PUBLIC
   # Tambahkan include path jika localization_node.cpp include header lain secara langsung
   ${rclcpp_INCLUDE_DIRS} # Contoh
)

# Link executable ke library utama dan dependensi runtime lainnya
target_link_libraries(localization_node PRIVATE # PRIVATE karena ini executable
  humanoidlocalization # Link ke library utama kita
  # Library lain yang mungkin dibutuhkan HANYA oleh node (jarang terjadi jika sudah di humanoidlocalization)
  # ${rclcpp_LIBRARIES} # Tidak perlu diulang jika humanoidlocalization sudah PUBLIC link
)

# ---------------------------
# Link OctoMap + include dirs
# ---------------------------
# Debug prints (opsional, hapus jika tidak ingin)
message(STATUS "octomap include dirs: ${octomap_INCLUDE_DIRS}")
message(STATUS "octomap libraries: ${octomap_LIBRARIES}")

if(TARGET raycastingmodel)
  if(DEFINED octomap_INCLUDE_DIRS)
    target_include_directories(raycastingmodel PRIVATE ${octomap_INCLUDE_DIRS})
  else()
    target_include_directories(raycastingmodel PRIVATE /opt/ros/humble/include)
  endif()

  target_link_libraries(raycastingmodel
    PRIVATE
      octomap
      octomath
      ${octomap_LIBRARIES}
  )
endif()

if(TARGET humanoidlocalization)
  if(DEFINED octomap_INCLUDE_DIRS)
    target_include_directories(humanoidlocalization PRIVATE ${octomap_INCLUDE_DIRS})
  else()
    target_include_directories(humanoidlocalization PRIVATE /opt/ros/humble/include)
  endif()

  target_link_libraries(humanoidlocalization
    PRIVATE
      octomap
      octomath
      ${octomap_LIBRARIES}
  )
endif()

if(TARGET localization_node)
  target_link_libraries(localization_node
    PRIVATE
      humanoidlocalization
      raycastingmodel
      octomap
      octomath
      ${octomap_LIBRARIES}
  )
  if(DEFINED octomap_INCLUDE_DIRS)
    target_include_directories(localization_node PRIVATE ${octomap_INCLUDE_DIRS})
  else()
    target_include_directories(localization_node PRIVATE /opt/ros/humble/include)
  endif()
endif()

# --- Install Targets ---
# Install libraries
install(TARGETS
  endpointmodel
  mapmodel
  motionmodel
  observationmodel
  raycastingmodel
  humanoidlocalization
  transform_utils
  dynamicedt3d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin # Jika ada executable di sini
)

# Install executable
install(TARGETS
  localization_node
  DESTINATION lib/${PROJECT_NAME} # Tempat umum untuk node di ROS 2
)

# Install include directories (agar paket lain bisa menggunakan header)
install(DIRECTORY include/
  DESTINATION include
  # FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" # Lebih eksplisit
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# --- Ament Package ---
# Pastikan semua dependensi ada di package.xml
ament_package()
