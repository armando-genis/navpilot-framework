cmake_minimum_required(VERSION 3.8)
project(camera_lidar_sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_lidar_sync src/camera_lidar_sync.cpp)

# Link libraries
ament_target_dependencies(camera_lidar_sync
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  OpenCV
)

target_include_directories(camera_lidar_sync PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  camera_lidar_sync
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (e.g., launch files)
install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
