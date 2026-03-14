cmake_minimum_required(VERSION 3.8)
project(lidar_camera_matcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(lidar_camera_matcher src/camera_lidar_sync.cpp)

target_include_directories(lidar_camera_matcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lidar_camera_matcher
  rclcpp
  sensor_msgs
  message_filters
)

# Install targets
install(TARGETS
  lidar_camera_matcher
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (e.g., launch files)
install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
