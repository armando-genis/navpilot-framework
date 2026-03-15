cmake_minimum_required(VERSION 3.8)
project(path_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# External dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(path_processing_node src/multicamera_subscriber.cpp)

ament_target_dependencies(
  path_processing_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(path_processing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install other files (e.g., launch files)
install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(path_processing_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
  yaml-cpp
)

install(TARGETS
  path_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

message(=============================================================)
message("-- path_processing: Build Complete")
message(=============================================================)

ament_package()
