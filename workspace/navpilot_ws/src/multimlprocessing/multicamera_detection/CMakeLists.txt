cmake_minimum_required(VERSION 3.8)
project(multicamera_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# External dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(multicamera_detection_node src/multicamera_subscriber.cpp)

ament_target_dependencies(
  multicamera_detection_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  cv_bridge
)

# Include YOLOs-CPP headers
include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/../YOLOs-CPP/include"
)

# Set the ONNX Runtime directory manually
set(ONNXRUNTIME_DIR "/workspace/navpilot_ws/src/multimlprocessing/YOLOs-CPP/onnxruntime-linux-x64-1.20.1")

# Include ONNX Runtime headers
include_directories("${ONNXRUNTIME_DIR}/include")

# Set the ONNX Runtime library
set(ONNXRUNTIME_LIB "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")

target_include_directories(multicamera_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../YOLOs-CPP/include>
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../YOLOs-CPP ${CMAKE_CURRENT_BINARY_DIR}/yolos_build)

# Install other files (e.g., launch files)
install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(multicamera_detection_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
  yaml-cpp
  yolos
)

install(TARGETS
  multicamera_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

message(=============================================================)
message("-- multicamera_detection: Build Complete")
message(=============================================================)

ament_package()
