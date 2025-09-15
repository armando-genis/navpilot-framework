cmake_minimum_required(VERSION 3.8)
project(pointcloud_clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(obstacles_information_msgs REQUIRED)

# External dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_geometry REQUIRED) 

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_clustering_node src/pointcloud_clustering.cpp)

# Link libraries
ament_target_dependencies(pointcloud_clustering_node
  rclcpp
  std_msgs
  sensor_msgs
  message_filters
  vision_msgs
  visualization_msgs
  tf2_ros
  geometry_msgs
  pcl_conversions
  obstacles_information_msgs
)

target_include_directories(pointcloud_clustering_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  pointcloud_clustering_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(pointcloud_clustering_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# Install other files (e.g., launch files)
install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("-- ROS Found. ROS Support is turned ON for pointcloud_clustering_node.")
message(=============================================================)

ament_package()
