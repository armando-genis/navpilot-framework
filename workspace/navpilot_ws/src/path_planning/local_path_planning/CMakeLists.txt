cmake_minimum_required(VERSION 3.8)
project(local_path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(obstacles_information_msgs REQUIRED)
find_package(traffic_information_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
set(tinysplinecxx_DIR "/usr/local/lib64/cmake/tinysplinecxx/")

find_package(OpenCV REQUIRED)

# External dependencies
find_package(Eigen3 REQUIRED)

add_executable(local_path_planning_node
                src/local_path_planning_node.cpp
                src/sat_collision_checker.cc
                src/Grid_map.cpp
                src/CarData.cpp
                src/CubicSpline1D.cpp
                src/HybridAstar.cpp
                src/reeds_shepp_path.cpp
                src/dubins_path.cpp
               )

# Link libraries
ament_target_dependencies(local_path_planning_node
  rclcpp
  std_msgs
  sensor_msgs
  grid_map_ros
  geometry_msgs
  visualization_msgs
  tf2
  obstacles_information_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2_ros
  traffic_information_msgs
  OpenCV
)

target_include_directories(local_path_planning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  local_path_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(local_path_planning_node
  Eigen3::Eigen
  tinysplinecxx
)

# Install other files (e.g., launch files)
install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("-- ROS Found. ROS Support is turned ON for local_path_planning_node.")
message(=============================================================)

ament_package()
