cmake_minimum_required(VERSION 3.8)
project(path_planning_dynamic)

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
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
set(tinysplinecxx_DIR "/usr/local/lib64/cmake/tinysplinecxx/")

find_package(lanelet2_core)
find_package(lanelet2_io)
find_package(lanelet2_maps)
find_package(lanelet2_projection)
find_package(lanelet2_routing)
find_package(lanelet2_traffic_rules)
find_package(lanelet2_validation)
find_package(polygon_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(path_planning_node
                src/path_planning.cpp
                src/sat_collision_checker.cc
                src/CarData.cpp
                src/Grid_map.cpp
                src/GlobalPlanner.cpp
                src/GlobalPlannerfromPolygon.cpp
               )

ament_target_dependencies(path_planning_node
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

  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  polygon_msgs
)

target_include_directories(path_planning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  path_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(path_planning_node
  Eigen3::Eigen
  tinysplinecxx
  nlohmann_json::nlohmann_json
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
