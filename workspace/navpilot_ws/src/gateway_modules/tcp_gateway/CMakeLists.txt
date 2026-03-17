cmake_minimum_required(VERSION 3.8)
project(tcp_gateway)

# ── Compiler settings ──────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ───────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# ── Executable ─────────────────────────────────────────────
add_executable(tcp_gateway_node src/tcp_gateway_node.cpp)

ament_target_dependencies(tcp_gateway_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2_msgs
)

# ── Install ────────────────────────────────────────────────
install(TARGETS tcp_gateway_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()