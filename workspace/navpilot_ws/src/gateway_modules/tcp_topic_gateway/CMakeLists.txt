cmake_minimum_required(VERSION 3.8)
project(tcp_topic_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(tcp_gateway_sender src/tcp_gateway_sender.cpp)
ament_target_dependencies(tcp_gateway_sender rclcpp sensor_msgs std_msgs)

add_executable(tcp_gateway_receiver src/tcp_gateway_receiver.cpp)
ament_target_dependencies(tcp_gateway_receiver rclcpp sensor_msgs std_msgs)

install(TARGETS
  tcp_gateway_sender
  tcp_gateway_receiver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
