cmake_minimum_required(VERSION 3.8)
project(localization_sound_alert)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(localization_sound_alert_node src/localization_sound_alert.cpp)

ament_target_dependencies(
  localization_sound_alert_node
  rclcpp
  std_msgs
)

install(TARGETS
  localization_sound_alert_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
