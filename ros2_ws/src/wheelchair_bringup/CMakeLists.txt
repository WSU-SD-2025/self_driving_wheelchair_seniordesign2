cmake_minimum_required(VERSION 3.8)
project(wheelchair_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(sensor_bridge
  src/sensor_bridge.cpp
)

ament_target_dependencies(sensor_bridge
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS
  sensor_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
