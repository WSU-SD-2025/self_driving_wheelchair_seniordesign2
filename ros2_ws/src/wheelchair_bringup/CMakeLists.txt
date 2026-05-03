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
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderStamped.msg"
  DEPENDENCIES std_msgs
 )

add_executable(sensor_bridge
  src/sensor_bridge.cpp
)

add_executable(wheel_odom_node
  src/wheel_odom_node.cpp
)


ament_target_dependencies(sensor_bridge
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
)

ament_target_dependencies(wheel_odom_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
)


rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(sensor_bridge
  ${cpp_typesupport_target}
)

target_link_libraries(wheel_odom_node
  ${cpp_typesupport_target}
)

install(TARGETS
  sensor_bridge
  wheel_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
