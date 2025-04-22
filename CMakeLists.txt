cmake_minimum_required(VERSION 3.8)
project(kobuki_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(kobuki_control_node src/kobuki_control_node.cpp)
target_include_directories(kobuki_control_node PUBLIC include)
ament_target_dependencies(kobuki_control_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

add_executable(mode_publisher src/mode_publisher.cpp)
target_include_directories(mode_publisher PUBLIC include)
ament_target_dependencies(mode_publisher rclcpp std_msgs)

install(TARGETS
  kobuki_control_node
  mode_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

