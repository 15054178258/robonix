cmake_minimum_required(VERSION 3.16)
project(go2_chassis_adapter LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(go2_chassis_adapter_node src/go2_chassis_adapter_node.cpp)
target_include_directories(go2_chassis_adapter_node PRIVATE ../../../include)
target_compile_options(go2_chassis_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(go2_chassis_adapter_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  unitree_go
)

install(TARGETS go2_chassis_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY ../../../include/go2_chassis
  DESTINATION include
)

ament_package()
