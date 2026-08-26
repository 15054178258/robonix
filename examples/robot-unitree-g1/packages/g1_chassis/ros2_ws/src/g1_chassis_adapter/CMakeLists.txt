cmake_minimum_required(VERSION 3.16)
project(g1_chassis_adapter LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(g1_chassis_adapter_node
  src/g1_chassis_adapter_node.cpp
)
ament_target_dependencies(g1_chassis_adapter_node
  std_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS g1_chassis_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
