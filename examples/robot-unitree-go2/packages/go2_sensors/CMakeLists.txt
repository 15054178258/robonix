cmake_minimum_required(VERSION 3.8)
project(go2_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(JPEG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(go2_sensor_relay src/sensor_relay_node.cpp)
target_include_directories(go2_sensor_relay PRIVATE include)
ament_target_dependencies(go2_sensor_relay
  diagnostic_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(go2_camera_bridge
  src/camera_bridge_node.cpp
  src/strict_jpeg_decoder.cpp
)
target_include_directories(go2_camera_bridge PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(go2_camera_bridge JPEG::JPEG ${OpenCV_LIBRARIES} Threads::Threads)
target_compile_definitions(go2_camera_bridge PRIVATE _GNU_SOURCE)
ament_target_dependencies(go2_camera_bridge
  cv_bridge
  diagnostic_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS
  go2_camera_bridge
  go2_sensor_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
