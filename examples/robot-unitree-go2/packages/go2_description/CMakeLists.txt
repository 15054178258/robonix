cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY dae urdf DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE.unitree DESTINATION share/${PROJECT_NAME})

ament_package()
