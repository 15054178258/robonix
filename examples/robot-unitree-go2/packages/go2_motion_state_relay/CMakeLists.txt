cmake_minimum_required(VERSION 3.16)
project(go2_motion_state_relay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(workstation_motion_state_relay
  src/workstation_motion_state_relay.cpp
)
target_include_directories(workstation_motion_state_relay PRIVATE include)
target_compile_options(workstation_motion_state_relay PRIVATE
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
)
ament_target_dependencies(workstation_motion_state_relay rclcpp unitree_go)

install(TARGETS workstation_motion_state_relay
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(gid_guard_test tests/gid_guard_test.cpp)
  target_include_directories(gid_guard_test PRIVATE include)
  target_compile_options(gid_guard_test PRIVATE
    -UNDEBUG -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
  )
  add_test(NAME gid_guard_test COMMAND gid_guard_test)

  add_executable(qos_contract_test tests/qos_contract_test.cpp)
  target_include_directories(qos_contract_test PRIVATE include)
  target_compile_options(qos_contract_test PRIVATE
    -UNDEBUG -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
  )
  ament_target_dependencies(qos_contract_test rclcpp)
  add_test(NAME qos_contract_test COMMAND qos_contract_test)

  add_executable(freshness_contract_test tests/freshness_contract_test.cpp)
  target_include_directories(freshness_contract_test PRIVATE include)
  target_compile_options(freshness_contract_test PRIVATE
    -UNDEBUG -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
  )
  add_test(NAME freshness_contract_test COMMAND freshness_contract_test)
endif()

ament_package()
