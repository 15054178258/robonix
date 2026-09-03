cmake_minimum_required(VERSION 3.16)
project(g1_chassis_offline_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(G1_CHASSIS_BUILD_OFFLINE_TESTS "Build tests that never contact a robot" ON)

if(G1_CHASSIS_BUILD_OFFLINE_TESTS)
  enable_testing()
  add_executable(g1_protocol_test tests/protocol_test.cpp)
  target_include_directories(g1_protocol_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk_daemon/include)
  target_link_libraries(g1_protocol_test PRIVATE Catch2WithMain)
  target_compile_options(g1_protocol_test PRIVATE
    -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME g1_protocol_test COMMAND g1_protocol_test)
endif()
