cmake_minimum_required(VERSION 3.16)
project(go2_camera_daemon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

set(UNITREE_SDK2_ROOT "" CACHE PATH "Path to a unitree_sdk2 source checkout")
if(NOT UNITREE_SDK2_ROOT AND DEFINED ENV{UNITREE_SDK2_ROOT})
  set(UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2_ROOT}")
endif()
if(NOT EXISTS "${UNITREE_SDK2_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR "Set UNITREE_SDK2_ROOT to an official unitree_sdk2 source checkout")
endif()

set(BUILD_EXAMPLES OFF CACHE BOOL "Do not build Unitree examples" FORCE)
add_subdirectory("${UNITREE_SDK2_ROOT}" unitree_sdk2-build EXCLUDE_FROM_ALL)
find_package(Threads REQUIRED)

add_executable(go2_camera_daemon src/camera_daemon.cpp)
target_compile_definitions(go2_camera_daemon PRIVATE _GNU_SOURCE)
target_include_directories(go2_camera_daemon PRIVATE ../include)
target_link_libraries(go2_camera_daemon PRIVATE unitree_sdk2 Threads::Threads)

# Keep the SDK process self-contained.  The provider and the two ROS nodes must
# never need the vendor SDK's CycloneDDS build on their LD_LIBRARY_PATH.
set_target_properties(go2_camera_daemon PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib"
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

install(
  TARGETS go2_camera_daemon
  RUNTIME DESTINATION bin
  COMPONENT go2_camera_runtime
)

set(UNITREE_DDS_LIBRARY_DIR
    "${UNITREE_SDK2_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
foreach(UNITREE_DDS_LIBRARY
        libddsc.so libddsc.so.0 libddscxx.so libddscxx.so.0)
  if(NOT EXISTS "${UNITREE_DDS_LIBRARY_DIR}/${UNITREE_DDS_LIBRARY}")
    message(FATAL_ERROR
      "Required Unitree DDS library is missing: ${UNITREE_DDS_LIBRARY}")
  endif()
  install(
    FILES "${UNITREE_DDS_LIBRARY_DIR}/${UNITREE_DDS_LIBRARY}"
    DESTINATION lib
    COMPONENT go2_camera_runtime
  )
endforeach()

install(
  FILES "${UNITREE_SDK2_ROOT}/LICENSE"
  DESTINATION share/licenses/unitree_sdk2
  COMPONENT go2_camera_runtime
)
install(
  FILES "${UNITREE_SDK2_ROOT}/licenses/eclipse-cyclonedds/cyclonedds/LICENSE"
  DESTINATION share/licenses/unitree_sdk2/cyclonedds
  COMPONENT go2_camera_runtime
)
install(
  FILES "${UNITREE_SDK2_ROOT}/licenses/eclipse-cyclonedds/cyclonedds-cxx/LICENSE"
  DESTINATION share/licenses/unitree_sdk2/cyclonedds-cxx
  COMPONENT go2_camera_runtime
)
install(
  FILES "${UNITREE_SDK2_ROOT}/licenses/eclipse-iceoryx/iceoryx/LICENSE"
  DESTINATION share/licenses/unitree_sdk2/iceoryx
  COMPONENT go2_camera_runtime
)
install(
  FILES "${UNITREE_SDK2_ROOT}/licenses/Tencent/rapidjson/LICENSE"
  DESTINATION share/licenses/unitree_sdk2/rapidjson
  COMPONENT go2_camera_runtime
)
