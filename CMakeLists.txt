cmake_minimum_required(VERSION 3.20)
project(inav_fdm_lite VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# 告警即眼线：新代码零警告通过
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)

add_executable(inav_sitl_bridge
  src/main.cpp
  src/config.cpp
  src/fdm.cpp
  src/xplane_link.cpp
  src/mavlink_link.cpp
  src/mavlink_codec.cpp
  src/ctl_server.cpp
)
target_include_directories(inav_sitl_bridge PRIVATE src)
target_link_libraries(inav_sitl_bridge PRIVATE pthread)

# 自检：MAVLink 编解码 + FDM 数学 + 配置解析 + 控制面解析
add_executable(bridge_selftest
  tests/selftest_main.cpp
  src/config.cpp
  src/fdm.cpp
  src/mavlink_codec.cpp
  src/xplane_link.cpp
  src/ctl_server.cpp
)
target_include_directories(bridge_selftest PRIVATE src)
target_link_libraries(bridge_selftest PRIVATE pthread)

enable_testing()
add_test(NAME bridge_selftest COMMAND bridge_selftest)
