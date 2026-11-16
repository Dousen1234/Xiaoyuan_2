cmake_minimum_required(VERSION 3.16)
project(testcmd_ssj1110 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "构建类型" FORCE)
endif()

add_compile_options(-Wall -Wextra)

# ---------------------------------------------------------------- 通信层
add_library(claw_can STATIC claw_can.cpp)
target_include_directories(claw_can PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ---------------------------------------------------------------- 功能程序
# 以后加功能 2、功能 3，照着这两行再写一遍即可
add_executable(test_rotate test_rotate.cpp)
target_link_libraries(test_rotate PRIVATE claw_can)

add_executable(test_positionctrl test_positionctrl.cpp)
target_link_libraries(test_positionctrl PRIVATE claw_can)

# ---------------------------------------------------------------- 离线自测
add_executable(selftest_sim selftest_sim.cpp)
target_link_libraries(selftest_sim PRIVATE claw_can)

add_executable(selftest_position selftest_position.cpp)

enable_testing()
add_test(NAME selftest_sim COMMAND selftest_sim)
add_test(NAME selftest_position COMMAND selftest_position)