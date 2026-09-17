cmake_minimum_required(VERSION 3.16)
project(TaiHu_motor_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =====================================================================
# 核心库：CAN 驱动 + 关节模组控制 + 工具函数 + 日志
# =====================================================================
add_library(taihu_core STATIC
    src/damiao_usb2can.cpp    # 达妙 USB2CAN 驱动（默认）
    src/socket_can.cpp        # SocketCAN 驱动（备用）
    src/joint_module.cpp      # 关节模组控制
    src/taihu_tools.cpp       # 工具函数封装（诊断/标零/旋转等）
    src/motor_logger.cpp      # 实时日志记录
)
target_include_directories(taihu_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =====================================================================
# 可执行目标（自动链接核心库）
# =====================================================================
function(taihu_add_executable name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE taihu_core)
endfunction()

# —— 示例程序 ——
taihu_add_executable(taihu_main         examples/taihu_main.cpp)
taihu_add_executable(taihu_singleleg_step examples/taihu_singleleg_step.cpp)
taihu_add_executable(taihu_single_motorctl examples/taihu_single_motorctl.cpp)

# —— 工具程序（可执行文件名 = 源文件名，统一 taihu_motor_tools_ 前缀） ——
taihu_add_executable(taihu_motor_tools_diag            tools/taihu_motor_tools_diag.cpp)
taihu_add_executable(taihu_motor_tools_change_id       tools/taihu_motor_tools_change_id.cpp)
taihu_add_executable(taihu_motor_tools_reset_factory   tools/taihu_motor_tools_reset_factory.cpp)
taihu_add_executable(taihu_motor_tools_set_zero        tools/taihu_motor_tools_set_zero.cpp)
taihu_add_executable(taihu_motor_tools_set_low_voltage tools/taihu_motor_tools_set_low_voltage.cpp)
taihu_add_executable(taihu_motor_tools_scan          tools/taihu_motor_tools_scan.cpp)
taihu_add_executable(taihu_motor_tools_openbreak     tools/taihu_motor_tools_openbreak.cpp)
