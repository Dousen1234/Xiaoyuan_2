cmake_minimum_required(VERSION 3.16)
project(TaiHu_motor_control LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =====================================================================
# 核心库：CAN 驱动 + 关节模组控制 + 工具函数 + 日志
# =====================================================================
add_library(taihu_core STATIC
    src/socket_can.cpp        # SocketCAN 驱动（鲲弘 KH-UCANFDX6-Mini，can0~can5）
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
taihu_add_executable(taihu_singleleg_step examples/taihu_singleleg_step.cpp)
taihu_add_executable(taihu_single_motorctl examples/taihu_single_motorctl.cpp)
taihu_add_executable(taihu_single_motorctl_xCAN examples/taihu_single_motorctl_xCAN.cpp)

# —— 工具程序 ——
# taihu_motor_tools：菜单式工具总入口（扫描/诊断/改ID/恢复出厂/标零/开刹车/低压阈值）
taihu_add_executable(taihu_motor_tools tools/taihu_motor_tools.cpp)

# —— 鲲弘 CAN 通道控制工具（纯 C，不依赖核心库） ——
# 用于手动打开/关闭指定 CAN 通道并配置波特率（需要 sudo 运行）
add_executable(kcanctl tools/kcanctl.c)
