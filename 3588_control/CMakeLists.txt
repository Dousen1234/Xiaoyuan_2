# =============================================================================
# 3588_control —— 腿臂上下位机控制程序
#
# 四个可执行文件：
#   upper_control   上位机：读 xlsx 轨迹 → 打包 CAN 帧 → 发下位机，收电机反馈
#   lower_control   下位机：收 16 个 ID 目标帧 → 拆 4 组 → 下发 can0~can3
#   leg_goto_zero   上位机：读当前位置 → 五次多项式 → 把指定腿臂 4 关节平滑回 0 位
#   x3588_ui        图形界面：把上面三个程序和 CAN 配置集成到一个窗口里（需 Qt5）
#
# 复用上级目录的 taihu 核心库（CAN 接口 + 电机模块），只取需要的 3 个源文件，
# 避免引入 motor_logger / torque_sensor / mpt_encoder 等无关依赖。
#
# 编译（在 3588 上）：
#   mkdir -p build && cd build && cmake .. && make -j4
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(X3588_Control LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -Wextra")

# -----------------------------------------------------------------------------
# taihu 核心（只编译上下位机真正用到的部分）
# -----------------------------------------------------------------------------
set(TAIHU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_library(taihu_core STATIC
    ${TAIHU_ROOT}/src/socket_can.cpp
    ${TAIHU_ROOT}/src/joint_module.cpp
    ${TAIHU_ROOT}/src/taihu_tools.cpp
)
target_include_directories(taihu_core PUBLIC ${TAIHU_ROOT}/include)

# std::thread 需要 pthread。
# glibc >= 2.34 已把 pthread 并入 libc，但 Ubuntu 20.04（glibc 2.31，即本 3588）
# 必须显式链接 -pthread，否则运行期抛 std::system_error。
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(taihu_core PUBLIC Threads::Threads)

# -----------------------------------------------------------------------------
# 上位机
# -----------------------------------------------------------------------------
add_executable(upper_control
    upper_control.cpp
    xlsx_reader.cpp
)
target_link_libraries(upper_control PRIVATE taihu_core)

# xlsx 解压需要 zlib（3588 系统自带 /usr/include/zlib.h + libz.so）
find_package(ZLIB REQUIRED)
target_link_libraries(upper_control PRIVATE ZLIB::ZLIB)

# -----------------------------------------------------------------------------
# 下位机
# -----------------------------------------------------------------------------
add_executable(lower_control lower_control.cpp)
target_link_libraries(lower_control PRIVATE taihu_core)

# -----------------------------------------------------------------------------
# 回零工具（上位机侧；同样打开 can_mcu，依赖 lower_control 在下位机侧运行）
# -----------------------------------------------------------------------------
add_executable(leg_goto_zero leg_goto_zero.cpp)
target_link_libraries(leg_goto_zero PRIVATE taihu_core)

# -----------------------------------------------------------------------------
# 图形界面（Qt5 Widgets）
#
# Qt5 不是必须的：没装 qtbase5-dev 时只是跳过这个目标，其余三个程序照常编译。
# 3588 上已预装 Qt 5.12.8（/usr/lib/aarch64-linux-gnu/libQt5Widgets.so.5.12）。
# -----------------------------------------------------------------------------
find_package(Qt5 COMPONENTS Widgets QUIET)
if(Qt5Widgets_FOUND)
    set(CMAKE_AUTOMOC ON)
    add_executable(x3588_ui x3588_ui.cpp)
    target_link_libraries(x3588_ui PRIVATE taihu_core Qt5::Widgets)
    # x3588_ui.cpp 里用 Q_OBJECT + #include "x3588_ui.moc"，AUTOMOC 会处理。
    set_target_properties(x3588_ui PROPERTIES
        AUTOMOC ON
        AUTOMOC_MOC_OPTIONS "")
else()
    message(STATUS "  未找到 Qt5 Widgets，跳过 x3588_ui（apt install qtbase5-dev 后可编译）")
endif()

# -----------------------------------------------------------------------------
# 构建完成提示
# -----------------------------------------------------------------------------
message(STATUS "")
message(STATUS "=== 3588_control 配置完成 ===")
message(STATUS "  上位机可执行: ${CMAKE_CURRENT_BINARY_DIR}/upper_control")
message(STATUS "  下位机可执行: ${CMAKE_CURRENT_BINARY_DIR}/lower_control")
message(STATUS "  回零工具    : ${CMAKE_CURRENT_BINARY_DIR}/leg_goto_zero")
if(Qt5Widgets_FOUND)
    message(STATUS "  图形界面    : ${CMAKE_CURRENT_BINARY_DIR}/x3588_ui")
    message(STATUS "              （用 sudo ./run_ui_sudo.sh 启动）")
endif()
message(STATUS "")
message(STATUS "  下一步: make -j4")
message(STATUS "")