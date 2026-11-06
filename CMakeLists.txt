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
    src/torque_sensor.cpp     # 坤维扭矩传感器（HIT-V2.0，可与电机共总线）
    src/mpt_encoder.cpp       # 金钢 MPT 磁编码器（RS485，独立于 CAN 总线）
    src/trajectory_pack.cpp   # 轨迹打包：16 关节轨迹 → 16-ID CAN 帧（纯计算，无 IO）
)
target_include_directories(taihu_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# std::thread / std::mutex 需要 pthread。glibc 2.34 之前 libpthread 未并入 libc，
# 必须显式链接（RK3588 板端是 glibc 2.31，漏掉会出现 undefined reference to pthread_create）。
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(taihu_core PUBLIC Threads::Threads)

# =====================================================================
# 可执行目标（自动链接核心库）
# =====================================================================
function(taihu_add_executable name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE taihu_core)
endfunction()

# —— 示例程序 ——
taihu_add_executable(taihu_singleleg_step examples/taihu_singleleg_step.cpp)
taihu_add_executable(taihu_singleleg_step_xCAN examples/taihu_singleleg_step_xCAN.cpp)
taihu_add_executable(taihu_single_motorctl examples/taihu_single_motorctl.cpp)
taihu_add_executable(taihu_single_motorctl_xCAN examples/taihu_single_motorctl_xCAN.cpp)
taihu_add_executable(taihu_keyboardctl examples/taihu_keyboardctl.cpp)

# 单腿 4 关节纯 PD 位置模式 demo（200Hz，hold/goto 交互命令 + 软限位/电流/急停保护）
taihu_add_executable(taihu_pd_leg examples/taihu_pd_leg.cpp)

# 四腿机器人关节级命令控制（语义命令 → 限位 → 平滑轨迹 → 到位确认）
taihu_add_executable(walk_control_V1 examples/walk_control_V1.cpp)

# 轨迹打包示例：把一段 16 关节轨迹转成 CAN 帧并打印出来（不打开硬件）
taihu_add_executable(trajectory_pack_demo examples/trajectory_pack_demo.cpp)

# —— 工具程序 ——
# taihu_motor_tools：菜单式工具总入口（扫描/诊断/改ID/恢复出厂/标零/开刹车/低压阈值）
taihu_add_executable(taihu_motor_tools tools/taihu_motor_tools.cpp)

# taihu_torque_sensor：坤维扭矩传感器独立查看工具（与主控制程序共总线、互不干扰）
taihu_add_executable(taihu_torque_sensor tools/taihu_torque_sensor.cpp)

# taihu_mpt_encoder：金钢 MPT 磁编码器独立查看工具（RS485 串口，独立于 CAN）
taihu_add_executable(taihu_mpt_encoder tools/taihu_mpt_encoder.cpp)

# taihu_encoder_can_bridge：MPT 编码器 → CAN 软件网关（RS485 读数转 CAN 帧，
# 与电机/扭矩传感器共总线，帧 ID 甄别互不干扰）
taihu_add_executable(taihu_encoder_can_bridge tools/taihu_encoder_can_bridge.cpp)

# —— 鲲弘 CAN 通道控制工具（纯 C，不依赖核心库） ——
# 用于手动打开/关闭指定 CAN 通道并配置波特率（需要 sudo 运行）
add_executable(kcanctl tools/kcanctl.c)

# =====================================================================
# 可选 Qt5 图形界面程序（未安装 Qt5 时自动跳过，不影响其它目标）
# =====================================================================
find_package(Qt5 COMPONENTS Widgets QUIET)
if(Qt5Widgets_FOUND)
    message(STATUS "Found Qt5 Widgets ${Qt5Widgets_VERSION} -> will build taihu_keyboardctl_ui")
    # .cpp 内含 Q_OBJECT 类，需要 AUTOMOC 自动生成 moc 并编译
    set(CMAKE_AUTOMOC ON)
    taihu_add_executable(taihu_keyboardctl_ui ui/taihu_keyboardctl_ui.cpp)
    target_link_libraries(taihu_keyboardctl_ui PRIVATE Qt5::Widgets)
else()
    message(STATUS "Qt5 Widgets not found -> skip taihu_keyboardctl_ui (install qtbase5-dev to enable)")
endif()
