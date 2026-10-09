cmake_minimum_required(VERSION 3.16)
project(taihu_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# =====================================================================
# taihu_core：直接复用仓库根目录的核心库源码（单一事实来源，不复制代码）
# =====================================================================
set(TAIHU_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

add_library(taihu_core_ros STATIC
    ${TAIHU_CORE_DIR}/src/socket_can.cpp
    ${TAIHU_CORE_DIR}/src/joint_module.cpp
    ${TAIHU_CORE_DIR}/src/taihu_tools.cpp
    ${TAIHU_CORE_DIR}/src/motor_logger.cpp
)
target_include_directories(taihu_core_ros PUBLIC
    ${TAIHU_CORE_DIR}/include
)
# taihu_tools.cpp 依赖 pthread
target_link_libraries(taihu_core_ros PUBLIC pthread)

# =====================================================================
# 电机驱动节点：单条 CAN 总线 + 四个电机（一条腿），
# 200Hz 发布 JointState，服务 enable/stop/set_gains
# =====================================================================
add_executable(taihu_motor_node src/taihu_motor_node.cpp)
target_link_libraries(taihu_motor_node taihu_core_ros)
ament_target_dependencies(taihu_motor_node rclcpp sensor_msgs std_srvs)

# =====================================================================
# 步态播放节点：复用单腿步态轨迹，支持「每步正摆后暂停等放行」
# =====================================================================
add_executable(taihu_gait_node src/taihu_gait_node.cpp)
target_link_libraries(taihu_gait_node taihu_core_ros)
ament_target_dependencies(taihu_gait_node rclcpp sensor_msgs std_srvs std_msgs)

install(TARGETS taihu_motor_node taihu_gait_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
