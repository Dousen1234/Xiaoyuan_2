#ifndef TAIHU_TAIHU_TOOLS_H
#define TAIHU_TAIHU_TOOLS_H

#include <cstdint>
#include <memory>
#include <string>

#include "taihu/can_interface.h"
#include "taihu/joint_module.h"

namespace taihu {

/**
 * @brief 电机参数配置。
 *
 * 用于描述总线上一个电机设备的标识与运动参数，不同型号（不同减速比）
 * 的电机可通过各自的 MotorConfig 区分。
 */
struct MotorConfig {
    std::string name;              ///< 电机名称（标识用）
    uint32_t    can_id     = 1;    ///< CAN ID
    double      gear_ratio = 81.0; ///< 减速比
    int32_t     kp         = 5000; ///< 位置环比例增益
    int32_t     kd         = 60;   ///< 位置环微分增益
};

/**
 * @brief 初始化电机：根据配置创建并配置一个 JointModule。
 *
 * 依据 MotorConfig 设置减速比与位置环增益，返回可用的电机对象。
 *
 * @param can CAN 接口实例
 * @param cfg 电机参数配置
 * @return 配置完成的 JointModule（unique_ptr 便于放入容器统一管理）
 */
std::unique_ptr<JointModule> initMotor(CanInterface& can, const MotorConfig& cfg);

/**
 * @brief 电机运行状态快照，汇总 ID/位置/转速/电流/电压/错误。
 */
struct MotorStatus {
    uint32_t can_id         = 0;    ///< 电机 CAN ID
    double   position_rad   = 0.0;  ///< 当前位置（弧度，输出端）
    double   velocity_rad_s = 0.0;  ///< 当前速度（rad/s，输出端）
    int32_t  current_ma     = 0;    ///< 当前电流（mA）
    int32_t  bus_voltage_v  = 0;    ///< 母线电压（V）
    uint32_t error_bits     = 0;    ///< 错误状态位
};

/**
 * @brief 读取电机完整运行状态（ID/位置/转速/电流/电压/错误）。
 * @param can         CAN 接口实例
 * @param can_id      电机 CAN ID
 * @param gear_ratio  减速比（用于速度换算，不同型号电机不同）
 * @param status      输出参数，存放读取到的状态
 * @return 成功返回 true
 */
bool readMotorStatus(CanInterface& can, uint32_t can_id, double gear_ratio,
                     MotorStatus& status);

/**
 * @brief 诊断：读取并打印电机的关键状态（位置/偏移/错误等）。
 * @param can     CAN 接口实例
 * @param can_id  电机 CAN ID
 */
void diagnose(CanInterface& can, uint32_t can_id);

/**
 * @brief 修改电机 CAN ID 并保存到 Flash（掉电保留）。
 * @param can     CAN 接口实例
 * @param cur_id  当前 CAN ID
 * @param new_id  新 CAN ID（1~127）
 * @return 成功返回 true
 */
bool changeCanId(CanInterface& can, uint32_t cur_id, uint32_t new_id);

/**
 * @brief 恢复出厂设置并保存。
 * @param can     CAN 接口实例
 * @param can_id  电机 CAN ID
 * @return 成功返回 true
 */
bool resetFactory(CanInterface& can, uint32_t can_id);

/**
 * @brief 标零：把当前物理位置标记为零点，并保存到 Flash（掉电保留）。
 * @param can     CAN 接口实例
 * @param can_id  电机 CAN ID
 * @return 成功返回 true
 */
bool setZeroPosition(CanInterface& can, uint32_t can_id);

/**
 * @brief 调整低压阈值（欠压保护阈值）。
 *
 * 流程：读母线电压 → 读旧阈值 → 设置新阈值 → 保存到 Flash → 读回确认。
 * 设置后需重新上电生效。
 *
 * @param can        CAN 接口实例
 * @param can_id     电机 CAN ID
 * @param voltage_v  新的低压阈值（V），应低于目标供电电压（如 28V 供电设为 24V）
 * @return 成功返回 true
 */
bool setLowVoltageThreshold(CanInterface& can, uint32_t can_id, int32_t voltage_v);

/**
 * @brief 单电机位置控制：回到零位后旋转指定角度。
 * @param can        CAN 接口实例
 * @param can_id     电机 CAN ID
 * @param angle_deg  旋转角度（度），逆时针为正
 * @param kp         位置环比例增益
 * @param kd         位置环微分增益
 * @return 成功返回 true
 */
bool rotateFromZero(CanInterface& can, uint32_t can_id, double angle_deg,
                    int32_t kp, int32_t kd);

} // namespace taihu

#endif // TAIHU_TAIHU_TOOLS_H
