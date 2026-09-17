#ifndef TAIHU_JOINT_MODULE_H
#define TAIHU_JOINT_MODULE_H

#include <cstdint>

#include "taihu/can_interface.h"

namespace taihu {

/**
 * @brief 钛虎 CRA 系列关节模组控制封装。
 *
 * 目标型号: CRA-RI50-60-PRO-2-81-B-2E-EC（双编码器，减速比 81）
 * 通讯方式: CAN，波特率 1M，默认 CAN ID = 1
 *
 * 协议要点（依据《电机协议_双编_Can版本_20250623.xlsx》）：
 *   - 写命令帧: [功能码(1 字节)][数据(4 字节, 小端 Int32)]，共 5 字节
 *   - 读命令帧: [功能码(1 字节)]，共 1 字节，电机返回 5 字节数据
 *   - 位置: 双编码器 262144 cnt = 360°（输出端，减速比按 1 计）
 *   - 速度: 单位 0.01 Hz（电机端转速）
 */
class JointModule {
public:
    /**
     * @brief 构造关节模组对象。
     * @param can    CAN 接口实例
     * @param tx_id  发送给电机的 CAN 标识符（默认 = 电机 ID，即 1）
     * @param rx_id  接收电机反馈的 CAN 标识符（默认 = 电机 ID，即 1）
     */
    explicit JointModule(CanInterface& can, uint32_t tx_id = 1, uint32_t rx_id = 1);

    // —— 基本控制 ——

    /// 停止电机：立即停止并抱死刹车（cmd=0x02）。
    bool stop();

    /// 清除锁存的电机错误，如欠压/过流等（cmd=0x0B）。
    bool clearError();

    // —— 参数管理 ——

    /**
     * @brief 修改电机 CAN ID（cmd=0x2E），范围 1~127，设置后立刻生效。
     * @note  如需掉电保留，请随后调用 saveParams()。
     */
    bool setMotorId(uint32_t new_id);

    /// 保存用户参数到 Flash（cmd=0x0E），掉电后仍保留。需在电机停止时操作。
    bool saveParams();

    /// 恢复出厂设置（cmd=0x0F）。文档注明恢复后需再调用 saveParams() 保存。
    bool resetFactory();

    // —— 编码器零点（双编码器标零） ——

    /// 编码器归零：标记当前位置为零点（cmd=0x50）。需随后 saveParams() 掉电保留。
    bool setZeroPosition();

    /// 设置位置偏移值（cmd=0x53），软标零：目标位置 = 编码器位置 - 偏移值。
    bool setPositionOffset(int32_t offset_cnt);

    /// 读取当前位置偏移值（cmd=0x54）。
    bool readPositionOffset(int32_t& offset_cnt);

    /// 读取原始编码器位置 cnt（cmd=0x08，不转换为弧度）。
    bool readPositionCnt(int32_t& cnt);

    // —— 位置控制 ——

    /**
     * @brief 位置模式：设置目标位置（cmd=0x1E），自动进入位置模式。
     * @param position_rad 目标角度（输出端），单位弧度。
     */
    bool setTargetPosition(double position_rad);

    /// 速度模式：设置目标速度（cmd=0x1D）。
    bool setTargetVelocity(double velocity_rad_s);

    /// 电流模式：设置目标电流（cmd=0x1C），单位 mA。
    /// 电流设为 0 即「使能但无转矩输出」，用于解除抱闸（打开刹车）让电机可自由旋转。
    bool setTargetCurrent(int32_t current_ma);

    /// 设置位置环比例增益 KP（cmd=0x2B）。
    bool setPositionKp(int32_t kp);

    /// 设置位置环微分增益 KD（cmd=0x2D）。
    bool setPositionKd(int32_t kd);

    /// 设置速度环比例/积分/微分增益（cmd=0x29 / 0x2A / 0x33）。
    bool setVelocityKp(int32_t kp);
    bool setVelocityKi(int32_t ki);
    bool setVelocityKd(int32_t kd);

    // —— 状态读取 ——

    /// 读取当前位置（cmd=0x08），输出单位为弧度（输出端）。
    bool readPosition(double& position_rad);

    /// 读取当前速度（cmd=0x06），输出单位为 rad/s（输出端）。
    bool readVelocity(double& velocity_rad_s);

    /// 读取当前电流（cmd=0x04），单位 mA。
    bool readCurrent(int32_t& current_ma);

    /// 一次性读取电流+速度+位置（cmd=0x41，8 字节回复），用于高频采样减少读次数。
    /// @param current_ma     输出：电流 (mA)
    /// @param velocity_rad_s 输出：速度 (rad/s，输出端)
    /// @param position_rad   输出：位置 (rad，输出端)
    bool readCurrentVelocityPosition(int32_t& current_ma,
                                     double& velocity_rad_s,
                                     double& position_rad);

    /// 读取错误状态位（cmd=0x0A）。
    bool readErrorState(uint32_t& error_bits);

    /// 读取母线电压（cmd=0x14），单位 V。
    bool readBusVoltage(int32_t& voltage_v);

    /// 读取低压阈值（cmd=0x8C），单位 V。
    bool readLowVoltageThreshold(int32_t& voltage_v);

    /// 设置低压阈值（cmd=0x89），单位 V，重启生效。
    bool setLowVoltageThreshold(int32_t voltage_v);

    /// 设置减速比（默认 81，来自型号 "2-81"）。
    void setGearRatio(double ratio) { gear_ratio_ = ratio; }

private:
    // —— 协议换算常量 ——
    static constexpr double kPi = 3.14159265358979323846; ///< 圆周率
    static constexpr double kTwoPi = 2.0 * kPi;           ///< 2π
    static constexpr double kEncoderCpr = 262144.0;       ///< 双编码器一圈 cnt 数
    static constexpr double kDefaultGearRatio = 81.0;     ///< 默认减速比

    CanInterface& can_;              ///< CAN 接口引用
    uint32_t tx_id_;                 ///< 发送 CAN ID
    uint32_t rx_id_;                 ///< 接收 CAN ID
    double gear_ratio_ = kDefaultGearRatio;

    /// 发送 5 字节写命令: [cmd][int32 小端数据]。
    bool sendWriteCmd(uint8_t cmd, int32_t data);

    /// 发送 1 字节读命令，接收 5 字节返回并解析出 int32。
    bool readInt32(uint8_t cmd, int32_t& value);

    /// 弧度（输出端）-> 编码 cnt（双编码器）。
    static int32_t radToCnt(double rad);

    /// 编码 cnt -> 弧度（输出端）。
    static double cntToRad(int32_t cnt);

    /// 输出端速度 (rad/s) -> 电机端 0.01 Hz 编码。
    int32_t radPerSToVel(double rad_s) const;

    /// 电机端 0.01 Hz 编码 -> 输出端速度 (rad/s)。
    double velToRadPerS(int32_t vel) const;
};

} // namespace taihu

#endif // TAIHU_JOINT_MODULE_H
