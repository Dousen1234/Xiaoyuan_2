#include "taihu/joint_module.h"

#include <cstdint>

namespace taihu {

namespace {

// —— 协议功能码（依据《电机协议_双编_Can版本_20250623.xlsx》canbus 表） ——

// 写命令：5 字节（cmd + 4 字节小端 int32）
constexpr uint8_t kCmdStop        = 0x02; // 停止电机（1 字节）
constexpr uint8_t kCmdClearError  = 0x0B; // 清除电机错误（1 字节）
constexpr uint8_t kCmdSaveParams  = 0x0E; // 保存用户参数到 Flash（1 字节）
constexpr uint8_t kCmdResetFactory= 0x0F; // 恢复出厂设置（1 字节）
constexpr uint8_t kCmdSetCurrent  = 0x1C; // 设置目标电流
constexpr uint8_t kCmdSetVelocity = 0x1D; // 设置目标速度
constexpr uint8_t kCmdSetPosition = 0x1E; // 设置目标位置
constexpr uint8_t kCmdSetVelKp    = 0x29; // 速度环 KP
constexpr uint8_t kCmdSetVelKi    = 0x2A; // 速度环 KI
constexpr uint8_t kCmdSetPosKp    = 0x2B; // 位置环 KP
constexpr uint8_t kCmdSetPosKi    = 0x2C; // 位置环 KI
constexpr uint8_t kCmdSetPosKd    = 0x2D; // 位置环 KD
constexpr uint8_t kCmdSetCanId    = 0x2E; // 修改电机响应 CAN ID
constexpr uint8_t kCmdSetVelKd    = 0x33; // 速度环 KD
constexpr uint8_t kCmdSetZeroPos  = 0x50; // 编码器归零
constexpr uint8_t kCmdSetPosOffset= 0x53; // 设置位置偏移值
constexpr uint8_t kCmdGetPosOffset= 0x54; // 获取当前位置偏移

// 读命令：1 字节（cmd），电机返回 5 字节
constexpr uint8_t kCmdGetMode     = 0x03; // 获取运行模式
constexpr uint8_t kCmdGetCurrent  = 0x04; // 获取当前电流 (mA)
constexpr uint8_t kCmdGetVelocity = 0x06; // 获取当前速度 (0.01 Hz)
constexpr uint8_t kCmdGetPosition = 0x08; // 获取当前位置 (cnt)
constexpr uint8_t kCmdGetError    = 0x0A; // 获取错误状态
constexpr uint8_t kCmdGetBusVoltage = 0x14; // 获取母线电压 (V)
constexpr uint8_t kCmdGetCurVelPos  = 0x41; // 获取电流+速度+位置反馈（8 字节回复）
constexpr uint8_t kCmdSetLowVoltage = 0x89; // 设置低压阈值 (V)
constexpr uint8_t kCmdGetLowVoltage = 0x8C; // 获取低压阈值 (V)

} // namespace

JointModule::JointModule(CanInterface& can, uint32_t tx_id, uint32_t rx_id)
    : can_(can), tx_id_(tx_id), rx_id_(rx_id) {}

// —— 基本控制 ——

bool JointModule::stop() {
    CanFrame frame;
    frame.id       = tx_id_;
    frame.dlc      = 1;
    frame.data[0]  = kCmdStop;
    return can_.send(frame);
}

bool JointModule::clearError() {
    CanFrame frame;
    frame.id       = tx_id_;
    frame.dlc      = 1;
    frame.data[0]  = kCmdClearError;
    return can_.send(frame);
}

// —— 参数管理 ——

bool JointModule::setMotorId(uint32_t new_id) {
    return sendWriteCmd(kCmdSetCanId, static_cast<int32_t>(new_id));
}

bool JointModule::saveParams() {
    CanFrame frame;
    frame.id       = tx_id_;
    frame.dlc      = 1;
    frame.data[0]  = kCmdSaveParams;
    return can_.send(frame);
}

bool JointModule::resetFactory() {
    CanFrame frame;
    frame.id       = tx_id_;
    frame.dlc      = 1;
    frame.data[0]  = kCmdResetFactory;
    return can_.send(frame);
}

// —— 编码器零点（双编码器标零） ——

bool JointModule::setZeroPosition() {
    return sendWriteCmd(kCmdSetZeroPos, 0);
}

bool JointModule::setPositionOffset(int32_t offset_cnt) {
    return sendWriteCmd(kCmdSetPosOffset, offset_cnt);
}

bool JointModule::readPositionOffset(int32_t& offset_cnt) {
    return readInt32(kCmdGetPosOffset, offset_cnt);
}

bool JointModule::readPositionCnt(int32_t& cnt) {
    return readInt32(kCmdGetPosition, cnt);
}

// —— 位置控制 ——

bool JointModule::setTargetPosition(double position_rad) {
    return sendWriteCmd(kCmdSetPosition, radToCnt(position_rad));
}

bool JointModule::setTargetVelocity(double velocity_rad_s) {
    return sendWriteCmd(kCmdSetVelocity, radPerSToVel(velocity_rad_s));
}

bool JointModule::setTargetCurrent(int32_t current_ma) {
    return sendWriteCmd(kCmdSetCurrent, current_ma);
}

bool JointModule::setPositionKp(int32_t kp) {
    return sendWriteCmd(kCmdSetPosKp, kp);
}

bool JointModule::setPositionKd(int32_t kd) {
    return sendWriteCmd(kCmdSetPosKd, kd);
}

bool JointModule::setVelocityKp(int32_t kp) {
    return sendWriteCmd(kCmdSetVelKp, kp);
}

bool JointModule::setVelocityKi(int32_t ki) {
    return sendWriteCmd(kCmdSetVelKi, ki);
}

bool JointModule::setVelocityKd(int32_t kd) {
    return sendWriteCmd(kCmdSetVelKd, kd);
}

// —— 状态读取 ——

bool JointModule::readPosition(double& position_rad) {
    int32_t cnt = 0;
    if (!readInt32(kCmdGetPosition, cnt)) return false;
    position_rad = cntToRad(cnt);
    return true;
}

bool JointModule::readVelocity(double& velocity_rad_s) {
    int32_t vel = 0;
    if (!readInt32(kCmdGetVelocity, vel)) return false;
    velocity_rad_s = velToRadPerS(vel);
    return true;
}

bool JointModule::readCurrent(int32_t& current_ma) {
    return readInt32(kCmdGetCurrent, current_ma);
}

bool JointModule::readCurrentVelocityPosition(int32_t& current_ma,
                                              double& velocity_rad_s,
                                              double& position_rad) {
    // 0x41 一次读回电流+速度+位置（8 字节回复，无功能码回显）
    CanFrame tx;
    tx.id       = tx_id_;
    tx.dlc      = 1;
    tx.data[0]  = kCmdGetCurVelPos;

    can_.drain();
    if (!can_.send(tx)) return false;

    constexpr int kMaxAttempts = 5;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
        CanFrame rx;
        if (!can_.receive(rx, /*timeout_ms=*/200)) continue;
        if (rx.dlc < 8) continue;

        // 回复 8 字节：电流 int16、速度 int16、位置 int32（小端）
        const int16_t cur = static_cast<int16_t>(rx.data[0] | (rx.data[1] << 8));
        const int16_t vel = static_cast<int16_t>(rx.data[2] | (rx.data[3] << 8));
        const int32_t pos = static_cast<int32_t>(rx.data[4])
                          | (static_cast<int32_t>(rx.data[5]) << 8)
                          | (static_cast<int32_t>(rx.data[6]) << 16)
                          | (static_cast<int32_t>(rx.data[7]) << 24);

        current_ma     = cur;                 // mA
        velocity_rad_s = velToRadPerS(vel);   // 0.01Hz -> 输出端 rad/s
        position_rad   = cntToRad(pos);       // cnt -> rad
        return true;
    }
    return false;
}

bool JointModule::readErrorState(uint32_t& error_bits) {
    int32_t value = 0;
    if (!readInt32(kCmdGetError, value)) return false;
    error_bits = static_cast<uint32_t>(value);
    return true;
}

bool JointModule::readBusVoltage(int32_t& voltage_v) {
    return readInt32(kCmdGetBusVoltage, voltage_v);
}

bool JointModule::readLowVoltageThreshold(int32_t& voltage_v) {
    return readInt32(kCmdGetLowVoltage, voltage_v);
}

bool JointModule::setLowVoltageThreshold(int32_t voltage_v) {
    return sendWriteCmd(kCmdSetLowVoltage, voltage_v);
}

// —— 私有实现 ——

bool JointModule::sendWriteCmd(uint8_t cmd, int32_t data) {
    CanFrame frame;
    frame.id       = tx_id_;
    frame.dlc      = 5;
    frame.data[0]  = cmd;
    frame.data[1]  = static_cast<uint8_t>(data & 0xFF);
    frame.data[2]  = static_cast<uint8_t>((data >> 8) & 0xFF);
    frame.data[3]  = static_cast<uint8_t>((data >> 16) & 0xFF);
    frame.data[4]  = static_cast<uint8_t>((data >> 24) & 0xFF);
    return can_.send(frame);
}

bool JointModule::readInt32(uint8_t cmd, int32_t& value) {
    // 发送 1 字节读命令
    CanFrame tx;
    tx.id       = tx_id_;
    tx.dlc      = 1;
    tx.data[0]  = cmd;

    // 发送前清空残留帧，避免写命令响应干扰读命令
    can_.drain();

    if (!can_.send(tx)) return false;

    // 循环接收，跳过非目标功能码的响应帧（如写命令残留的反馈）
    constexpr int kMaxAttempts = 5;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
        CanFrame rx;
        if (!can_.receive(rx, /*timeout_ms=*/200)) continue; // 超时则继续重试
        if (rx.dlc < 5) continue;         // 数据不足，继续读
        if (rx.data[0] != cmd) continue;  // 功能码不匹配（残留帧），跳过

        // 第 1 字节功能码回显，后 4 字节为小端 int32
        value = static_cast<int32_t>(rx.data[1])
              | (static_cast<int32_t>(rx.data[2]) << 8)
              | (static_cast<int32_t>(rx.data[3]) << 16)
              | (static_cast<int32_t>(rx.data[4]) << 24);
        return true;
    }
    return false;
}

int32_t JointModule::radToCnt(double rad) {
    // 双编码器：262144 cnt = 2π rad（输出端）
    return static_cast<int32_t>(rad / kTwoPi * kEncoderCpr);
}

double JointModule::cntToRad(int32_t cnt) {
    return static_cast<double>(cnt) * kTwoPi / kEncoderCpr;
}

int32_t JointModule::radPerSToVel(double rad_s) const {
    // 0.01 Hz 为电机端转速单位；电机端转/秒 = (rad/s / 2π) * 减速比
    return static_cast<int32_t>(rad_s / kTwoPi * gear_ratio_ * 100.0);
}

double JointModule::velToRadPerS(int32_t vel) const {
    // 0.01 Hz 编码 -> 输出端 rad/s
    return static_cast<double>(vel) / 100.0 / gear_ratio_ * kTwoPi;
}

} // namespace taihu
