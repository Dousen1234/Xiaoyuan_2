/**
 * @file proto.h
 * @brief 3588 上位机 ⇄ 下位机 通讯协议常量（共享头，两边必须一致）
 *
 * 本协议沿用仓库既有 V2.0 关节位置流协议（docs/step_protocol.pdf），
 * 因此新写的 upper_control / lower_control 可以与旧的
 * tools/up_walk_control_V1.py、build/walk_control_V1 互操联调。
 *
 * 帧结构与钛虎电机写命令帧【同构】：DLC=5，[功能码 1B][int32 小端 4B]，
 * 但 int32 的语义被重定义为「关节端角度(rad) × 10000」。
 *
 *  上位机 → 下位机（标准帧，ID = 1~16 = 全局关节号）：
 *    [0x1E][int32 LE 目标关节端角(rad)×10000]   DLC=5，10Hz
 *    [0x02]                                     DLC=1，退出流模式（保持不抱闸）
 *    [0x03]                                     DLC=1，清除软限位报警锁定
 *    [0x41]                                     DLC=1，状态请求 → 立即回该 ID
 *
 *  下位机 → 上位机：
 *    ID 1~16 [0x41][int32 LE 实际关节端角(rad)×10000]   DLC=5，10Hz
 *    ID 0x161 [状态字节]                                DLC=1，200ms 心跳
 *              bit0 = 流模式中
 *              bit1 = 最近 1s 有样点
 *              bit2 = 软限位报警锁定中
 *    ID 0x162 [0x62][关节ID][侧别][0][int32 LE 当前实际角] DLC=8
 *              锁定期间每 200ms 重发；侧别 0x01=上限 0x02=下限
 *
 * 腿臂命名（本目录新约定，等价于仓库旧称 L1~L4）：
 *    腿臂 A ↔ can0 ↔ 旧 L1        A1 A2 A3 A4 ↔ L1_J1 L1_J2 L1_J3 L1_J4
 *    腿臂 B ↔ can1 ↔ 旧 L2        B1 B2 B3 B4 ↔ L2_J1 … L2_J4
 *    腿臂 C ↔ can2 ↔ 旧 L3
 *    腿臂 D ↔ can3 ↔ 旧 L4
 */

#ifndef X3588_PROTO_H
#define X3588_PROTO_H

#include <cmath>
#include <cstdint>

namespace x3588
{

    // =========================================================================
    // 身份映射
    // =========================================================================

    constexpr int kLegCount = 4;                          ///< 腿臂数 A~D
    constexpr int kJointCount = 4;                        ///< 每腿关节数 J1~J4
    constexpr int kTotalJoints = kLegCount * kJointCount; ///< 全局关节数 = 16

    /// 全局关节 ID 基址（ID 1~16）
    constexpr uint16_t kJointIdBase = 1;

    /// 全局关节 ID → 腿下标 0~3
    inline int idToLeg(uint16_t id) { return (id - kJointIdBase) / kJointCount; }

    /// 全局关节 ID → 关节下标 0~3
    inline int idToJoint(uint16_t id) { return (id - kJointIdBase) % kJointCount; }

    /// (腿下标, 关节下标) → 全局关节 ID
    inline uint16_t jointToId(int leg_idx, int joint_idx)
    {
        return static_cast<uint16_t>(kJointIdBase + leg_idx * kJointCount + joint_idx);
    }

    /// 是否为合法全局关节 ID（1~16）
    inline bool isJointId(uint16_t id)
    {
        return id >= kJointIdBase && id < kJointIdBase + kTotalJoints;
    }

    /// 腿名下标 → 字母（'A'~'D'）
    inline char legLetter(int leg_idx)
    {
        static const char kLetters[kLegCount] = {'A', 'B', 'C', 'D'};
        return kLetters[leg_idx & 3];
    }

    // =========================================================================
    // 角度定点标度
    // =========================================================================

    /// 协议 int32 = 关节端角度(rad) × 10000
    /// 1 LSB = 0.0001 rad ≈ 0.00573°，int32 范围 ±214748 rad，远大于任何关节行程
    constexpr double kAngleScale = 10000.0;

    // =========================================================================
    // 功能码
    // =========================================================================

    constexpr uint8_t kCmdSetPos = 0x1E;     ///< 目标位置（int32 = rad × 10000）
    constexpr uint8_t kCmdStop = 0x02;       ///< 退出流模式（电机保持，不抱闸）
    constexpr uint8_t kCmdClearLimit = 0x03; ///< 清除软限位报警锁定
    constexpr uint8_t kCmdGetState = 0x41;   ///< 状态请求 / 状态回复标记

    // =========================================================================
    // 广播帧
    // =========================================================================

    constexpr uint16_t kHeartbeatId = 0x161;  ///< 心跳帧 ID（避开关节 ID 1~16）
    constexpr uint16_t kLimitAlarmId = 0x162; ///< 软限位报警帧 ID

    constexpr uint8_t kLimitAlarmCode = 0x62; ///< 报警帧 data[0]
    constexpr uint8_t kLimitSideMax = 0x01;   ///< 撞上限
    constexpr uint8_t kLimitSideMin = 0x02;   ///< 撞下限

    constexpr uint8_t kHbBitStreaming = 0x01; ///< bit0：流模式中
    constexpr uint8_t kHbBitOnline = 0x02;    ///< bit1：最近 1s 有样点
    constexpr uint8_t kHbBitLimitLock = 0x04; ///< bit2：软限位报警锁定中

    // =========================================================================
    // 小端读写（协议统一小端）
    // =========================================================================

    inline void putI32LE(uint8_t *p, int32_t v)
    {
        p[0] = static_cast<uint8_t>(v & 0xFF);
        p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
        p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
        p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
    }

    inline int32_t getI32LE(const uint8_t *p)
    {
        return static_cast<int32_t>(static_cast<uint32_t>(p[0]) |
                                    (static_cast<uint32_t>(p[1]) << 8) |
                                    (static_cast<uint32_t>(p[2]) << 16) |
                                    (static_cast<uint32_t>(p[3]) << 24));
    }

    /// 关节端 rad → 协议 int32
    inline int32_t radToProto(double rad)
    {
        return static_cast<int32_t>(std::llround(rad * kAngleScale));
    }

    /// 协议 int32 → 关节端 rad
    inline double protoToRad(int32_t v)
    {
        return static_cast<double>(v) / kAngleScale;
    }

    // =========================================================================
    // 角度单位
    // =========================================================================

    constexpr double kDegToRad = 0.017453292519943295;
    constexpr double kRadToDeg = 57.29577951308232;

} // namespace x3588

#endif // X3588_PROTO_H