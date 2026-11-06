#ifndef TAIHU_TRAJECTORY_PACK_H
#define TAIHU_TRAJECTORY_PACK_H

/**
 * @file trajectory_pack.h
 * @brief 轨迹打包：输入一段 16 关节轨迹，输出对应的 16-ID CAN 指令帧。
 *
 * 本文件只做一件事——把关节角打包成 CAN 帧，不打开设备、不发送、不接收、
 * 不检查限位、不做任何 IO。发送节奏与发送时机完全由调用方决定。
 *
 * ---------------------------------------------------------------------------
 * 协议（上位机 → 下位机，标准帧 CAN 2.0A，1 Mbps）
 * ---------------------------------------------------------------------------
 *   每个关节一帧，共 16 帧：
 *
 *     CAN ID : 1 ~ 16            关节 ID = (腿号-1)*4 + (关节号)  ，从 1 开始
 *     DLC    : 5
 *     data[0]: 0x1E              功能码：设置目标位置
 *     data[1..4]: int32 小端     目标角度 = 关节端弧度 × 10000
 *
 *   例：CAN ID=3，目标 10° → 10° = 0.1745329 rad → ×10000 = 1745 = 0x000006D1
 *       帧内容：1E D1 06 00 00
 *
 *   ID 与腿/关节的对应（4 腿 × 4 关节，每条腿 YPPY 排布）：
 *
 *     ID  1  2  3  4 |  5  6  7  8 |  9 10 11 12 | 13 14 15 16
 *     腿  1  1  1  1 |  2  2  2  2 |  3  3  3  3 |  4  4  4  4
 *     关节 J1 J2 J3 J4 | J1 J2 J3 J4 | J1 J2 J3 J4 | J1 J2 J3 J4
 *
 * ---------------------------------------------------------------------------
 * 单位约定（重要）
 * ---------------------------------------------------------------------------
 * 本库输入的角度一律是【关节端弧度】。不做任何换算。
 *
 * 注意与 examples/taihu_singleleg_step_xCAN.cpp 区分：那个程序用的是
 * 【电机输出端角度(度)】。J2/J3 上有 1:2 锥齿轮，两者差 2 倍，不要混用。
 *
 * ---------------------------------------------------------------------------
 * 用法
 * ---------------------------------------------------------------------------
 * @code
 *   #include "taihu/trajectory_pack.h"
 *
 *   std::vector<taihu::TrajectoryPoint> traj;   // 你的轨迹
 *   // traj[i].t    采样时刻（秒）
 *   // traj[i].pose 16 个关节端弧度，pose[0] 对应 CAN ID 1
 *
 *   std::vector<taihu::TimedFrame> frames = taihu::packTrajectory(traj);
 *
 *   for (const auto& tf : frames) {
 *       can.send(tf.frame);       // 用你已有的 CAN 接口发出去
 *   }
 * @endcode
 *
 * 轨迹播完后如需退出下位机流模式，自行发一帧 [0x02]（任意一个关节 ID，
 * DLC=1）：@code taihu::CanFrame stop; stop.id = 1; stop.dlc = 1;
 * stop.data[0] = 0x02; @endcode
 */

#include <array>
#include <cstdint>
#include <vector>

#include "taihu/can_interface.h"

namespace taihu
{

    // ===========================================================================
    // 协议常量
    // ===========================================================================

    /// 关节总数（4 腿 × 4 关节）
    constexpr int kTrajTotalJoints = 16;
    /// 腿数
    constexpr int kTrajLegCount = 4;
    /// 每条腿的关节数
    constexpr int kTrajJointPerLeg = 4;
    /// 最小的关节 CAN ID
    constexpr int kTrajJointIdBase = 1;
    /// 角度定点缩放：int32 = 弧度 × 该系数
    constexpr double kTrajAngleScale = 10000.0;
    /// 功能码：设置目标位置
    constexpr uint8_t kTrajFuncSetPos = 0x1E;
    /// 功能码：退出流模式（由调用方自行组帧，本库不产出）
    constexpr uint8_t kTrajFuncStop = 0x02;

    // ===========================================================================
    // 类型
    // ===========================================================================

    /**
     * @brief 一个姿态：16 个关节的目标角度，单位【关节端弧度】。
     *
     * 索引 0 ↔ CAN ID 1，索引 15 ↔ CAN ID 16。
     */
    using JointPose = std::array<double, kTrajTotalJoints>;

    /// @brief 轨迹上的一个采样点。
    struct TrajectoryPoint
    {
        double t = 0.0;   ///< 采样时刻，单位秒（仅用于回传给调用方，不影响编码）
        JointPose pose{}; ///< 16 个关节端弧度
    };

    /// @brief 打包结果：一个 CAN 帧 + 它所属采样点的时刻。
    struct TimedFrame
    {
        double t = 0.0;   ///< 该帧所属采样点的时刻，单位秒
        CanFrame frame{}; ///< 已填好的 CAN 帧，可直接交给 send()
    };

    // ===========================================================================
    // 角度编解码
    // ===========================================================================

    /**
     * @brief 弧度 → 协议 int32 定点值（弧度 × 10000，四舍五入）。
     *
     * 超出 int32 范围时饱和到 INT32_MAX / INT32_MIN；NaN、Inf 返回 0
     * （调用方应先用 isFinite 判断，避免把 0 当成真实目标发出去）。
     */
    int32_t encodeAngle(double rad);

    /// @brief 协议 int32 定点值 → 弧度。
    double decodeAngle(int32_t raw);

    /// @brief 判断一个姿态里所有关节角是否都是有限值（非 NaN / 非 Inf）。
    bool isFinitePose(const JointPose &pose);

    // ===========================================================================
    // ID 映射
    // ===========================================================================

    /// @brief 腿号(0~3) + 关节号(0~3) → CAN ID(1~16)；越界返回 0。
    constexpr int jointId(int leg, int joint)
    {
        return (leg < 0 || leg >= kTrajLegCount || joint < 0 ||
                joint >= kTrajJointPerLeg)
                   ? 0
                   : leg * kTrajJointPerLeg + joint + kTrajJointIdBase;
    }

    /// @brief CAN ID → 腿号(0~3)；越界返回 -1。
    constexpr int idToLeg(int id)
    {
        return (id < kTrajJointIdBase || id >= kTrajJointIdBase + kTrajTotalJoints)
                   ? -1
                   : (id - kTrajJointIdBase) / kTrajJointPerLeg;
    }

    /// @brief CAN ID → 关节号(0~3)；越界返回 -1。
    constexpr int idToJoint(int id)
    {
        return (id < kTrajJointIdBase || id >= kTrajJointIdBase + kTrajTotalJoints)
                   ? -1
                   : (id - kTrajJointIdBase) % kTrajJointPerLeg;
    }

    // ===========================================================================
    // 核心打包
    // ===========================================================================

    /**
     * @brief 把一个姿态打包成 16 个 CAN 帧（CAN ID 1~16）。
     *
     * @param pose 16 个关节端弧度
     * @return 帧列表，正常情况长度 16。若某个关节角是 NaN/Inf，该关节的帧会被
     *         跳过（不会用 0 顶替），此时返回长度小于 16，顺序仍按 ID 递增。
     */
    std::vector<CanFrame> packPose(const JointPose &pose);

    /**
     * @brief 【主入口】把一整段轨迹打包成扁平帧数组（N 个采样点 → 最多 N×16 帧）。
     *
     * 输出顺序：先第 0 个采样点的 ID1..ID16，再第 1 个采样点的 ID1..ID16，以此类推。
     * 每个帧旁带上它所属采样点的时刻 t，方便调用方按自己的节奏发送。
     *
     * 若某个采样点里有 NaN/Inf 关节角，只跳过该关节的帧，其它照常输出。
     * 因此返回长度可能小于 N×16。
     *
     * @param traj 轨迹采样点序列
     * @return 扁平帧列表；traj 为空时返回空列表
     */
    std::vector<TimedFrame> packTrajectory(
        const std::vector<TrajectoryPoint> &traj);

} // namespace taihu

#endif // TAIHU_TRAJECTORY_PACK_H