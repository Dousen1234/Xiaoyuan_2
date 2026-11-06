/**
 * @file trajectory_pack_demo.cpp
 * @brief packTrajectory() 的最小用法演示：不打开任何硬件，只把帧打印出来。
 *
 * 编译：cmake --build build --target trajectory_pack_demo
 * 运行：./build/trajectory_pack_demo
 */

#include <cmath>
#include <cstdio>
#include <vector>

#include "taihu/trajectory_pack.h"

namespace
{

    constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

    /// 造一段示例轨迹：2 秒、50 Hz（共 101 个点），只有每条腿的 J3 在 ±2° 内摆动。
    std::vector<taihu::TrajectoryPoint> makeDemoTrajectory()
    {
        std::vector<taihu::TrajectoryPoint> traj;

        const int kCount = 101;
        const double kDt = 0.02;
        const double kAmpDeg = 2.0;

        for (int k = 0; k < kCount; ++k)
        {
            taihu::TrajectoryPoint pt;
            pt.t = k * kDt;

            const double phase =
                2.0 * 3.14159265358979323846 * (static_cast<double>(k) / (kCount - 1));
            const double j3 = kAmpDeg * std::sin(phase) * kDeg2Rad;

            for (int leg = 0; leg < taihu::kTrajLegCount; ++leg)
            {
                const int idx = taihu::jointId(leg, 2) - taihu::kTrajJointIdBase; // J3
                pt.pose[static_cast<std::size_t>(idx)] = j3;
            }
            traj.push_back(pt);
        }
        return traj;
    }

    void printFrame(const taihu::TimedFrame &tf)
    {
        std::printf("t=%.2fs  ID=%2u  DLC=%u  ", tf.t, tf.frame.id,
                    static_cast<unsigned>(tf.frame.dlc));
        for (int i = 0; i < tf.frame.dlc; ++i)
        {
            std::printf("%02X ", tf.frame.data[i]);
        }
        std::printf("\n");
    }

} // namespace

int main()
{
    // ---- 1. 准备轨迹（真实使用时换成你自己模块给出的数据）----
    const std::vector<taihu::TrajectoryPoint> traj = makeDemoTrajectory();
    std::printf("轨迹：%zu 个采样点，时长 %.2f s\n\n", traj.size(),
                traj.back().t - traj.front().t);

    // ---- 2. 一行调用完成打包 ----
    const std::vector<taihu::TimedFrame> frames = taihu::packTrajectory(traj);

    std::printf("打包结果：%zu 帧（期望 %zu 帧 = %zu 点 × %d 关节）\n\n",
                frames.size(), traj.size() * taihu::kTrajTotalJoints,
                traj.size(), taihu::kTrajTotalJoints);

    // ---- 3. 打印前 6 帧，看看长什么样 ----
    std::printf("前 6 帧：\n");
    for (std::size_t i = 0; i < frames.size() && i < 6; ++i)
    {
        printFrame(frames[i]);
    }

    // ---- 4. 单独演示一个已知角度，方便逐字节核对 ----
    std::printf("\n单点核对（CAN ID 3 = 腿1 J3）：\n");
    taihu::JointPose pose{};
    pose[2] = 10.0 * kDeg2Rad; // 10°
    const std::vector<taihu::CanFrame> frames10 = taihu::packPose(pose);
    const taihu::CanFrame &f10 = frames10[2]; // 索引 2 ↔ ID 3
    std::printf("  +10°  ID=%u  ", f10.id);
    for (int i = 0; i < f10.dlc; ++i)
        std::printf("%02X ", f10.data[i]);
    std::printf("  → 应为 1E D1 06 00 00（1745 = 0x06D1）\n");

    pose[2] = -10.0 * kDeg2Rad;
    const std::vector<taihu::CanFrame> framesm10 = taihu::packPose(pose);
    const taihu::CanFrame &fm10 = framesm10[2];
    std::printf("  -10°  ID=%u  ", fm10.id);
    for (int i = 0; i < fm10.dlc; ++i)
        std::printf("%02X ", fm10.data[i]);
    std::printf("  → 应为 1E 2F F9 FF FF（-1745 补码）\n");

    // packPose 对全 0 姿态也会返回 16 帧（0 是合法目标角），不会裁剪。
    std::printf("\n说明：packPose() 恒定返回 16 帧（ID 1~16 各一帧）；"
                "只有当某个关节角是 NaN/Inf 时，才会跳过该帧。\n");
    return 0;
}