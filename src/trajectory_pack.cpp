/**
 * @file trajectory_pack.cpp
 * @brief 轨迹打包实现。纯计算，无 IO、无状态、线程安全。
 */

#include "taihu/trajectory_pack.h"

#include <cmath>
#include <limits>

namespace taihu
{
    namespace
    {

        /// 把一个 int32 按小端序写入 4 字节。
        void putI32Le(uint8_t *out, int32_t v)
        {
            const uint32_t u = static_cast<uint32_t>(v);
            out[0] = static_cast<uint8_t>(u & 0xFFu);
            out[1] = static_cast<uint8_t>((u >> 8) & 0xFFu);
            out[2] = static_cast<uint8_t>((u >> 16) & 0xFFu);
            out[3] = static_cast<uint8_t>((u >> 24) & 0xFFu);
        }

    } // namespace

    int32_t encodeAngle(double rad)
    {
        if (!std::isfinite(rad))
            return 0;

        const double v = rad * kTrajAngleScale;
        if (v >= static_cast<double>(std::numeric_limits<int32_t>::max()))
            return std::numeric_limits<int32_t>::max();
        if (v <= static_cast<double>(std::numeric_limits<int32_t>::min()))
            return std::numeric_limits<int32_t>::min();

        // std::lround 是四舍五入（远离零），与下位机 Python 侧一致
        return static_cast<int32_t>(std::lround(v));
    }

    double decodeAngle(int32_t raw)
    {
        return static_cast<double>(raw) / kTrajAngleScale;
    }

    bool isFinitePose(const JointPose &pose)
    {
        for (double a : pose)
        {
            if (!std::isfinite(a))
                return false;
        }
        return true;
    }

    std::vector<CanFrame> packPose(const JointPose &pose)
    {
        std::vector<CanFrame> out;
        out.reserve(kTrajTotalJoints);

        for (int i = 0; i < kTrajTotalJoints; ++i)
        {
            const double rad = pose[static_cast<std::size_t>(i)];
            if (!std::isfinite(rad))
                continue; // 非法角度跳过，不冒充 0

            CanFrame f;
            f.id = static_cast<uint32_t>(i + kTrajJointIdBase);
            f.dlc = 5;
            f.is_extended = false;
            f.data[0] = kTrajFuncSetPos;
            putI32Le(&f.data[1], encodeAngle(rad));
            out.push_back(f);
        }
        return out;
    }

    std::vector<TimedFrame> packTrajectory(
        const std::vector<TrajectoryPoint> &traj)
    {
        std::vector<TimedFrame> out;
        out.reserve(traj.size() * static_cast<std::size_t>(kTrajTotalJoints));

        for (const TrajectoryPoint &pt : traj)
        {
            const std::vector<CanFrame> frames = packPose(pt.pose);
            for (const CanFrame &f : frames)
            {
                out.push_back(TimedFrame{pt.t, f});
            }
        }
        return out;
    }

} // namespace taihu