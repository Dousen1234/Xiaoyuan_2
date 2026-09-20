/**
 * @file taihu_singleleg_step.cpp
 * @brief 单腿步态控制示例：四电机（ID 1~4）三线程异步同步位置控制。
 *
 * 电机与减速比：
 *   - ID 1、ID 4：减速比 101
 *   - ID 2、ID 3：减速比 81
 *
 * 单腿步态（6 个阶段，每阶段 kStepSec 秒，相邻关键帧之间五次多项式插值保证平滑）：
 *   阶段 0（回零停顿）：四个电机全部回到零位并保持不动；
 *   阶段 1（膝盖摆动）：ID 1/4 保持不动，ID 2 转 -180°，ID 3 转 +360°；
 *   阶段 2（ID1 往复摆）：ID 2/3/4 保持不动，ID 1 完成 0° → 60° → -60° → 0°；
 *   阶段 3（膝盖回零）：ID 1/4 保持不动，ID 2/3 按与阶段 1 相反方向回到零位；
 *   阶段 4（ID4 往复摆）：ID 1/2/3 保持不动，ID 4 完成 0° → 90° → -90° → 0°；
 *   阶段 5（回零停顿）：四个电机全部回到零位并保持不动。
 *
 * 阶段 2/4 的往复运动在阶段内均分为 3 个子段（每子段 kStepSec/3 秒），
 * 每个子段端点为一个关键帧，子段间同样做五次多项式插值。
 *
 * - 发送线程：只负责按步态轨迹向总线发送目标位置；
 * - 接收线程：只负责读取四个电机的返回数据，放入队列；
 * - 记录线程：从队列取出数据，写入 motor_log.csv。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "taihu/config.h"
#include "taihu/motor_logger.h"
#include "taihu/socket_can.h"
#include "taihu/taihu_tools.h"

namespace {

constexpr const char* kCanDevice = "can0"; // CAN 通道（鲲弘 KH-UCANFDX6-Mini，can0~can5）

// —— 电机配置 ——
const std::vector<uint32_t> kMotorIds        = {1, 2, 3, 4};            // 四个电机的 CAN ID
const std::vector<double>   kMotorGearRatios = {101.0, 81.0, 81.0, 101.0}; // 对应减速比（ID1/4=101，ID2/3=81）

// —— 位置环增益 ——
constexpr int32_t kPositionKp = 4000;
constexpr int32_t kPositionKd = 60;

// —— 步态轨迹参数 ——
constexpr double kStepSec = 10.0; ///< 每个阶段时长 t (s)，按需修改

// 一个关键帧里四个电机的目标角度（单位：度，逆时针为正，相对零位）
struct StepPose {
    double id1_deg;
    double id2_deg;
    double id3_deg;
    double id4_deg;
};

// 6 个阶段的关键帧序列：每个阶段由 1 或 3 个等长子段组成，
// 子段端点即关键帧；相邻关键帧之间做五次多项式插值（位置/速度/加速度连续）。
//   单段阶段：{起点, 终点}（2 个关键帧，1 个子段，子段时长 = kStepSec）
//   往复阶段：{起点, 峰值, 谷值, 终点}（4 个关键帧，3 个子段，子段时长 = kStepSec/3）
const std::vector<std::vector<StepPose>> kPhaseKeyframes = {
    // 阶段 0：四电机回零，保持不动
    {{0.0, 0.0, 0.0, 0.0},
     {0.0, 0.0, 0.0, 0.0}},
    // 阶段 1：ID1/4 不动，ID2 -180°，ID3 +360°
    {{0.0,    0.0,   0.0,   0.0},
     {0.0, -180.0, 360.0,   0.0}},
    // 阶段 2：ID2/3/4 不动，ID1 往复 0° → 60° → -60° → 0°
    {{0.0, -180.0, 360.0, 0.0},
     {60.0, -180.0, 360.0, 0.0},
     {-60.0, -180.0, 360.0, 0.0},
     {0.0, -180.0, 360.0, 0.0}},
    // 阶段 3：ID1/4 不动，ID2/3 按与阶段 1 相反方向回零
    {{0.0, -180.0, 360.0, 0.0},
     {0.0,    0.0,   0.0, 0.0}},
    // 阶段 4：ID1/2/3 不动，ID4 往复 0° → 90° → -90° → 0°
    {{0.0, 0.0, 0.0,   0.0},
     {0.0, 0.0, 0.0,  90.0},
     {0.0, 0.0, 0.0, -90.0},
     {0.0, 0.0, 0.0,   0.0}},
    // 阶段 5：四电机回零，保持不动
    {{0.0, 0.0, 0.0, 0.0},
     {0.0, 0.0, 0.0, 0.0}},
};

constexpr int    kNumPhases = 6;                     ///< 阶段数
constexpr double kTotalSec  = kStepSec * kNumPhases; ///< 总运行时长 (s)

// 接收队列容量上限（防止记录跟不上时无限增长）
constexpr size_t kMaxQueueSize = 1000;

/// 五次多项式平滑函数（quintic smoothstep）。
/// 满足 s(0)=0、s(1)=1，且一阶、二阶导数在两端均为 0，
/// 用于相邻关键帧之间的位置插值，保证位置、速度、加速度连续，轨迹更圆滑。
double quinticSmoothstep(double t) {
    return 6.0 * t * t * t * t * t - 15.0 * t * t * t * t + 10.0 * t * t * t;
}

/// 根据相对时间 t_rel（0 ~ kTotalSec）计算四个电机的目标角度（度）。
/// 先定位阶段，再在该阶段的关键帧序列中定位子段并做五次多项式插值。
StepPose computeTarget(double t_rel) {
    if (t_rel < 0.0) return kPhaseKeyframes.front().front();
    if (t_rel >= kTotalSec) return kPhaseKeyframes.back().back();

    const int phase = static_cast<int>(t_rel / kStepSec);            // 0 ~ kNumPhases-1
    const std::vector<StepPose>& kf = kPhaseKeyframes[phase];
    const int    n_seg  = static_cast<int>(kf.size()) - 1;          // 该阶段子段数（1 或 3）
    const double seg_sec = kStepSec / n_seg;                         // 子段时长

    double t_phase = t_rel - phase * kStepSec;                       // 阶段内时间
    if (t_phase >= kStepSec) t_phase = kStepSec - 1e-9;              // 数值保护
    int    seg  = static_cast<int>(t_phase / seg_sec);
    if (seg >= n_seg) seg = n_seg - 1;
    const double frac = (t_phase - seg * seg_sec) / seg_sec;         // 子段内归一化时间
    const double s    = quinticSmoothstep(frac);

    const StepPose& a = kf[seg];
    const StepPose& b = kf[seg + 1];

    StepPose p;
    p.id1_deg = a.id1_deg + (b.id1_deg - a.id1_deg) * s;
    p.id2_deg = a.id2_deg + (b.id2_deg - a.id2_deg) * s;
    p.id3_deg = a.id3_deg + (b.id3_deg - a.id3_deg) * s;
    p.id4_deg = a.id4_deg + (b.id4_deg - a.id4_deg) * s;
    return p;
}

// 一个采样点：时间戳 + 各电机状态
struct Sample {
    double timestamp_s = 0.0;
    std::vector<taihu::MotorStatus> motors;
};

} // namespace

int main() {
    using namespace taihu;

    // 1. 打开 CAN 设备
    SocketCanInterface can;
    if (!can.open(kCanDevice, 0)) {
        std::cerr << "[错误] 打开 " << kCanDevice << " 失败，请确认已接上鲲弘 CANFD 模块且接口已 up（ip link set " << kCanDevice << " up）" << std::endl;
        return -1;
    }

    // 2. 定义并初始化四个电机
    std::vector<MotorConfig> configs = {
        {"motor_id1", kMotorIds[0], kMotorGearRatios[0], kPositionKp, kPositionKd},
        {"motor_id2", kMotorIds[1], kMotorGearRatios[1], kPositionKp, kPositionKd},
        {"motor_id3", kMotorIds[2], kMotorGearRatios[2], kPositionKp, kPositionKd},
        {"motor_id4", kMotorIds[3], kMotorGearRatios[3], kPositionKp, kPositionKd},
    };
    std::vector<std::unique_ptr<JointModule>> motors;
    for (const auto& cfg : configs) {
        auto m = initMotor(can, cfg);
        m->clearError();
        motors.push_back(std::move(m));
        std::printf("[信息] 已初始化电机 %s（ID=%u，减速比 %.0f）\n",
                    cfg.name.c_str(), cfg.can_id, cfg.gear_ratio);
    }

    // 3. 日志记录器
    MotorLogger logger("record/motor_log.csv");
    if (!logger.isOpen()) {
        std::cerr << "[错误] 打开日志文件失败: record/motor_log.csv" << std::endl;
        std::cerr << "        常见原因: 该文件由 root 创建(sudo 运行过)导致当前用户无写权限,"
                  << std::endl
                  << "        或 record/ 目录不存在。可执行: sudo chown $USER record/motor_log.csv"
                  << std::endl
                  << "        或删除旧文件: rm -f record/motor_log.csv" << std::endl;
        return -1;
    }
    logger.writeHeader(kMotorIds);

    // 4. 线程同步原语
    std::atomic<bool> running{true};
    std::queue<Sample> queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;

    // 5. 发送线程：按单腿步态轨迹发目标位置
    std::thread send_thread([&]() {
        const auto start = std::chrono::steady_clock::now();

        while (running.load()) {
            const double t = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start).count();
            if (t >= kTotalSec) break;  // 6 个阶段结束

            const StepPose pose = computeTarget(t);

            motors[0]->setTargetPosition(pose.id1_deg * kDegToRad);
            motors[1]->setTargetPosition(pose.id2_deg * kDegToRad);
            motors[2]->setTargetPosition(pose.id3_deg * kDegToRad);
            motors[3]->setTargetPosition(pose.id4_deg * kDegToRad);

            std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
        }
    });

    // 6. 接收线程：读四个电机状态，组合成 Sample 放入队列（带节流与容量上限）
    std::thread recv_thread([&]() {
        uint64_t sample_index = 0;
        while (running.load()) {
            std::vector<MotorStatus> statuses;
            statuses.reserve(kMotorIds.size());
            bool all_ok = true;
            for (size_t i = 0; i < kMotorIds.size(); ++i) {
                MotorStatus st;
                if (readMotorStatus(can, kMotorIds[i], kMotorGearRatios[i], st)) {
                    statuses.push_back(st);
                } else {
                    all_ok = false;
                    break;
                }
            }

            if (all_ok) {
                Sample s;
                // 均匀时间戳：采样序号 × 采样周期（采样周期 = 控制周期 × 5）
                s.timestamp_s = static_cast<double>(sample_index) * kLogSamplePeriodSec;
                s.motors = std::move(statuses);

                {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    if (queue.size() >= kMaxQueueSize) {
                        queue.pop();   // 队列已满，丢弃最旧采样，避免无限增长
                    }
                    queue.push(std::move(s));
                }
                queue_cv.notify_one();
                ++sample_index;
            }

            // 节流：按采样周期（控制周期 × 5）读取，保证时间戳均匀
            std::this_thread::sleep_for(std::chrono::milliseconds(kLogSamplePeriodMs));
        }
    });

    // 7. 记录线程：从队列取数据写日志
    std::thread log_thread([&]() {
        while (true) {
            Sample s;
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                queue_cv.wait(lock, [&] { return !queue.empty() || !running.load(); });
                if (queue.empty() && !running.load()) break;
                s = std::move(queue.front());
                queue.pop();
            }
            logger.log(s.timestamp_s, s.motors);
        }
    });

    // 8. 等待轨迹发送线程结束（轨迹运行完成）
    send_thread.join();

    // 9. 轨迹结束后，停止全部电机
    for (auto& m : motors) m->stop();

    // 10. 停止接收与记录线程（记录持续到程序即将完全结束）
    running.store(false);
    queue_cv.notify_all();  // 唤醒阻塞在 wait 的记录线程，让其检查 running 并退出
    recv_thread.join();
    log_thread.join();

    // 11. 清理
    logger.close();
    can.close();
    std::printf("[信息] 单腿步态控制结束\n");
    return 0;
}
