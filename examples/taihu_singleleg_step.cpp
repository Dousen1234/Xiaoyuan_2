/**
 * @file taihu_singleleg_step.cpp
 * @brief 单腿步态控制示例：四电机（ID 1~4）三线程异步同步位置控制。
 *
 * 电机与减速比：
 *   - ID 1、ID 4：减速比 101
 *   - ID 2、ID 3：减速比 81
 *
 * 单腿步态（5 个阶段，每阶段 kStepSec 秒，阶段间线性插值保证平滑）：
 *   阶段 0（回零停顿）：四个电机全部回到零位并停顿；
 *   阶段 1（膝盖摆动）：ID 1/4 保持不动，ID 2 转 +180°，ID 3 转 -180°；
 *   阶段 2（髋转整圈）：ID 1 转 +360°，ID 4 转 -360°，ID 2/3 保持不动；
 *   阶段 3（膝盖回零）：ID 1/4 保持不动，ID 2 转 -180° 回零，ID 3 转 +180° 回零；
 *   阶段 4（回零停顿）：四个电机全部回到零位并停顿。
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
#include "taihu/damiao_usb2can.h"
#include "taihu/motor_logger.h"
#include "taihu/taihu_tools.h"

namespace {

constexpr const char* kSerialDevice = "/dev/ttyACM0"; // 串口设备（每条总线独立配置）

// —— 电机配置 ——
const std::vector<uint32_t> kMotorIds        = {1, 2, 3, 4};            // 四个电机的 CAN ID
const std::vector<double>   kMotorGearRatios = {101.0, 81.0, 81.0, 101.0}; // 对应减速比（ID1/4=101，ID2/3=81）

// —— 位置环增益 ——
constexpr int32_t kPositionKp = 4000;
constexpr int32_t kPositionKd = 60;

// —— 步态轨迹参数 ——
constexpr double kStepSec = 5.0; ///< 每个阶段时长 t (s)，按需修改

// 一个步态阶段里四个电机的目标角度（单位：度，逆时针为正，相对零位）
struct StepPose {
    double id1_deg;
    double id2_deg;
    double id3_deg;
    double id4_deg;
};

// 5 个阶段的目标角度（相邻阶段之间做五次多项式插值，保证位置/速度/加速度连续）
const std::vector<StepPose> kPhaseTargets = {
    {0.0,    0.0,    0.0,    0.0},    // 阶段 0：四电机回零，停顿
    {0.0,    -180.0, 180.0,  0.0},    // 阶段 1：ID1/4 不动，ID2 +180°，ID3 -180°
    {360.0,  -180.0, 180.0, -360.0},  // 阶段 2：ID1 +360°，ID4 -360°，ID2/3 不动
    {360.0,    0.0,    0.0, -360.0},  // 阶段 3：ID1/4 不动，ID2 -180° 回零，ID3 +180° 回零
    {0.0,      0.0,    0.0,    0.0},  // 阶段 4：四电机回零，停顿
};

constexpr int    kNumPhases = 5;                    ///< 阶段数
constexpr double kTotalSec  = kStepSec * kNumPhases; ///< 总运行时长 (s)

// 接收队列容量上限（防止记录跟不上时无限增长）
constexpr size_t kMaxQueueSize = 1000;

/// 五次多项式平滑函数（quintic smoothstep）。
/// 满足 s(0)=0、s(1)=1，且一阶、二阶导数在两端均为 0，
/// 用于相邻阶段之间的位置插值，保证位置、速度、加速度连续，轨迹更圆滑。
double quinticSmoothstep(double t) {
    return 6.0 * t * t * t * t * t - 15.0 * t * t * t * t + 10.0 * t * t * t;
}

/// 根据相对时间 t_rel（0 ~ kTotalSec）计算四个电机的目标角度（度）。
StepPose computeTarget(double t_rel) {
    if (t_rel < 0.0) return kPhaseTargets.front();
    if (t_rel >= kTotalSec) return kPhaseTargets.back();

    const int    phase = static_cast<int>(t_rel / kStepSec);          // 0 ~ kNumPhases-1
    if (phase >= kNumPhases - 1) return kPhaseTargets.back();         // 最后阶段（回零停顿）已到终点

    const double frac  = (t_rel - phase * kStepSec) / kStepSec;       // 0.0 ~ 1.0 归一化时间
    const double s     = quinticSmoothstep(frac);                     // 五次多项式插值系数

    const StepPose& a = kPhaseTargets[phase];
    const StepPose& b = kPhaseTargets[phase + 1];

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
    DamiaoUsb2CanInterface can;
    if (!can.open(kSerialDevice, 0)) {
        std::cerr << "[错误] 打开 " << kSerialDevice << " 失败" << std::endl;
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
        std::cerr << "[错误] 打开日志文件失败" << std::endl;
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
            if (t >= kTotalSec) break;  // 5 个阶段结束

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
