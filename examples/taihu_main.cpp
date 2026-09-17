/**
 * @file taihu_main.cpp
 * @brief 主演示程序：三线程异步控制两个电机，执行分段周期轨迹。
 *
 * 轨迹（每个大周期）：
 *   1. 两个电机在零点保持 2 秒；
 *   2. 同时运动 2 秒：电机 1 转 +90°，电机 2 转 -90°；
 *   3. 保持 1 秒；
 *   4. 2 秒内回到零点。
 * 上述大周期重复 3 次。
 *
 * - 发送线程：只负责向总线发送目标位置；
 * - 接收线程：只负责读取两个电机的返回数据，放入队列；
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
const std::vector<uint32_t> kMotorIds = {1,2};            // 两个电机的 CAN ID
const std::vector<double>   kMotorGearRatios = {101.0, 81.0}; // 对应减速比（不同型号）

// —— 位置环增益 ——
constexpr int32_t kPositionKp = 2000;
constexpr int32_t kPositionKd = 60;

// —— 轨迹参数 ——
constexpr double kInitSec = 2.0;          // 初始回到零点后等待的时间 (s)
constexpr double kHoldZeroSec = 2.0;      // 阶段 1：保持零点 (s)
constexpr double kMoveSec = 2.0;          // 阶段 2：正转 ±90° (s)
constexpr double kHoldTargetSec = 1.0;    // 阶段 3：保持目标等待 (s)
constexpr double kReturnSec = 2.0;        // 阶段 4：回零 (s)
constexpr double kAngleDeg = 90.0;        // 目标角度 (度)
constexpr int    kNumCycles = 3;          // 大周期次数

// 接收队列容量上限（防止记录跟不上时无限增长）
constexpr size_t kMaxQueueSize = 1000;

// 一个大周期的总时长
constexpr double kCycleSec = kHoldZeroSec + kMoveSec + kHoldTargetSec + kReturnSec;

// 两个电机的目标角度
struct TargetAngles {
    double m1_deg; // 电机 1 目标角度（度，逆时针为正）
    double m2_deg; // 电机 2 目标角度（度）
};

/// 根据周期内相对时间 t_rel（0 ~ kCycleSec）计算两个电机的目标角度。
TargetAngles computeTarget(double t_rel) {
    TargetAngles t;

    if (t_rel < kHoldZeroSec) {
        // 阶段 1：保持零点
        t.m1_deg = 0.0;
        t.m2_deg = 0.0;
    } else if (t_rel < kHoldZeroSec + kMoveSec) {
        // 阶段 2：同时正转（0 → ±90°，线性插值）
        const double p = (t_rel - kHoldZeroSec) / kMoveSec;
        t.m1_deg = kAngleDeg * p;
        t.m2_deg = -kAngleDeg * p;
    } else if (t_rel < kHoldZeroSec + kMoveSec + kHoldTargetSec) {
        // 阶段 3：保持目标（等待 1 秒）
        t.m1_deg = kAngleDeg;
        t.m2_deg = -kAngleDeg;
    } else {
        // 阶段 4：回到零点（线性插值）
        const double p = (t_rel - kHoldZeroSec - kMoveSec - kHoldTargetSec) / kReturnSec;
        t.m1_deg = kAngleDeg * (1.0 - p);
        t.m2_deg = -kAngleDeg * (1.0 - p);
    }
    return t;
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

    // 2. 定义并初始化两个电机
    std::vector<MotorConfig> configs = {
        {"motor_a", kMotorIds[0], kMotorGearRatios[0], kPositionKp, kPositionKd},
        {"motor_b", kMotorIds[1], kMotorGearRatios[1], kPositionKp, kPositionKd},
    };
    std::vector<std::unique_ptr<JointModule>> motors;
    for (const auto& cfg : configs) {
        auto m = initMotor(can, cfg);
        m->clearError();
        motors.push_back(std::move(m));
        std::printf("[信息] 已初始化电机 %s（ID=%u）\n", cfg.name.c_str(), cfg.can_id);
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

    // 5. 发送线程：按分段轨迹发目标位置
    std::thread send_thread([&]() {
        const auto start = std::chrono::steady_clock::now();

        // 初始：回到零点
        motors[0]->setTargetPosition(0.0);
        motors[1]->setTargetPosition(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(kInitSec)));
        std::printf("[信息] 已回到零点，开始周期轨迹\n");

        while (running.load()) {
            const double t = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start).count();
            const double t_cycle = t - kInitSec;   // 扣除初始回零时间
            if (t_cycle >= kCycleSec * kNumCycles) break;  // 3 个周期结束

            const double t_rel = std::fmod(t_cycle, kCycleSec);
            const TargetAngles tgt = computeTarget(t_rel);

            motors[0]->setTargetPosition(tgt.m1_deg * kDegToRad);
            motors[1]->setTargetPosition(tgt.m2_deg * kDegToRad);

            std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
        }
        running.store(false);
        queue_cv.notify_all();  // 唤醒阻塞在 wait 的记录线程，避免其无法退出
    });

    // 6. 接收线程：读两个电机状态，组合成 Sample 放入队列（带节流与容量上限）
    std::thread recv_thread([&]() {
        uint64_t sample_index = 0;
        while (running.load()) {
            MotorStatus m1, m2;
            if (readMotorStatus(can, kMotorIds[0], kMotorGearRatios[0], m1) &&
                readMotorStatus(can, kMotorIds[1], kMotorGearRatios[1], m2)) {
                Sample s;
                s.timestamp_s = static_cast<double>(sample_index) * kLogSamplePeriodSec;
                s.motors = {m1, m2};

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

    // 8. 等待各线程结束
    send_thread.join();
    recv_thread.join();
    log_thread.join();

    // 9. 清理
    logger.close();
    for (auto& m : motors) m->stop();
    can.close();
    std::printf("[信息] 结束\n");
    return 0;
}
