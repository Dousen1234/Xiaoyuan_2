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
 * - 接收线程：只负责读取各电机的返回数据，放入队列；
 * - 记录线程：从队列取出数据，写入 motor_log.csv。
 *
 * 用法:
 *   ./taihu_main                  默认两个电机 ID=1,2（减速比 101/81）
 *   ./taihu_main 2                单电机模式（ID=2，减速比默认 81）
 *   ./taihu_main 2:81 3:101       显式指定各电机 ID 与减速比
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

// —— 默认电机配置（可被命令行参数覆盖）——
const std::vector<uint32_t> kDefaultMotorIds = {1, 2};             // 默认两个电机的 CAN ID
const std::vector<double>   kDefaultGearRatios = {101.0, 81.0};    // 对应减速比（不同型号）

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

/// 根据周期内相对时间 t_rel（0 ~ kCycleSec）计算各电机目标角度（度）。
/// 电机 0 转 +kAngleDeg，电机 1 转 -kAngleDeg，更多电机按奇偶交替方向。
std::vector<double> computeTarget(double t_rel, size_t n) {
    double base = 0.0; // 当前幅度（度）

    if (t_rel < kHoldZeroSec) {
        base = 0.0;                                            // 阶段 1：保持零点
    } else if (t_rel < kHoldZeroSec + kMoveSec) {
        base = kAngleDeg * (t_rel - kHoldZeroSec) / kMoveSec;  // 阶段 2：线性转到目标
    } else if (t_rel < kHoldZeroSec + kMoveSec + kHoldTargetSec) {
        base = kAngleDeg;                                      // 阶段 3：保持目标
    } else {
        const double p = (t_rel - kHoldZeroSec - kMoveSec - kHoldTargetSec) / kReturnSec;
        base = kAngleDeg * (1.0 - p);                          // 阶段 4：回到零点
    }

    std::vector<double> angles(n);
    for (size_t i = 0; i < n; ++i) {
        angles[i] = (i % 2 == 0) ? base : -base; // 相邻电机反方向运动
    }
    return angles;
}

// 一个采样点：时间戳 + 各电机状态
struct Sample {
    double timestamp_s = 0.0;
    std::vector<taihu::MotorStatus> motors;
};

} // namespace

int main(int argc, char* argv[]) {
    using namespace taihu;

    // 0. 解析命令行：电机列表，格式 "ID[:减速比]"，空格分隔；默认 1:101 2:81
    std::vector<uint32_t> motor_ids = kDefaultMotorIds;
    std::vector<double>   gear_ratios = kDefaultGearRatios;
    if (argc >= 2) {
        motor_ids.clear();
        gear_ratios.clear();
        for (int i = 1; i < argc; ++i) {
            char* colon = std::strchr(argv[i], ':');
            uint32_t id = static_cast<uint32_t>(std::strtoul(argv[i], nullptr, 0));
            double ratio = 81.0;
            if (colon) ratio = std::strtod(colon + 1, nullptr);
            if (id == 0 || id > 127) {
                std::cerr << "[错误] 无效电机 ID: " << argv[i] << "（应为 1~127）" << std::endl;
                return -1;
            }
            motor_ids.push_back(id);
            gear_ratios.push_back(ratio);
        }
    }

    // 1. 打开 CAN 设备
    SocketCanInterface can;
    if (!can.open(kCanDevice, 0)) {
        std::cerr << "[错误] 打开 " << kCanDevice << " 失败，请确认已接上鲲弘 CANFD 模块且接口已 up（ip link set " << kCanDevice << " up）" << std::endl;
        return -1;
    }

    // 2. 定义并初始化各电机
    std::vector<MotorConfig> configs;
    for (size_t i = 0; i < motor_ids.size(); ++i) {
        configs.push_back({"motor_" + std::to_string(i), motor_ids[i], gear_ratios[i],
                           kPositionKp, kPositionKd});
    }
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
        std::cerr << "[错误] 打开日志文件失败: record/motor_log.csv" << std::endl;
        std::cerr << "        常见原因: 该文件由 root 创建(sudo 运行过)导致当前用户无写权限,"
                  << std::endl
                  << "        或 record/ 目录不存在。可执行: sudo chown $USER record/motor_log.csv"
                  << std::endl
                  << "        或删除旧文件: rm -f record/motor_log.csv" << std::endl;
        return -1;
    }
    logger.writeHeader(motor_ids);

    // 4. 线程同步原语
    std::atomic<bool> running{true};
    std::queue<Sample> queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;

    // 5. 发送线程：按分段轨迹发目标位置
    std::thread send_thread([&]() {
        const auto start = std::chrono::steady_clock::now();

        // 初始：回到零点
        for (auto& m : motors) m->setTargetPosition(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(kInitSec)));
        std::printf("[信息] 已回到零点，开始周期轨迹\n");

        while (running.load()) {
            const double t = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start).count();
            const double t_cycle = t - kInitSec;   // 扣除初始回零时间
            if (t_cycle >= kCycleSec * kNumCycles) break;  // 3 个周期结束

            const double t_rel = std::fmod(t_cycle, kCycleSec);
            const std::vector<double> angles = computeTarget(t_rel, motors.size());

            for (size_t i = 0; i < motors.size(); ++i) {
                motors[i]->setTargetPosition(angles[i] * kDegToRad);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
        }
        running.store(false);
        queue_cv.notify_all();  // 唤醒阻塞在 wait 的记录线程，避免其无法退出
    });

    // 6. 接收线程：读各电机状态，组合成 Sample 放入队列（带节流与容量上限）
    std::thread recv_thread([&]() {
        uint64_t sample_index = 0;
        while (running.load()) {
            std::vector<MotorStatus> statuses(motor_ids.size());
            bool all_ok = true;
            for (size_t i = 0; i < motor_ids.size(); ++i) {
                if (!readMotorStatus(can, motor_ids[i], gear_ratios[i], statuses[i])) {
                    all_ok = false;
                    break;
                }
            }
            if (all_ok) {
                Sample s;
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
