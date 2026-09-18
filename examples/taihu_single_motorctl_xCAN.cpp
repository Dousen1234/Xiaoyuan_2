/**
 * @file taihu_single_motorctl_xCAN.cpp
 * @brief 多 CAN 总线电机同步控制示例（基于 taihu_single_motorctl 扩展）。
 *
 * 与 taihu_single_motorctl 的唯一区别：
 *   - 支持多条 CAN 总线（如 can0 / can1），每条总线可挂多个电机；
 *   - 每条总线由独立线程控制；
 *   - 所有线程通过共享「统一起跑时刻」对齐时序，多条总线上的电机同时开始运动。
 *
 * 对单个电机的控制方法与 taihu_single_motorctl 完全一致：
 *   - 模式 1：位置控制（五次多项式轨迹，指定时间内平滑到达目标角度）；
 *   - 模式 2：正弦轨迹测试。
 *
 * 运行时交互输入：
 *   - CAN 总线数量（1~6，对应鲲弘 KH-UCANFDX6-Mini 的 can0~can5）；
 *   - 每条总线的接口名（可输入 "can0" 或通道号 "0"，自动补全为 canN）
 *     与该总线上要控制的电机数量；
 *   - 每个电机的 CAN ID 与减速比；
 *   - 控制模式；模式 1 还需输入目标角度（度）与完成时间（秒）。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "taihu/config.h"
#include "taihu/joint_module.h"
#include "taihu/socket_can.h"

namespace {

// —— 位置环增益（需按实际负载调试，与 taihu_single_motorctl 一致） ——
constexpr int32_t kPositionKp = 4000;
constexpr int32_t kPositionKd = 60;

// —— 模式 2：正弦轨迹参数（与 taihu_single_motorctl 一致） ——
constexpr double kAmplitude        = 0.5;   ///< 正弦幅度 (rad)
constexpr double kFrequency        = 0.5;   ///< 频率 (Hz)
constexpr double kTotalDurationSec = 10.0;  ///< 总运行时长 (s)

// —— 多总线同步参数 ——
constexpr int kMaxBusCount    = 6;   ///< 鲲弘设备最多 6 路通道（can0~can5）
constexpr int kLeadTimeMs     = 300; ///< 全部就绪后到统一起跑时刻的提前量 (ms)

/// 五次多项式平滑函数（quintic smoothstep），与 taihu_single_motorctl 一致。
double quinticSmoothstep(double t) {
    return 6.0 * t * t * t * t * t - 15.0 * t * t * t * t + 10.0 * t * t * t;
}

/// 单个电机的配置与运行时状态（属于某条总线）。
struct MotorCtx {
    uint32_t can_id   = 1;
    double gear_ratio = 81.0;
    double pos_start  = 0.0;  ///< 轨迹起点（准备阶段读取）
    std::unique_ptr<taihu::JointModule> joint;
};

/// 一条 CAN 总线的配置与该总线上所有电机。
struct BusCtx {
    std::string ifname;               ///< 接口名，如 "can0"
    std::vector<MotorCtx> motors;     ///< 该总线上的电机
    taihu::SocketCanInterface can;    ///< 每条总线独立的 SocketCAN 实例
    bool ok = false;                  ///< 该总线线程执行结果
    std::string err;                  ///< 失败原因
};

// —— 多线程同步用的共享状态 ——
std::atomic<int>  g_ready_count{0};   ///< 已完成准备（含读起始位置）的总线数
std::atomic<bool> g_failed{false};    ///< 任一总线准备失败
std::atomic<bool> g_go{false};        ///< 主线程放行信号
std::chrono::steady_clock::time_point g_start_time; ///< 统一起跑时刻

/// 输入一个正整数，带默认值与范围检查。
int inputInt(const char* prompt, int def, int lo, int hi) {
    std::printf("%s", prompt);
    int v = def;
    if (std::scanf("%d", &v) != 1 || v < lo || v > hi) {
        std::printf("[提示] 输入无效，使用默认值 %d\n", def);
        return def;
    }
    return v;
}

/// 输入一个正浮点数，带默认值检查。
double inputDouble(const char* prompt, double def) {
    std::printf("%s", prompt);
    double v = def;
    if (std::scanf("%lf", &v) != 1) {
        std::printf("[提示] 输入无效，使用默认值 %.3f\n", def);
        return def;
    }
    return v;
}

/// 归一化接口名输入：纯数字（0~5）自动补全为 canN，其余原样返回。
/// 例如输入 "0" -> "can0"，输入 "can1" -> "can1"。
std::string normalizeIfname(const char* raw) {
    char* end = nullptr;
    const long v = std::strtol(raw, &end, 10);
    if (end != raw && *end == '\0' && v >= 0 && v < kMaxBusCount) {
        char buf[16];
        std::snprintf(buf, sizeof buf, "can%ld", v);
        return buf;
    }
    return raw;
}

// =====================================================================
// 模式 1：位置控制（五次多项式轨迹）—— 一条总线上的所有电机同步执行
// =====================================================================
void runPositionMode(BusCtx& bus, double target_rad, double duration_sec) {
    // 统一起跑时刻之前自旋等待（yield，保证亚毫秒级对齐）
    while (std::chrono::steady_clock::now() < g_start_time)
        std::this_thread::yield();

    while (true) {
        const double t = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - g_start_time).count();
        if (t >= duration_sec) break;

        const double s = quinticSmoothstep(t / duration_sec);
        for (auto& m : bus.motors) {
            const double target = m.pos_start + (target_rad - m.pos_start) * s;
            if (!m.joint->setTargetPosition(target)) {
                std::printf("[%s][错误] 电机 %u 发送位置指令失败\n",
                            bus.ifname.c_str(), m.can_id);
                bus.ok = false;
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(taihu::kLoopPeriodMs));
    }

    // 最终精确到位
    for (auto& m : bus.motors)
        m.joint->setTargetPosition(target_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    for (auto& m : bus.motors)
        m.joint->stop();

    std::printf("[%s][信息] 位置控制结束\n", bus.ifname.c_str());
    bus.ok = true;
}

// =====================================================================
// 模式 2：正弦轨迹测试 —— 一条总线上的所有电机同步执行
// =====================================================================
void runSinusoidal(BusCtx& bus) {
    // 统一起跑时刻之前自旋等待
    while (std::chrono::steady_clock::now() < g_start_time)
        std::this_thread::yield();

    while (true) {
        const double t = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - g_start_time).count();
        if (t >= kTotalDurationSec) break;

        const double target = kAmplitude * std::sin(2.0 * M_PI * kFrequency * t);
        for (auto& m : bus.motors) {
            if (!m.joint->setTargetPosition(target)) {
                std::printf("[%s][错误] 电机 %u 发送位置指令失败\n",
                            bus.ifname.c_str(), m.can_id);
                bus.ok = false;
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(taihu::kLoopPeriodMs));
    }

    for (auto& m : bus.motors)
        m.joint->stop();

    std::printf("[%s][信息] 正弦轨迹测试结束\n", bus.ifname.c_str());
    bus.ok = true;
}

// =====================================================================
// 总线线程：打开接口 -> 准备所有电机 -> 等待统一起跑 -> 执行控制
// =====================================================================
void busThread(BusCtx& bus, int mode, double target_rad, double duration_sec) {
    using namespace taihu;

    // 1. 打开该总线的 CAN 接口
    if (!bus.can.open(bus.ifname.c_str(), 0)) {
        char buf[256];
        std::snprintf(buf, sizeof buf,
                      "打开 %s 失败，请确认接口已 up（sudo ./build/kcanctl up %s 1000000）",
                      bus.ifname.c_str(), bus.ifname.c_str());
        bus.err = buf;
        bus.ok = false;
        g_failed = true;
        return;
    }

    // 2. 准备该总线上的所有电机（清错、设增益、读起始位置）
    for (auto& m : bus.motors) {
        m.joint = std::make_unique<JointModule>(bus.can, m.can_id, m.can_id);
        m.joint->setGearRatio(m.gear_ratio);

        m.joint->clearError();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        m.joint->setPositionKp(kPositionKp);
        m.joint->setPositionKd(kPositionKd);

        if (!m.joint->readPosition(m.pos_start)) {
            char buf[256];
            std::snprintf(buf, sizeof buf, "电机 %u 读取当前位置失败（ID 不匹配或未上电？）",
                          m.can_id);
            bus.err = buf;
            bus.ok = false;
            bus.can.close();
            g_failed = true;
            return;
        }
        std::printf("[%s][信息] 电机 %u 起始位置 %.3f rad\n",
                    bus.ifname.c_str(), m.can_id, m.pos_start);
    }

    // 3. 报告就绪，等待主线程放行 + 统一起跑时刻
    g_ready_count++;
    while (!g_go.load() && !g_failed.load())
        std::this_thread::yield();
    if (g_failed.load()) {
        bus.can.close();
        return;
    }

    // 4. 按模式执行控制
    if (mode == 1)
        runPositionMode(bus, target_rad, duration_sec);
    else
        runSinusoidal(bus);

    bus.can.close();
}

} // namespace

int main() {
    std::printf("========================================\n");
    std::printf("  多 CAN 总线电机同步控制 demo (xCAN)\n");
    std::printf("========================================\n");

    // 1. 输入总线数量
    const int bus_count = inputInt(
        "请输入 CAN 总线数量（1~6，对应 can0~can5，默认 2）：", 2, 1, kMaxBusCount);

    // 2. 逐条总线输入接口名与电机列表
    std::vector<std::unique_ptr<BusCtx>> buses;
    for (int i = 0; i < bus_count; i++) {
        auto bus = std::make_unique<BusCtx>();

        char default_if[16];
        std::snprintf(default_if, sizeof default_if, "can%d", i);
        char ifbuf[32] = {0};
        std::printf("总线 %d：请输入 CAN 接口名或通道号（默认 %s）：", i + 1, default_if);
        if (std::scanf("%31s", ifbuf) != 1 || ifbuf[0] == '\0')
            std::snprintf(ifbuf, sizeof ifbuf, "%s", default_if);
        bus->ifname = normalizeIfname(ifbuf);

        char prompt[128];
        std::snprintf(prompt, sizeof prompt,
                      "总线 %d（%s）：请输入电机数量（默认 1）：", i + 1, bus->ifname.c_str());
        const int motor_count = inputInt(prompt, 1, 1, 127);

        for (int j = 0; j < motor_count; j++) {
            MotorCtx m;
            std::snprintf(prompt, sizeof prompt,
                          "  电机 %d：请输入 CAN ID（1~127，默认 1）：", j + 1);
            m.can_id = static_cast<uint32_t>(inputInt(prompt, 1, 1, 127));
            std::snprintf(prompt, sizeof prompt,
                          "  电机 %d：请输入减速比（默认 81）：", j + 1);
            m.gear_ratio = inputDouble(prompt, 81.0);
            bus->motors.push_back(std::move(m));
        }
        buses.push_back(std::move(bus));
    }

    // 3. 选择控制模式（所有总线/电机使用同一模式与参数）
    std::printf("----------------------------------------\n");
    std::printf("  1 —— 位置控制（五次多项式轨迹）\n");
    std::printf("  2 —— 正弦轨迹测试\n");
    const int choice = inputInt("请输入 1 或 2 选择控制模式：", 0, 1, 2);

    double angle_deg = 0.0, duration_sec = 0.0, target_rad = 0.0;
    if (choice == 1) {
        angle_deg   = inputDouble("请输入目标角度（度，逆时针为正，所有电机相同）：", 0.0);
        duration_sec = inputDouble("请输入完成时间（秒）：", 0.0);
        if (duration_sec <= 0.0) {
            std::printf("[错误] 完成时间必须为正数\n");
            return -1;
        }
        target_rad = angle_deg * taihu::kDegToRad;
        std::printf("[信息] 位置控制：所有电机将同时运动到 %.3f rad，时长 %.1f 秒（五次多项式轨迹）\n",
                    target_rad, duration_sec);
    } else {
        std::printf("[信息] 正弦轨迹测试：幅度 %.2f rad，频率 %.2f Hz，时长 %.1f 秒\n",
                    kAmplitude, kFrequency, kTotalDurationSec);
    }

    // 4. 每条总线启动一个控制线程
    std::vector<std::thread> threads;
    threads.reserve(buses.size());
    for (auto& bus : buses)
        threads.emplace_back(busThread, std::ref(*bus), choice, target_rad, duration_sec);

    // 5. 等待所有总线完成准备（或任一失败）
    const int total = static_cast<int>(buses.size());
    while (g_ready_count.load() < total && !g_failed.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (g_ready_count.load() == total) {
        // 全部就绪：设定统一起跑时刻并放行，各线程自旋等待到该时刻后同时开始发指令
        g_start_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(kLeadTimeMs);
        g_go = true;
        std::printf("[信息] 所有总线就绪，%d ms 后同步开始运动\n", kLeadTimeMs);
    } else {
        g_failed = true; // 通知已就绪的线程退出
    }

    for (auto& t : threads)
        t.join();

    // 6. 汇总结果
    bool all_ok = true;
    std::printf("----------------------------------------\n");
    for (auto& bus : buses) {
        if (!bus->err.empty()) {
            std::printf("[%s][错误] %s\n", bus->ifname.c_str(), bus->err.c_str());
            all_ok = false;
        } else if (!bus->ok) {
            std::printf("[%s][错误] 控制执行失败\n", bus->ifname.c_str());
            all_ok = false;
        } else {
            std::printf("[%s][信息] 完成\n", bus->ifname.c_str());
        }
    }
    return all_ok ? 0 : -1;
}
