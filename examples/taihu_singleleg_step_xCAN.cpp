/**
 * @file taihu_singleleg_step_xCAN.cpp
 * @brief 多 CAN 总线单腿步态同步控制示例（步数 N 运行时输入）。
 *
 * 与 taihu_singleleg_step 的区别：
 *   - 支持多条 CAN 总线（can0~can5），每条总线挂一条腿（四个电机 ID 1~4）；
 *   - 每条总线由独立线程组控制（发送 / 接收 / 记录各一个线程）；
 *   - 所有总线通过共享「统一起跑时刻」对齐时序，多条腿的步态同时开始；
 *   - 运行时交互输入步数 N、要控制的 CAN 总线数目（1~6）与每条总线的接口名。
 *
 * 步态定义：一步 = 3 个等长阶段，每阶段 kPhaseSec 秒（三个阶段时长完全相同）：
 *   阶段 0（零位保持）：四个电机都在 0 位保持不动；
 *   阶段 1（正摆）：ID 1/4 不动，ID 2/3 从 0° 转到 +180°；
 *   阶段 2（回摆）：ID 1/4 不动，ID 2/3 从 +180° 转回 0°。
 * 相邻关键帧之间用五次多项式插值（位置/速度/加速度连续）；一步的终点姿态与
 * 起点重合，因此 N 步之间衔接连续。总时长 = kPhaseSec × 3 × N。
 *
 * 电机配置：ID 1~4，减速比 101 / 81 / 81 / 101（与 taihu_singleleg_step 一致）。
 *
 * 日志：每条总线独立写一个 CSV（覆盖模式），文件名为
 *   record/motor_log_<接口名>.csv（如 record/motor_log_can0.csv）。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
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

// —— 每条总线（每条腿）的电机配置，与 taihu_singleleg_step 一致 ——
const std::vector<uint32_t> kMotorIds        = {1, 2, 3, 4};               // 四个电机的 CAN ID
const std::vector<double>   kMotorGearRatios = {101.0, 81.0, 81.0, 101.0}; // 对应减速比（ID1/4=101，ID2/3=81）

// —— 位置环增益 ——
constexpr int32_t kPositionKp = 4000;
constexpr int32_t kPositionKd = 60;

// =====================================================================
// 步态轨迹参数
// =====================================================================

/// 单个阶段时长 (s)。三个阶段（0/1/2）时长完全相同，只改这一个变量即可整体缩放步态快慢。
/// 修改后绘图时须给 tools/plot_motor_log.py 传 --phase-sec 保持一致（步数 N 用 --num-steps）。
constexpr double kPhaseSec = 5.0;

/// 一步包含的阶段数（阶段 0 零位保持 / 阶段 1 正摆 / 阶段 2 回摆）。
constexpr int kPhasesPerStep = 3;

/// 步数 N：运行时由键盘输入（见 main），在所有总线线程启动前写入，之后只读。
std::atomic<int> g_num_steps{1};

// 一个关键帧里四个电机的目标角度（单位：度，逆时针为正，相对零位）
struct StepPose {
    double id1_deg;
    double id2_deg;
    double id3_deg;
    double id4_deg;
};

// 一步的关键帧序列：kPhasesPerStep + 1 个关键帧 = kPhasesPerStep 个子段，
// 第 i 个阶段在 kStepKeyframes[i] → kStepKeyframes[i + 1] 之间做五次多项式插值，
// 时长均为 kPhaseSec。多步之间循环复用这组关键帧（首尾姿态相同，故衔接连续）。
const std::vector<StepPose> kStepKeyframes = {
    {0.0,   0.0,   0.0, 0.0},  ///< 阶段 0 起点：四电机全在零位
    {0.0,   0.0,   0.0, 0.0},  ///< 阶段 0 终点 = 阶段 1 起点：四电机零位保持不动
    {0.0, 180.0, 180.0, 0.0},  ///< 阶段 1 终点 = 阶段 2 起点：ID1/4 不动，ID2/3 到 +180°
    {0.0,   0.0,   0.0, 0.0},  ///< 阶段 2 终点：ID1/4 不动，ID2/3 回到零位（= 下一步起点）
};

/// 总运行时长 (s) = 阶段时长 × 每步阶段数 × 步数 N。
double totalSec() {
    return kPhaseSec * kPhasesPerStep * static_cast<double>(g_num_steps.load());
}

// 接收队列容量上限（防止记录跟不上时无限增长）
constexpr size_t kMaxQueueSize = 1000;

// —— 多总线同步参数（与 taihu_single_motorctl_xCAN 一致） ——
constexpr int kMaxBusCount = 6;   ///< 鲲弘设备最多 6 路通道（can0~can5）
constexpr int kLeadTimeMs  = 300; ///< 全部就绪后到统一起跑时刻的提前量 (ms)

/// 五次多项式平滑函数（quintic smoothstep）。
/// 满足 s(0)=0、s(1)=1，且一阶、二阶导数在两端均为 0，
/// 用于相邻关键帧之间的位置插值，保证位置、速度、加速度连续，轨迹更圆滑。
double quinticSmoothstep(double t) {
    return 6.0 * t * t * t * t * t - 15.0 * t * t * t * t + 10.0 * t * t * t;
}

/// 根据相对时间 t_rel（0 ~ totalSec()）计算四个电机的目标角度（度）。
/// 先定位全局阶段号，取模得到「步内阶段号」（0/1/2），再在该阶段内做五次多项式插值。
StepPose computeTarget(double t_rel) {
    if (t_rel < 0.0) return kStepKeyframes.front();
    if (t_rel >= totalSec()) return kStepKeyframes.back();

    const int phase_global  = static_cast<int>(t_rel / kPhaseSec);   // 0 ~ kPhasesPerStep*N-1
    const int phase_in_step = phase_global % kPhasesPerStep;         // 步内阶段号 0/1/2

    double t_phase = t_rel - phase_global * kPhaseSec;               // 阶段内时间
    if (t_phase >= kPhaseSec) t_phase = kPhaseSec - 1e-9;            // 数值保护
    const double s = quinticSmoothstep(t_phase / kPhaseSec);         // 阶段内归一化 + 平滑

    const StepPose& a = kStepKeyframes[phase_in_step];
    const StepPose& b = kStepKeyframes[phase_in_step + 1];

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

/// 一条 CAN 总线（一条腿）的配置、电机与运行时状态。
struct BusCtx {
    std::string ifname;                                  ///< 接口名，如 "can0"
    taihu::SocketCanInterface can;                       ///< 该总线独立的 SocketCAN 实例
    std::vector<std::unique_ptr<taihu::JointModule>> motors; ///< 四个电机（ID 1~4）
    std::unique_ptr<taihu::MotorLogger> logger;          ///< 该总线独立的日志记录器

    std::queue<Sample> queue;                            ///< 接收 -> 记录 的有界队列
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::atomic<bool> running{true};                     ///< 该总线线程组运行标志

    bool ok = false;                                     ///< 该总线执行结果
    std::string err;                                     ///< 失败原因
};

// —— 多线程同步用的共享状态 ——
std::atomic<int>  g_ready_count{0};   ///< 已完成准备的总线数
std::atomic<bool> g_failed{false};    ///< 任一总线准备失败
std::atomic<bool> g_go{false};        ///< 主线程放行信号
std::chrono::steady_clock::time_point g_start_time; ///< 统一起跑时刻

// —— 步间确认（按回车开始下一步）的共享状态 ——
std::atomic<int> g_step_done_count{0};   ///< 已完成步的总线数（单调累计：第 k 步后为 总线数×k）
std::atomic<int> g_step_release_gen{0};  ///< 步放行代数：主线程每收到一次回车 +1（放行第 k+1 步后为 k）
std::chrono::steady_clock::time_point g_step_release_time; ///< 最近一次放行时刻（各总线共享，保证同步开跑）

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
// 总线线程组：打开接口 -> 初始化电机与日志 -> 等待统一起跑 -> 步态控制
// =====================================================================
void busThread(BusCtx& bus) {
    using namespace taihu;

    // 1. 打开该总线的 CAN 接口
    if (!bus.can.open(bus.ifname.c_str(), 0)) {
        char buf[256];
        std::snprintf(buf, sizeof buf,
                      "打开 %s 失败，请确认已接上鲲弘 CANFD 模块且接口已 up（sudo ./build/kcanctl up %s 1000000）",
                      bus.ifname.c_str(), bus.ifname.c_str());
        bus.err = buf;
        bus.ok = false;
        g_failed = true;
        return;
    }

    // 2. 初始化该总线上的四个电机（与 taihu_singleleg_step 一致）
    for (size_t i = 0; i < kMotorIds.size(); ++i) {
        MotorConfig cfg{"motor_id" + std::to_string(kMotorIds[i]),
                        kMotorIds[i], kMotorGearRatios[i], kPositionKp, kPositionKd};
        auto m = initMotor(bus.can, cfg);
        m->clearError();
        bus.motors.push_back(std::move(m));
    }

    // 3. 打开该总线独立的日志文件：record/motor_log_<接口名>.csv
    const std::string log_path = "record/motor_log_" + bus.ifname + ".csv";
    bus.logger = std::make_unique<MotorLogger>(log_path);
    if (!bus.logger->isOpen()) {
        bus.err = "打开日志文件失败: " + log_path +
                  "（可能由 root 创建导致无写权限，或 record/ 目录不存在）";
        bus.ok = false;
        bus.can.close();
        g_failed = true;
        return;
    }
    bus.logger->writeHeader(kMotorIds);

    // 4. 报告就绪，等待主线程放行 + 统一起跑时刻
    g_ready_count++;
    while (!g_go.load() && !g_failed.load())
        std::this_thread::yield();
    if (g_failed.load()) {
        bus.logger->close();
        bus.can.close();
        return;
    }
    while (std::chrono::steady_clock::now() < g_start_time)
        std::this_thread::yield();

    // 5. 接收线程：读四个电机状态，组合成 Sample 放入队列（带节流与容量上限）
    std::thread recv_thread([&]() {
        uint64_t sample_index = 0;
        while (bus.running.load()) {
            std::vector<MotorStatus> statuses;
            statuses.reserve(kMotorIds.size());
            bool all_ok = true;
            for (size_t i = 0; i < kMotorIds.size(); ++i) {
                MotorStatus st;
                if (readMotorStatus(bus.can, kMotorIds[i], kMotorGearRatios[i], st)) {
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
                    std::lock_guard<std::mutex> lock(bus.queue_mutex);
                    if (bus.queue.size() >= kMaxQueueSize) {
                        bus.queue.pop();   // 队列已满，丢弃最旧采样，避免无限增长
                    }
                    bus.queue.push(std::move(s));
                }
                bus.queue_cv.notify_one();
                ++sample_index;
            }

            // 节流：按采样周期（控制周期 × 5）读取，保证时间戳均匀
            std::this_thread::sleep_for(std::chrono::milliseconds(kLogSamplePeriodMs));
        }
    });

    // 6. 记录线程：从队列取数据写日志
    std::thread log_thread([&]() {
        while (true) {
            Sample s;
            {
                std::unique_lock<std::mutex> lock(bus.queue_mutex);
                bus.queue_cv.wait(lock, [&] { return !bus.queue.empty() || !bus.running.load(); });
                if (bus.queue.empty() && !bus.running.load()) break;
                s = std::move(bus.queue.front());
                bus.queue.pop();
            }
            bus.logger->log(s.timestamp_s, s.motors);
        }
    });

    // 7. 发送线程逻辑（当前线程）：按单腿步态轨迹发目标位置。
    //    每一步（3 个阶段）走完后暂停，等待主线程收到回车再同步开始下一步。
    //    实现方式：以「本步起始时刻 step_time」为时间原点执行本步轨迹；
    //    某条总线先走完本步后自旋等待，直到主线程放行（刷新 g_step_release_time）。
    const int total_steps = g_num_steps.load();
    std::chrono::steady_clock::time_point step_time = g_start_time;  // 当前步的起始时刻
    for (int step = 1; step <= total_steps; ++step) {
        const double step_sec = kPhaseSec * kPhasesPerStep;   // 本步时长

        // 7.1 执行本步轨迹（从 step_time 起算 kPhaseSec×3 秒）
        while (true) {
            const double t = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - step_time).count();
            if (t >= step_sec) break;

            const StepPose pose = computeTarget(t);   // 本步内的相对时间（连续 N 步与原版一致）

            bus.motors[0]->setTargetPosition(pose.id1_deg * kDegToRad);
            bus.motors[1]->setTargetPosition(pose.id2_deg * kDegToRad);
            bus.motors[2]->setTargetPosition(pose.id3_deg * kDegToRad);
            bus.motors[3]->setTargetPosition(pose.id4_deg * kDegToRad);

            std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
        }

        // 7.2 本步完成：报告并等待主线程收到回车后放行
        g_step_done_count++;    // 各总线对「完成第 step 步」的计数（主线程按 总线数×step 判断）
        if (step < total_steps) {
            std::printf("[%s][信息] 第 %d/%d 步完成，等待回车开始下一步...\n",
                        bus.ifname.c_str(), step, total_steps);
            const int release_gen = g_step_release_gen.load();
            while (g_step_release_gen.load() == release_gen && !g_failed.load())
                std::this_thread::yield();   // 自旋等主线程放行
            if (g_failed.load()) {
                bus.running.store(false);
                bus.queue_cv.notify_all();
                recv_thread.join();
                log_thread.join();
                bus.logger->close();
                bus.can.close();
                return;
            }
            step_time = g_step_release_time;   // 以主线程放行时刻为新一步的时间原点
        } else {
            std::printf("[%s][信息] 第 %d/%d 步完成，全部步数走完\n",
                        bus.ifname.c_str(), step, total_steps);
        }
    }

    // 8. 步态结束后，停止该总线全部电机
    for (auto& m : bus.motors) m->stop();

    // 9. 停止接收与记录线程
    bus.running.store(false);
    bus.queue_cv.notify_all();
    recv_thread.join();
    log_thread.join();

    // 10. 清理
    bus.logger->close();
    bus.can.close();
    std::printf("[%s][信息] 单腿步态控制结束\n", bus.ifname.c_str());
    bus.ok = true;
}

} // namespace

int main() {
    std::printf("========================================\n");
    std::printf("  多 CAN 总线单腿步态同步控制 demo (xCAN)\n");
    std::printf("========================================\n");
    std::printf("  一步 = 3 个等长阶段：\n");
    std::printf("    阶段 0：四电机在 0 位保持不动\n");
    std::printf("    阶段 1：ID1/4 不动，ID2/3 从 0° 转到 +180°\n");
    std::printf("    阶段 2：ID1/4 不动，ID2/3 从 +180° 转回 0°\n");
    std::printf("  每阶段时长 kPhaseSec = %.1f s（改本文件开头的 kPhaseSec 即可）\n", kPhaseSec);
    std::printf("  每条总线挂一条腿：电机 ID 1~4（减速比 101/81/81/101）\n");
    std::printf("----------------------------------------\n");

    // 1. 输入步数 N（一步 = 3 个等长阶段）
    const int num_steps = inputInt("请输入步数 N（1~1000，默认 1）：", 1, 1, 1000);
    g_num_steps.store(num_steps);
    std::printf("[信息] N = %d 步，共 %d 个阶段，总时长 %.1f s\n",
                num_steps, num_steps * kPhasesPerStep, totalSec());

    // 2. 输入要控制的 CAN 总线数量
    const int bus_count = inputInt(
        "请输入要控制的 CAN 总线数量（1~6，对应 can0~can5，默认 1）：", 1, 1, kMaxBusCount);

    // 3. 逐条总线输入接口名（默认 can0、can1、...）
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

        buses.push_back(std::move(bus));
    }

    std::printf("[信息] 共 %d 条总线参与同步步态，日志写入 record/motor_log_<接口名>.csv\n",
                bus_count);

    // 4. 每条总线启动一个控制线程组
    std::vector<std::thread> threads;
    threads.reserve(buses.size());
    for (auto& bus : buses)
        threads.emplace_back(busThread, std::ref(*bus));

    // 5. 等待所有总线完成准备（或任一失败）
    const int total = static_cast<int>(buses.size());
    while (g_ready_count.load() < total && !g_failed.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (g_ready_count.load() == total) {
        // 全部就绪：设定统一起跑时刻并放行，各线程自旋等待到该时刻后同时开始发指令
        g_start_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(kLeadTimeMs);
        g_go = true;
        std::printf("[信息] 所有总线就绪，%d ms 后同步开始步态\n", kLeadTimeMs);
    } else {
        g_failed = true; // 通知已就绪的线程退出
    }

    // 5.4 吃掉交互输入阶段 scanf 留在缓冲区里的换行（只做一次，
    //     否则会与后面每次回车判定混淆）
    int drain;
    while ((drain = std::getchar()) != '\n' && drain != EOF) {}

    // 5.5 步间确认：所有总线走完同一步后，等用户敲回车再放行下一步。
    //     （回车只在这里由主线程读取；各总线线程通过 g_step_release_gen 感知放行。）
    for (int step = 1; step < num_steps; ++step) {
        // 等待所有总线完成第 step 步（计数达到 总线数 × step）或任一失败
        while (g_step_done_count.load() < total * step && !g_failed.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (g_failed.load()) break;

        // 等一次新的回车（敲其它键也会被忽略，直到回车）
        std::printf(">>> 第 %d/%d 步已全部完成，按回车开始下一步 <<<\n", step, num_steps);
        int c;
        do { c = std::getchar(); } while (c != '\n' && c != EOF);

        // 放行：刷新放行时刻并递增代数，所有总线同步开始第 step+1 步
        g_step_release_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(kLeadTimeMs);
        g_step_release_gen++;
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
            std::printf("[%s][错误] 步态执行失败\n", bus->ifname.c_str());
            all_ok = false;
        } else {
            std::printf("[%s][信息] 完成\n", bus->ifname.c_str());
        }
    }
    return all_ok ? 0 : -1;
}
