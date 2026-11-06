/**
 * @file taihu_singleleg_step_xCAN.cpp
 * @brief 多 CAN 总线单腿步态同步控制示例（步数 N 运行时输入）。
 *
 * 与 taihu_singleleg_step 的区别：
 *   - 支持多条 CAN 总线（can0~can5），每条总线挂一条腿（四个电机 ID 1~4）；
 *   - 每条总线由独立线程组控制（发送 / 接收 / 记录各一个线程）；
 *   - 所有总线通过共享「统一起跑时刻」对齐时序，多条腿的步态同时开始；
 *   - **每一步开始之前（阶段 0 之前）停顿**，等用户按回车，所有总线才同步开始
 *     这一步；一步之内的阶段 0 → 1 → 2 连续执行，中途不再停顿；
 *   - 运行时交互输入步数 N、要控制的 CAN 总线数目（1~6）与每条总线的接口名。
 *
 * 步态定义：一步 = 3 个阶段：
 *   阶段 0（零位保持）：四个电机都在 0 位保持不动；
 *   阶段 1（正摆）：ID 1/4 不动，ID 2/3 从 0° 转到 +180°；
 *   阶段 2（回摆）：ID 1/4 不动，ID 2/3 从 +180° 转回 0°。
 * 一步的终点姿态与起点重合，因此 N 步之间衔接连续。
 *
 * 阶段推进方式（由 kUsePdPointToPoint 选择，二选一）：
 *   ① 纯 PD 点到点（默认，kUsePdPointToPoint = true）
 *      每个阶段直接把「阶段终点姿态」作为目标位置下发（0x1E），不做任何时间
 *      插值，完全交给电机内部位置环（KP/KD）以自己当前能力把腿从一个点搬到
 *      另一个点。**阶段何时结束由「实测到位」决定，而不是由时钟决定**：接收
 *      线程比对四个电机的实测位置与本阶段目标，全部落在 kSettleErrorRad 内并
 *      连续保持 kSettleHoldSamples 个采样周期，发送线程才推进到下一阶段；超过
 *      kSettleTimeoutMs 仍未到位则打印警告并强制推进（避免永久卡死）。
 *      这样每阶段的实际时长是自适应的——电机跑得快就短，跑得慢就长，永远不会
 *      出现「轨迹没走完就切下一阶段」的参考跳变。
 *   ② 定时 + 五次多项式（原版行为，kUsePdPointToPoint = false）
 *      每阶段固定 kPhaseSec 秒，相邻关键帧之间用五次多项式插值（位置/速度/
 *      加速度连续），到点即切换。注意：kPhaseSec 取得较短（如 1.0 s）时，
 *      五次多项式要求的峰值速度为 1.875·Δ/T、峰值加速度为 5.77·Δ/T²
 *      （Δ=180°、T=1.0 s 时约 5.9 rad/s / 18 rad/s²，折算到电机端约 4500 rpm），
 *      电机会进入速度/电流饱和而严重滞后，实际轨迹走不完就被切到下一阶段。
 *      要跑快节奏请用方式 ①。
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

namespace
{

    // —— 每条总线（每条腿）的电机配置，与 taihu_singleleg_step 一致 ——
    const std::vector<uint32_t> kMotorIds = {1, 2, 3, 4};                    // 四个电机的 CAN ID
    const std::vector<double> kMotorGearRatios = {101.0, 81.0, 81.0, 101.0}; // 对应减速比（ID1/4=101，ID2/3=81）

    // —— 位置环增益 ——
    constexpr int32_t kPositionKp = 6000;
    constexpr int32_t kPositionKd = 60;

    // =====================================================================
    // 步态轨迹参数
    // =====================================================================

    /// 轨迹生成方式：true = 纯 PD 点到点（到位判定推进，推荐）；
    ///              false = 定时 + 五次多项式插值（原版行为，仅慢节奏适用）。
    constexpr bool kUsePdPointToPoint = true;

    /// 单个阶段时长 (s)，控制步态节奏的**唯一**时间参数。
    /// - kUsePdPointToPoint = false（定时模式）：即每阶段的实际时长，到点强制切换；
    /// - kUsePdPointToPoint = true （纯 PD 模式）：作为每阶段的**最短停留时间**——
    ///   即使腿已经到位也至少停留这么久（保证阶段 0 的零位保持不被塌缩，多总线之间
    ///   节奏一致）；超过该时间后由「到位判定」决定何时推进，最多等到 kSettleTimeoutMs。
    ///   因此实际阶段时长 = max(kPhaseSec, 到位所需时间)，把它调小（如 1.0）即可加快
    ///   步态，且绝不会出现「轨迹没走完就切下一阶段」。
    /// 修改后绘图时须给 tools/plot_motor_log.py 传 --phase-sec 保持一致（步数 N 用 --num-steps）。
    constexpr double kPhaseSec = 10.0;

    /// 一步包含的阶段数（阶段 0 零位保持 / 阶段 1 正摆 / 阶段 2 回摆）。
    constexpr int kPhasesPerStep = 3;

    // —— 纯 PD 点到点模式的到位判定参数 ——

    /// 到位判定位置误差阈值 (rad，输出端)。0.035 rad ≈ 2.0°。
    /// 阈值太小会因编码器量化/静摩擦导致永远判不到位（只能靠超时强制推进）；
    /// 太大则腿还没走到位就切阶段。默认取 2°，比全局 kSettleThresholdRad(≈1.1°) 宽松。
    constexpr double kSettleErrorRad = 0.035;

    /// 到位判定需要连续满足阈值的采样周期数（接收线程按 kLogSamplePeriodMs=100ms 采样）。
    /// 3 个采样 = 0.3 s，用于滤除「路过目标点」的瞬时满足（速度还很大时不算到位）。
    constexpr int kSettleHoldSamples = 3;

    /// 到位超时 (ms)。超过该时间仍未判到位则强制推进到下一阶段并打印警告
    /// （含当时的最大位置误差，便于判断是阈值太严还是电机确实跑不到）。
    /// 默认 15000 ms：给电机足够时间跑完 180°，同时避免卡死后无法退出。
    constexpr int kSettleTimeoutMs = 15000;

    /// 纯 PD 点到点的参考斜坡时间 (s)，0 表示直接阶跃下发（最纯粹）。
    /// 阶跃瞬间位置误差最大（可达 π rad），电机内部位置环会一步输出最大转矩，
    /// 存在电流冲击与过冲/振荡风险。若实机上觉得太猛，把它改成 0.1~0.2，
    /// 参考会在这么长时间内从起点线性爬到终点（仍然只靠 PD 跟随，不做五次整形）。
    constexpr double kPdRampSec = 0.2;

    /// 步数 N：运行时由键盘输入（见 main），在所有总线线程启动前写入，之后只读。
    std::atomic<int> g_num_steps{1};

    // 一个关键帧里四个电机的目标角度（单位：度，逆时针为正，相对零位）
    struct StepPose
    {
        double id1_deg;
        double id2_deg;
        double id3_deg;
        double id4_deg;
    };

    // 一步的关键帧序列：kPhasesPerStep + 1 个关键帧 = kPhasesPerStep 个子段。
    // - 定时模式：第 i 个阶段在 kStepKeyframes[i] → kStepKeyframes[i + 1] 之间做
    //   五次多项式插值，时长均为 kPhaseSec；
    // - 纯 PD 模式：第 i 个阶段直接把 kStepKeyframes[i + 1] 作为目标下发。
    // 多步之间循环复用这组关键帧（首尾姿态相同，故衔接连续）。
    const std::vector<StepPose> kStepKeyframes = {
        {0.0, 0.0, 0.0, 0.0},     ///< 阶段 0 起点：四电机全在零位
        {0.0, 0.0, 0.0, 0.0},     ///< 阶段 0 终点 = 阶段 1 起点：四电机零位保持不动
        {0.0, 180.0, 180.0, 0.0}, ///< 阶段 1 终点 = 阶段 2 起点：ID1/4 不动，ID2/3 到 +180°
        {0.0, 0.0, 0.0, 0.0},     ///< 阶段 2 终点：ID1/4 不动，ID2/3 回到零位（= 下一步起点）
    };

    /// 总运行时长 (s) = 阶段时长 × 每步阶段数 × 步数 N（名义值，仅定时模式严格成立）。
    double totalSec()
    {
        return kPhaseSec * kPhasesPerStep * static_cast<double>(g_num_steps.load());
    }

    // 接收队列容量上限（防止记录跟不上时无限增长）
    constexpr size_t kMaxQueueSize = 1000;

    // —— 多总线同步参数（与 taihu_single_motorctl_xCAN 一致） ——
    constexpr int kMaxBusCount = 6;  ///< 鲲弘设备最多 6 路通道（can0~can5）
    constexpr int kLeadTimeMs = 300; ///< 全部就绪后到统一起跑时刻的提前量 (ms)

    /// 五次多项式平滑函数（quintic smoothstep）。
    /// 满足 s(0)=0、s(1)=1，且一阶、二阶导数在两端均为 0，
    /// 用于相邻关键帧之间的位置插值，保证位置、速度、加速度连续，轨迹更圆滑。
    /// 仅在定时模式（kUsePdPointToPoint = false）下使用。
    double quinticSmoothstep(double t)
    {
        return 6.0 * t * t * t * t * t - 15.0 * t * t * t * t + 10.0 * t * t * t;
    }

    /// 按电机下标（0~3，对应 ID 1~4）取关键帧中的目标角度（度）。
    /// 把 StepPose 的四个字段统一成数组访问，便于用循环处理。
    double poseDeg(const StepPose &p, size_t idx)
    {
        switch (idx)
        {
        case 0:
            return p.id1_deg;
        case 1:
            return p.id2_deg;
        case 2:
            return p.id3_deg;
        default:
            return p.id4_deg;
        }
    }

    /// 根据相对时间 t_rel（0 ~ totalSec()）计算四个电机的目标角度（度）。
    /// 先定位全局阶段号，取模得到「步内阶段号」（0/1/2），再在该阶段内做五次多项式插值。
    /// **仅定时模式使用**；纯 PD 模式下阶段推进由到位判定驱动，时间轴不再唯一确定，
    /// 因此不经过本函数（直接取关键帧终点，见 busThread 中的 PD 分支）。
    StepPose computeTarget(double t_rel)
    {
        if (t_rel < 0.0)
            return kStepKeyframes.front();
        if (t_rel >= totalSec())
            return kStepKeyframes.back();

        const int phase_global = static_cast<int>(t_rel / kPhaseSec); // 0 ~ kPhasesPerStep*N-1
        const int phase_in_step = phase_global % kPhasesPerStep;      // 步内阶段号 0/1/2

        double t_phase = t_rel - phase_global * kPhaseSec; // 阶段内时间
        if (t_phase >= kPhaseSec)
            t_phase = kPhaseSec - 1e-9;                          // 数值保护
        const double s = quinticSmoothstep(t_phase / kPhaseSec); // 阶段内归一化 + 平滑

        const StepPose &a = kStepKeyframes[phase_in_step];
        const StepPose &b = kStepKeyframes[phase_in_step + 1];

        StepPose p;
        p.id1_deg = a.id1_deg + (b.id1_deg - a.id1_deg) * s;
        p.id2_deg = a.id2_deg + (b.id2_deg - a.id2_deg) * s;
        p.id3_deg = a.id3_deg + (b.id3_deg - a.id3_deg) * s;
        p.id4_deg = a.id4_deg + (b.id4_deg - a.id4_deg) * s;
        return p;
    }

    // 一个采样点：时间戳 + 各电机状态
    struct Sample
    {
        double timestamp_s = 0.0;
        std::vector<taihu::MotorStatus> motors;
    };

    /// 一条 CAN 总线（一条腿）的配置、电机与运行时状态。
    struct BusCtx
    {
        std::string ifname;                                      ///< 接口名，如 "can0"
        taihu::SocketCanInterface can;                           ///< 该总线独立的 SocketCAN 实例
        std::vector<std::unique_ptr<taihu::JointModule>> motors; ///< 四个电机（ID 1~4）
        std::unique_ptr<taihu::MotorLogger> logger;              ///< 该总线独立的日志记录器

        std::queue<Sample> queue; ///< 接收 -> 记录 的有界队列
        std::mutex queue_mutex;
        std::condition_variable queue_cv;
        std::atomic<bool> running{true}; ///< 该总线线程组运行标志

        // —— 纯 PD 点到点模式的到位判定（发送线程发布目标、接收线程判定到位）——
        // 用「单调递增且跨步不重复的阶段序号」做握手标记：接收线程只在判定通过时
        // 把 settled_phase_seq 写成它当时评估的 seq，发送线程比较 seq 是否相等。
        // 这样过期写入携带的是更小的 seq，永远不会被误认成当前阶段已到位（无锁无竞态）。
        std::atomic<int> cur_phase_seq{-1};        ///< 当前阶段序号（-1 = 不在阶段中）
        std::atomic<int> settled_phase_seq{-1};    ///< 已判定到位的阶段序号
        std::atomic<double> last_max_err_rad{0.0}; ///< 最近一次采样的四电机最大位置误差 (rad)，用于超时诊断

        bool ok = false; ///< 该总线执行结果
        std::string err; ///< 失败原因
    };

    // —— 多线程同步用的共享状态 ——
    std::atomic<int> g_ready_count{0};                  ///< 已完成准备的总线数
    std::atomic<bool> g_failed{false};                  ///< 任一总线准备失败
    std::atomic<bool> g_go{false};                      ///< 主线程放行信号
    std::chrono::steady_clock::time_point g_start_time; ///< 统一起跑时刻

    // —— 每步开始前的回车确认（阶段 0 之前）的共享状态 ——
    std::atomic<int> g_step_ready_count{0};                    ///< 已就绪等待开始某一步的总线数（单调累计：第 k 步为 总线数×k）
    std::atomic<int> g_step_release_gen{0};                    ///< 放行代数：主线程每收到一次回车 +1（放行第 k 步后为 k）
    std::chrono::steady_clock::time_point g_step_release_time; ///< 最近一次放行时刻（各总线共享，保证同步开跑）

    /// 输入一个正整数，带默认值与范围检查。
    int inputInt(const char *prompt, int def, int lo, int hi)
    {
        std::printf("%s", prompt);
        int v = def;
        if (std::scanf("%d", &v) != 1 || v < lo || v > hi)
        {
            std::printf("[提示] 输入无效，使用默认值 %d\n", def);
            return def;
        }
        return v;
    }

    /// 归一化接口名输入：纯数字（0~5）自动补全为 canN，其余原样返回。
    /// 例如输入 "0" -> "can0"，输入 "can1" -> "can1"。
    std::string normalizeIfname(const char *raw)
    {
        char *end = nullptr;
        const long v = std::strtol(raw, &end, 10);
        if (end != raw && *end == '\0' && v >= 0 && v < kMaxBusCount)
        {
            char buf[16];
            std::snprintf(buf, sizeof buf, "can%ld", v);
            return buf;
        }
        return raw;
    }

    // =====================================================================
    // 总线线程组：打开接口 -> 初始化电机与日志 -> 等待统一起跑 -> 步态控制
    // =====================================================================
    void busThread(BusCtx &bus)
    {
        using namespace taihu;

        // 1. 打开该总线的 CAN 接口
        if (!bus.can.open(bus.ifname.c_str(), 0))
        {
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
        for (size_t i = 0; i < kMotorIds.size(); ++i)
        {
            MotorConfig cfg{"motor_id" + std::to_string(kMotorIds[i]),
                            kMotorIds[i], kMotorGearRatios[i], kPositionKp, kPositionKd};
            auto m = initMotor(bus.can, cfg);
            m->clearError();
            bus.motors.push_back(std::move(m));
        }

        // 3. 打开该总线独立的日志文件：record/motor_log_<接口名>.csv
        const std::string log_path = "record/motor_log_" + bus.ifname + ".csv";
        bus.logger = std::make_unique<MotorLogger>(log_path);
        if (!bus.logger->isOpen())
        {
            bus.err = "打开日志文件失败: " + log_path +
                      "（可能由 root 创建导致无写权限，或 record/ 目录不存在）";
            bus.ok = false;
            bus.can.close();
            g_failed = true;
            return;
        }
        bus.logger->writeHeader(kMotorIds);

        // 4. 报告就绪，等主线程放行（放行后并不立即运动——每一步开始前还有回车确认，
        //    见 7.0；统一起跑时刻只用来对齐各总线线程的启动）
        g_ready_count++;
        while (!g_go.load() && !g_failed.load())
            std::this_thread::yield();
        if (g_failed.load())
        {
            bus.logger->close();
            bus.can.close();
            return;
        }
        while (std::chrono::steady_clock::now() < g_start_time)
            std::this_thread::yield();

        // 5. 接收线程：读四个电机状态，组合成 Sample 放入队列（带节流与容量上限）；
        //    纯 PD 模式下同时负责「到位判定」——比对实测位置与当前阶段目标关键帧。
        std::thread recv_thread([&]()
                                {
        uint64_t sample_index = 0;
        int hold_count = 0;          // 连续满足阈值的采样次数（到位去抖）
        int last_seq = -1;           // 上一次判定时针对的阶段序号（切换阶段时清零计数）
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
                // 5.1 到位判定：四个电机实测位置都落在当前阶段目标 ±kSettleErrorRad 内，
                //     且连续保持 kSettleHoldSamples 个采样周期，才把 settled_phase_seq
                //     写成当前 seq（发送线程据此推进）。目标关键帧由 seq 推导：
                //     步内阶段号 = seq % kPhasesPerStep，其终点关键帧下标 = 步内阶段号 + 1。
                const int seq = bus.cur_phase_seq.load();
                if (seq >= 0) {
                    if (seq != last_seq) {   // 阶段切换：复位去抖计数
                        hold_count = 0;
                        last_seq = seq;
                    }
                    const StepPose& tgt = kStepKeyframes[(seq % kPhasesPerStep) + 1];

                    double max_err = 0.0;
                    for (size_t i = 0; i < statuses.size(); ++i) {
                        const double target_rad = poseDeg(tgt, i) * kDegToRad;
                        const double err = std::fabs(statuses[i].position_rad - target_rad);
                        if (err > max_err) max_err = err;
                    }
                    bus.last_max_err_rad.store(max_err);   // 供超时诊断打印

                    if (max_err <= kSettleErrorRad) {
                        if (++hold_count >= kSettleHoldSamples)
                            bus.settled_phase_seq.store(seq);   // 到位（携带 seq，发送线程比对）
                    } else {
                        hold_count = 0;
                    }
                }

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
        } });

        // 6. 记录线程：从队列取数据写日志
        std::thread log_thread([&]()
                               {
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
        } });

        // 7. 发送线程逻辑（当前线程）：按单腿步态轨迹发目标位置。
        //    停顿点：**每一步开始之前（阶段 0 之前）**——本线程先报告就绪
        //    （g_step_ready_count++），再自旋等主线程收到回车放行；一步之内
        //    阶段 0 → 1 → 2 连续执行，中途不再停顿。第 1 步同样需要回车确认。
        //    阶段推进：纯 PD 模式 = 到位判定（接收线程判定，见 5.1）；
        //              定时模式 = 固定 kPhaseSec 秒（原版行为）。
        const int total_steps = g_num_steps.load();
        std::chrono::steady_clock::time_point phase_time = g_start_time; // 当前阶段的起始时刻（定时模式用）
        int phase_seq = 0;                                               // 全局阶段序号（单调递增，跨步不重复，用于到位握手）
        for (int step = 1; step <= total_steps; ++step)
        {
            // 7.0 每一步开始前的回车确认：报告就绪后自旋等主线程放行。
            //     所有总线都就绪时主线程才提示并读取回车，因此各总线严格同步开跑。
            //     放行判据用「绝对代数」g_step_release_gen >= step（主线程放行第 k 步后
            //     代数为 k），比「捕获旧值再等变化」更稳，即使主线程抢跑也不会死锁。
            g_step_ready_count++; // 「第 step 步就绪」的总线计数（主线程按 总线数×step 判断）
            std::printf("[%s][信息] 第 %d/%d 步就绪，等待回车开始（阶段 0 零位保持）...\n",
                        bus.ifname.c_str(), step, total_steps);
            while (g_step_release_gen.load() < step && !g_failed.load())
                std::this_thread::yield(); // 自旋等主线程放行第 step 步
            if (g_failed.load())
            {
                bus.running.store(false);
                bus.queue_cv.notify_all();
                recv_thread.join();
                log_thread.join();
                bus.logger->close();
                bus.can.close();
                return;
            }
            phase_time = g_step_release_time; // 以放行时刻为本步的时间原点

            for (int phase = 0; phase < kPhasesPerStep; ++phase)
            {
                if (kUsePdPointToPoint)
                {
                    // 7.1a 纯 PD 点到点：直接把阶段终点关键帧作为目标位置下发，
                    //      不做时间插值；每阶段至少停留 kPhaseSec，之后由「实测到位」
                    //      判定推进（kSettleErrorRad 内连续 kSettleHoldSamples 个采样
                    //      周期），超时 kSettleTimeoutMs 强制推进。最短停留保证阶段 0
                    //      的零位保持不被塌缩（其目标与当前姿态相同，会立刻判到位）。
                    const StepPose &src = kStepKeyframes[phase];     // 斜坡起点（名义上=上一阶段终点）
                    const StepPose &tgt = kStepKeyframes[phase + 1]; // 本阶段终点 = PD 目标
                    const auto t0 = std::chrono::steady_clock::now();
                    bus.settled_phase_seq.store(-1);    // 清掉上一阶段的到位标记
                    bus.cur_phase_seq.store(phase_seq); // 发布当前阶段，接收线程开始判定

                    while (true)
                    {
                        const double t = std::chrono::duration<double>(
                                             std::chrono::steady_clock::now() - t0)
                                             .count();
                        if (t >= kPhaseSec)
                        {
                            if (bus.settled_phase_seq.load() == phase_seq)
                                break; // 实测到位
                            if (t * 1000.0 >= kSettleTimeoutMs)
                            {
                                std::printf("[%s][警告] 第 %d/%d 步阶段 %d 到位超时（%d ms），"
                                            "最大位置误差 %.1f°，强制推进\n",
                                            bus.ifname.c_str(), step, total_steps, phase,
                                            kSettleTimeoutMs,
                                            bus.last_max_err_rad.load() / kDegToRad);
                                break;
                            }
                        }

                        // 参考下发：kPdRampSec = 0 时直接阶跃到终点（最纯粹的 PD 点到点）；
                        // > 0 时在斜坡时间内从名义起点线性爬到终点，缓和电流冲击。
                        double a = 1.0;
                        if (kPdRampSec > 0.0 && t < kPdRampSec)
                            a = t / kPdRampSec;
                        for (size_t i = 0; i < kMotorIds.size(); ++i)
                        {
                            const double s_deg = poseDeg(src, i);
                            const double e_deg = poseDeg(tgt, i);
                            bus.motors[i]->setTargetPosition(
                                (s_deg + (e_deg - s_deg) * a) * kDegToRad);
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
                    }
                    bus.cur_phase_seq.store(-1);                   // 本阶段结束，停止到位判定
                    phase_time = std::chrono::steady_clock::now(); // PD 模式下仅作记录
                }
                else
                {
                    // 7.1b 定时 + 五次多项式（原版行为）：固定 kPhaseSec 秒，到点切换。
                    //      步内相对时间 = phase*kPhaseSec + t，关键帧循环复用。
                    while (true)
                    {
                        const double t = std::chrono::duration<double>(
                                             std::chrono::steady_clock::now() - phase_time)
                                             .count();
                        if (t >= kPhaseSec)
                            break;

                        const StepPose pose = computeTarget(phase * kPhaseSec + t);

                        bus.motors[0]->setTargetPosition(pose.id1_deg * kDegToRad);
                        bus.motors[1]->setTargetPosition(pose.id2_deg * kDegToRad);
                        bus.motors[2]->setTargetPosition(pose.id3_deg * kDegToRad);
                        bus.motors[3]->setTargetPosition(pose.id4_deg * kDegToRad);

                        std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
                    }

                    // 连续衔接下一阶段：名义时间轴推进 kPhaseSec（与原版全局时间轴一致）
                    phase_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(kPhaseSec));
                }
                ++phase_seq;
            }
        }

        // 8. 步态结束后，停止该总线全部电机
        for (auto &m : bus.motors)
            m->stop();

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

int main()
{
    std::printf("========================================\n");
    std::printf("  多 CAN 总线单腿步态同步控制 demo (xCAN)\n");
    std::printf("========================================\n");
    std::printf("  一步 = 3 个阶段：\n");
    std::printf("    阶段 0：四电机在 0 位保持不动\n");
    std::printf("    阶段 1：ID1/4 不动，ID2/3 从 0° 转到 +180°\n");
    std::printf("    阶段 2：ID1/4 不动，ID2/3 从 +180° 转回 0°\n");
    std::printf("  ★ 每一步开始之前（阶段 0 之前）需按一次回车确认，共 N 次\n");
    if (kUsePdPointToPoint)
    {
        std::printf("  轨迹方式：纯 PD 点到点（到位判定推进，每阶段时长自适应）\n");
        std::printf("    每阶段最短停留 %.1f s（kPhaseSec）；到位阈值 %.1f°，连续 %d 个采样，超时 %d ms\n",
                    kPhaseSec, kSettleErrorRad / taihu::kDegToRad, kSettleHoldSamples, kSettleTimeoutMs);
        std::printf("    参考斜坡 kPdRampSec = %.2f s（0 = 直接阶跃）\n", kPdRampSec);
    }
    else
    {
        std::printf("  轨迹方式：定时 + 五次多项式插值（每阶段固定 %.1f s）\n", kPhaseSec);
    }
    std::printf("  每条总线挂一条腿：电机 ID 1~4（减速比 101/81/81/101）\n");
    std::printf("----------------------------------------\n");

    // 1. 输入步数 N（一步 = 3 个阶段）
    const int num_steps = inputInt("请输入步数 N（1~1000，默认 1）：", 1, 1, 1000);
    g_num_steps.store(num_steps);
    std::printf("[信息] N = %d 步，共 %d 个阶段%s\n",
                num_steps, num_steps * kPhasesPerStep,
                kUsePdPointToPoint ? "（纯 PD 到位判定推进，总时长自适应）"
                                   : "，总时长由 kPhaseSec 决定");

    // 2. 输入要控制的 CAN 总线数量
    const int bus_count = inputInt(
        "请输入要控制的 CAN 总线数量（1~6，对应 can0~can5，默认 1）：", 1, 1, kMaxBusCount);

    // 3. 逐条总线输入接口名（默认 can0、can1、...）
    std::vector<std::unique_ptr<BusCtx>> buses;
    for (int i = 0; i < bus_count; i++)
    {
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
    for (auto &bus : buses)
        threads.emplace_back(busThread, std::ref(*bus));

    // 5. 等待所有总线完成准备（或任一失败）
    const int total = static_cast<int>(buses.size());
    while (g_ready_count.load() < total && !g_failed.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (g_ready_count.load() == total)
    {
        // 全部就绪：设定统一起跑时刻并放行，各总线线程同时启动（启动后不立即运动，
        // 而是停在「每一步开始前的回车确认」处，见 busThread 7.0）
        g_start_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(kLeadTimeMs);
        g_go = true;
        std::printf("[信息] 所有总线就绪，%d ms 后进入步态流程（每一步开始前需按回车确认）\n",
                    kLeadTimeMs);
    }
    else
    {
        g_failed = true; // 通知已就绪的线程退出
    }

    // 5.4 吃掉交互输入阶段 scanf 留在缓冲区里的换行（只做一次，
    //     否则会与后面每次回车判定混淆）
    int drain;
    while ((drain = std::getchar()) != '\n' && drain != EOF)
    {
    }

    // 5.5 回车确认：**每一步开始之前**（阶段 0 之前）等用户敲一次回车再放行，
    //     共 N 次停顿（每步一次）；一步之内阶段 0 → 1 → 2 连续执行。
    //     （回车只在这里由主线程读取；各总线线程通过 g_step_release_gen 感知放行。）
    for (int step = 1; step <= num_steps; ++step)
    {
        // 等待所有总线都报告「第 step 步就绪」（计数达到 总线数 × step）或任一失败
        while (g_step_ready_count.load() < total * step && !g_failed.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (g_failed.load())
            break;

        // 等一次新的回车（敲其它键会被忽略，直到回车）
        std::printf(">>> 按回车开始第 %d/%d 步（阶段 0 零位保持 → 阶段 1 正摆 → 阶段 2 回摆）<<<\n",
                    step, num_steps);
        int c;
        do
        {
            c = std::getchar();
        } while (c != '\n' && c != EOF);

        // 放行：刷新放行时刻并把代数递增到 step，所有总线同步开始本步
        g_step_release_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(kLeadTimeMs);
        g_step_release_gen.store(step);
    }

    for (auto &t : threads)
        t.join();

    // 6. 汇总结果
    bool all_ok = true;
    std::printf("----------------------------------------\n");
    for (auto &bus : buses)
    {
        if (!bus->err.empty())
        {
            std::printf("[%s][错误] %s\n", bus->ifname.c_str(), bus->err.c_str());
            all_ok = false;
        }
        else if (!bus->ok)
        {
            std::printf("[%s][错误] 步态执行失败\n", bus->ifname.c_str());
            all_ok = false;
        }
        else
        {
            std::printf("[%s][信息] 完成\n", bus->ifname.c_str());
        }
    }
    return all_ok ? 0 : -1;
}
