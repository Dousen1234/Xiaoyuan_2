/**
 * taihu_pd_leg.cpp —— 单腿 4 关节「纯 PD 位置模式」控制 demo（200Hz）
 *
 * 控制律：
 *   电机内部位置环 PD 伺服（KP/KD 经 0x2B/0x2D 下发，默认 4000/60），
 *   上位机以 200Hz（5ms）周期下发关节目标角（0x1E 位置模式），
 *   不做重力前馈（纯 PD）：悬空时存在重力下垂稳态误差，属预期现象。
 *
 * 运动模式（运行时命令）：
 *   hold            姿态保持：以当前实测姿态为目标进入 PD 伺服（解抱闸）
 *   goto a b c d [s] 交互目标姿态：五次多项式平滑过渡到 4 个关节目标角（度），
 *                   时长 s（默认 2.0s），到位后自动转为保持
 *   pos             打印当前关节角/目标角/电流
 *   gains kp kd     运行时调整电机位置环 KP/KD
 *   mon on|off      1Hz 状态行打印开关（默认 off）
 *   estop           急停：全部电机 0x02 抱闸并退出伺服
 *   help / quit     帮助 / 退出（退出自动抱闸）
 *
 * 构型（与 walk_control_V1 一致，关节端）：
 *   关节 1~4 = 电机 CAN ID 1~4；关节端总减速比 101/162/162/101；
 *   J2/J3 含 1:2 锥齿轮（kMotorRadPerJointRad = 1/2/2/1）；
 *   软限位（关节端，度）：J1 ±90、J2 ±120（电机端 ±240）、
 *                          J3 −30~+180（电机端 −60~+360）、J4 ±90。
 *   方向符号（kJointDirSign）与 taihu_singleleg_step_xCAN 一致：正角 = 逆时针（ccw）。
 *
 * 安全保护：
 *   - 目标角下发前钳位到软限位；
 *   - 实测角越限 + kLimitMarginDeg 立即急停抱闸；
 *   - 单电机电流超 kCurrentEstopMa 持续 0.5s 急停；超 kCurrentWarnMa 告警；
 *   - 电机错误位非零（每 0.5s 巡检）急停；
 *   - 连续 kReadFailEstop 次读反馈失败（掉线/总线异常）急停；
 *   - Ctrl+C / quit 退出时全部 0x02 抱闸。
 *
 * 日志：record/pd_leg_log.csv（关节端 rad / 关节端 rad/s / mA，100ms 采样）。
 *   注意 plot_motor_log.py 的步态叠加模型不适用于本 demo，仅用于查看曲线。
 *
 * 运行前置：sudo ./build/kcanctl up can0 1000000（按需逐通道开启）。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "taihu/config.h"
#include "taihu/joint_module.h"
#include "taihu/motor_logger.h"
#include "taihu/socket_can.h"
#include "taihu/taihu_tools.h"

namespace
{

    using Clock = std::chrono::steady_clock;

    // —— 构型常量（关节端） ——
    constexpr int kJointCount = 4;
    const uint32_t kMotorIds[kJointCount] = {1, 2, 3, 4};
    const double kJointGearRatios[kJointCount] = {101.0, 162.0, 162.0, 101.0};
    const double kMotorRadPerJointRad[kJointCount] = {1.0, 2.0, 2.0, 1.0};
    /// 方向符号与 taihu_singleleg_step_xCAN 一致：正角 = 逆时针（ccw）；
    /// 该 demo 关键帧 {0,180,180,0} 中 ID2/ID3 同为 +180°，P 型锥齿轮电机端同向。
    const double kJointDirSign[kJointCount] = {1.0, 1.0, 1.0, 1.0};
    /// 软件限位（关节端度）；电机端等效 = ×kMotorRadPerJointRad：
    ///   J2 ±120° ↔ ±240°；J3 −30°~+180° ↔ −60°~+360°。
    const double kJointLimitDeg[kJointCount][2] = {
        {-90.0, 90.0}, {-120.0, 120.0}, {-30.0, 180.0}, {-90.0, 90.0}};

    // —— 控制参数 ——
    constexpr int32_t kPositionKp = 4000;     ///< 电机位置环 KP（0x2B）
    constexpr int32_t kPositionKd = 60;       ///< 电机位置环 KD（0x2D）
    constexpr int kControlPeriodMs = 5;       ///< 200Hz 控制周期
    constexpr double kLimitMarginDeg = 3.0;   ///< 实测越限急停裕量（度）
    constexpr int32_t kCurrentWarnMa = 5000;  ///< 电流告警阈值 (mA)
    constexpr int32_t kCurrentEstopMa = 8000; ///< 电流急停阈值 (mA)
    constexpr int kCurrentEstopTicks = 100;   ///< 电流急停持续 tick 数（0.5s）
    constexpr int kReadFailEstop = 20;        ///< 连续读失败急停阈值
    constexpr double kDefaultGotoSec = 2.0;   ///< goto 默认过渡时长 (s)
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

    double jointToMotorRad(int j, double joint_rad)
    {
        return joint_rad * kMotorRadPerJointRad[j] * kJointDirSign[j];
    }
    double motorToJointRad(int j, double motor_rad)
    {
        return motor_rad / (kMotorRadPerJointRad[j] * kJointDirSign[j]);
    }
    double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
    /// 五次多项式平滑步：s(0)=0, s(1)=1，端点一阶/二阶导为 0
    double quinticSmoothstep(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

    // —— 命令（主线程 → 控制线程） ——
    struct Cmd
    {
        enum Type
        {
            None,
            Hold,
            Goto,
            Pos,
            Gains,
            Mon,
            Estop
        } type = None;
        double target_deg[kJointCount] = {0, 0, 0, 0};
        double sec = kDefaultGotoSec;
        int32_t kp = 0, kd = 0;
        bool mon_on = false;
    };

    std::unique_ptr<taihu::JointModule> g_motors[kJointCount];
    taihu::SocketCanInterface g_can;

    std::mutex g_mtx; ///< 保护 g_pending / g_pending_has
    Cmd g_pending;
    bool g_pending_has = false;

    std::atomic<bool> g_running{true};
    std::atomic<bool> g_estop{false};

    void onSignal(int)
    {
        g_running.store(false);
        g_estop.store(true);
    }

    void stopAll()
    {
        for (int j = 0; j < kJointCount; ++j)
            if (g_motors[j])
                g_motors[j]->stop();
    }

    void printHelp()
    {
        std::printf(
            "命令:\n"
            "  hold              以当前姿态进入 PD 保持（解抱闸）\n"
            "  goto a b c d [s]  平滑过渡到关节目标角(度)，时长 s 默认 %.1f\n"
            "  pos               打印当前关节角/目标角/电流\n"
            "  gains kp kd       调整电机位置环 KP/KD\n"
            "  mon on|off        1Hz 状态行打印开关\n"
            "  estop             急停抱闸\n"
            "  help / quit       帮助 / 退出（自动抱闸）\n",
            kDefaultGotoSec);
    }

    // =====================================================================
    // 控制线程：200Hz 读反馈 → 计算参考 → 限位/安全 → 下发目标
    // =====================================================================
    void controlThread()
    {
        const Clock::time_point t_start = Clock::now();
        double meas_deg[kJointCount] = {0, 0, 0, 0};
        double cur_ma[kJointCount] = {0, 0, 0, 0};
        bool ok[kJointCount] = {false, false, false, false};
        uint32_t err_bits[kJointCount] = {0, 0, 0, 0};

        double ref[kJointCount] = {0, 0, 0, 0};
        double start[kJointCount] = {0, 0, 0, 0};
        double end[kJointCount] = {0, 0, 0, 0};
        int mode = 0; // 0 空闲 1 保持 2 过渡
        bool enabled = false;
        bool mon_on = false;
        Clock::time_point traj_t0;
        double traj_sec = kDefaultGotoSec;

        int fail_cnt[kJointCount] = {0, 0, 0, 0};
        int over_cur[kJointCount] = {0, 0, 0, 0};
        auto last_warn = Clock::now();
        auto last_mon = Clock::now();

        int tick = 0;
        auto next = Clock::now();

        taihu::MotorLogger logger("record/pd_leg_log.csv");
        const std::vector<uint32_t> ids(kMotorIds, kMotorIds + kJointCount);
        logger.writeHeader(ids);
        auto last_log = Clock::now();

        while (g_running.load())
        {
            // —— 急停（优先级最高） ——
            if (g_estop.exchange(false))
            {
                if (enabled)
                    std::printf("[急停] 全部电机已抱闸（0x02）\n");
                stopAll();
                enabled = false;
                mode = 0;
            }

            // —— 消费命令 ——
            {
                std::lock_guard<std::mutex> lk(g_mtx);
                if (g_pending_has)
                {
                    const Cmd c = g_pending;
                    g_pending_has = false;
                    switch (c.type)
                    {
                    case Cmd::Hold:
                    {
                        for (int j = 0; j < kJointCount; ++j)
                        {
                            if (!ok[j])
                            { // 快照无效则补读一次
                                int32_t ma = 0;
                                double vel = 0.0, pos = 0.0;
                                if (g_motors[j]->readCurrentVelocityPosition(ma, vel, pos))
                                {
                                    meas_deg[j] = motorToJointRad(j, pos) * kRadToDeg;
                                    ok[j] = true;
                                }
                            }
                            start[j] = end[j] = ref[j] = meas_deg[j];
                        }
                        mode = 1;
                        enabled = true;
                        std::printf("[PD] 保持模式：目标 = 当前姿态 "
                                    "(%.1f, %.1f, %.1f, %.1f)°\n",
                                    ref[0], ref[1], ref[2], ref[3]);
                        break;
                    }
                    case Cmd::Goto:
                    {
                        for (int j = 0; j < kJointCount; ++j)
                        {
                            end[j] = clampd(c.target_deg[j],
                                            kJointLimitDeg[j][0], kJointLimitDeg[j][1]);
                            if (end[j] != c.target_deg[j])
                                std::printf("[限位] J%d 目标 %.1f° 钳位到 %.1f°\n",
                                            j + 1, c.target_deg[j], end[j]);
                            if (mode == 0 && !ok[j])
                            {
                                // 首次 goto：起点用实测位置，避免从 0° 猛冲
                                int32_t ma = 0;
                                double vel = 0.0, pos = 0.0;
                                if (g_motors[j]->readCurrentVelocityPosition(ma, vel, pos))
                                {
                                    meas_deg[j] = motorToJointRad(j, pos) * kRadToDeg;
                                    ok[j] = true;
                                }
                            }
                            start[j] = (mode != 0) ? ref[j] : meas_deg[j];
                            ref[j] = start[j];
                        }
                        traj_t0 = Clock::now();
                        traj_sec = c.sec;
                        mode = 2;
                        enabled = true;
                        std::printf("[PD] 过渡模式：→ (%.1f, %.1f, %.1f, %.1f)° / %.2fs\n",
                                    end[0], end[1], end[2], end[3], traj_sec);
                        break;
                    }
                    case Cmd::Pos:
                    {
                        std::printf("[状态] %s  关节角(°): %7.2f %7.2f %7.2f %7.2f | "
                                    "目标(°): %7.2f %7.2f %7.2f %7.2f | 电流(A): "
                                    "%5.2f %5.2f %5.2f %5.2f\n",
                                    enabled ? (mode == 2 ? "过渡中" : "保持中") : "未使能",
                                    meas_deg[0], meas_deg[1], meas_deg[2], meas_deg[3],
                                    ref[0], ref[1], ref[2], ref[3],
                                    cur_ma[0] / 1000.0, cur_ma[1] / 1000.0,
                                    cur_ma[2] / 1000.0, cur_ma[3] / 1000.0);
                        break;
                    }
                    case Cmd::Gains:
                    {
                        for (int j = 0; j < kJointCount; ++j)
                        {
                            g_motors[j]->setPositionKp(c.kp);
                            g_motors[j]->setPositionKd(c.kd);
                        }
                        std::printf("[增益] 位置环 KP=%d KD=%d 已下发\n", c.kp, c.kd);
                        break;
                    }
                    case Cmd::Mon:
                        mon_on = c.mon_on;
                        break;
                    case Cmd::Estop:
                        g_estop.store(true); // 下一 tick 统一处理
                        break;
                    case Cmd::None:
                        break;
                    }
                }
            }

            // —— 参考轨迹 ——
            if (enabled && mode == 2)
            {
                const double t = std::chrono::duration<double>(Clock::now() - traj_t0).count();
                double u = clampd(t / traj_sec, 0.0, 1.0);
                const double s = quinticSmoothstep(u);
                for (int j = 0; j < kJointCount; ++j)
                    ref[j] = start[j] + (end[j] - start[j]) * s;
                if (u >= 1.0)
                {
                    mode = 1;
                    for (int j = 0; j < kJointCount; ++j)
                        ref[j] = start[j] = end[j];
                    std::printf("[PD] 到位，转入保持\n");
                }
            }

            // —— 读反馈 + 安全 + 下发 ——
            if (enabled)
            {
                bool tripped = false;
                for (int j = 0; j < kJointCount && !tripped; ++j)
                {
                    int32_t ma = 0;
                    double vel = 0.0, pos = 0.0;
                    if (g_motors[j]->readCurrentVelocityPosition(ma, vel, pos))
                    {
                        fail_cnt[j] = 0;
                        meas_deg[j] = motorToJointRad(j, pos) * kRadToDeg;
                        cur_ma[j] = ma;
                        ok[j] = true;

                        // 实测越限急停
                        if (meas_deg[j] < kJointLimitDeg[j][0] - kLimitMarginDeg ||
                            meas_deg[j] > kJointLimitDeg[j][1] + kLimitMarginDeg)
                        {
                            std::printf("[保护] J%d 实测 %.1f° 越限，急停\n", j + 1, meas_deg[j]);
                            tripped = true;
                        }
                        // 电流保护
                        const int32_t abs_ma = ma < 0 ? -ma : ma;
                        if (abs_ma > kCurrentEstopMa)
                        {
                            if (++over_cur[j] > kCurrentEstopTicks)
                            {
                                std::printf("[保护] J%d 电流 %.2fA 持续超限，急停\n",
                                            j + 1, ma / 1000.0);
                                tripped = true;
                            }
                        }
                        else
                            over_cur[j] = 0;
                        if (abs_ma > kCurrentWarnMa &&
                            Clock::now() - last_warn > std::chrono::seconds(1))
                        {
                            last_warn = Clock::now();
                            std::printf("[告警] J%d 电流 %.2fA 偏大\n", j + 1, ma / 1000.0);
                        }
                    }
                    else if (++fail_cnt[j] > kReadFailEstop)
                    {
                        std::printf("[保护] J%d 连续 %d 次读反馈失败，急停\n",
                                    j + 1, fail_cnt[j]);
                        tripped = true;
                    }

                    if (!tripped)
                    {
                        const double clamped = clampd(ref[j], kJointLimitDeg[j][0],
                                                      kJointLimitDeg[j][1]);
                        if (!g_motors[j]->setTargetPosition(jointToMotorRad(j, clamped * kDegToRad)))
                        {
                            std::printf("[保护] J%d 目标下发失败，急停\n", j + 1);
                            tripped = true;
                        }
                    }
                }
                if (tripped)
                {
                    stopAll();
                    enabled = false;
                    mode = 0;
                }

                // 错误位巡检（0.5s 一次，避免额外总线负载）
                if (!tripped && (tick % 100) == 0)
                {
                    for (int j = 0; j < kJointCount; ++j)
                    {
                        uint32_t bits = 0;
                        if (g_motors[j]->readErrorState(bits) && bits != 0 &&
                            bits != err_bits[j])
                        {
                            std::printf("[保护] J%d 错误位 0x%X，急停\n", j + 1, bits);
                            stopAll();
                            enabled = false;
                            mode = 0;
                            break;
                        }
                        err_bits[j] = bits;
                    }
                }
            }

            // —— 日志 / 监视打印 ——
            if (Clock::now() - last_log >= std::chrono::milliseconds(taihu::kLogSamplePeriodMs))
            {
                last_log = Clock::now();
                std::vector<taihu::MotorStatus> sts;
                for (int j = 0; j < kJointCount; ++j)
                {
                    taihu::MotorStatus st;
                    st.can_id = kMotorIds[j];
                    st.position_rad = meas_deg[j] * kDegToRad; // 关节端
                    st.velocity_rad_s = 0.0;                   // 本 demo 不记录速度
                    st.current_ma = static_cast<int32_t>(cur_ma[j]);
                    st.bus_voltage_v = 0;
                    st.error_bits = err_bits[j];
                    sts.push_back(st);
                }
                const double t = std::chrono::duration<double>(Clock::now() - t_start).count();
                logger.log(t, sts);
            }
            if (mon_on && Clock::now() - last_mon >= std::chrono::seconds(1))
            {
                last_mon = Clock::now();
                std::printf("[mon] %s q(°): %7.2f %7.2f %7.2f %7.2f | ref(°): "
                            "%7.2f %7.2f %7.2f %7.2f | I(A): %5.2f %5.2f %5.2f %5.2f\n",
                            enabled ? (mode == 2 ? "过渡" : "保持") : "空闲",
                            meas_deg[0], meas_deg[1], meas_deg[2], meas_deg[3],
                            ref[0], ref[1], ref[2], ref[3],
                            cur_ma[0] / 1000.0, cur_ma[1] / 1000.0,
                            cur_ma[2] / 1000.0, cur_ma[3] / 1000.0);
            }

            // —— 节拍 ——
            ++tick;
            next += std::chrono::milliseconds(kControlPeriodMs);
            if (next < Clock::now() - std::chrono::milliseconds(50))
                next = Clock::now() + std::chrono::milliseconds(kControlPeriodMs);
            std::this_thread::sleep_until(next);
        }

        stopAll(); // 退出即抱闸
    }

} // namespace

// =====================================================================
// 主线程：初始化 + 命令循环
// =====================================================================
int main(int argc, char **argv)
{
    const std::string ifname = (argc > 1) ? argv[1] : "can0";

    std::printf("=== 单腿 4 关节 纯 PD 位置模式 demo（200Hz）===\n");
    std::printf("接口 %s | 电机 ID 1~4 | 关节端减速比 101/162/162/101 | KP=%d KD=%d\n",
                ifname.c_str(), kPositionKp, kPositionKd);
    std::printf("限位(°): J1 ±90 | J2 ±120 | J3 −30~+180 | J4 ±90\n");
    std::printf("提示: 通道需先手动开启: sudo ./build/kcanctl up %s 1000000\n\n",
                ifname.c_str());

    if (!g_can.open(ifname.c_str(), 0))
    {
        std::fprintf(stderr, "[错误] 打开 %s 失败（先 sudo ./build/kcanctl up %s 1000000）\n",
                     ifname.c_str(), ifname.c_str());
        return 1;
    }

    for (int j = 0; j < kJointCount; ++j)
    {
        taihu::MotorConfig cfg;
        cfg.name = "J" + std::to_string(j + 1);
        cfg.can_id = kMotorIds[j];
        cfg.gear_ratio = kJointGearRatios[j];
        cfg.kp = kPositionKp;
        cfg.kd = kPositionKd;
        g_motors[j] = taihu::initMotor(g_can, cfg);
        g_motors[j]->clearError();
        double pos = 0.0;
        if (g_motors[j]->readPosition(pos))
            std::printf("  J%d (ID %u) 在线，当前 %.2f°\n", j + 1, kMotorIds[j],
                        motorToJointRad(j, pos) * kRadToDeg);
        else
            std::printf("  J%d (ID %u) 无响应！请检查接线/供电/通道\n", j + 1, kMotorIds[j]);
    }
    std::printf("\n电机当前处于抱闸状态。输入 hold 进入 PD 保持，help 查看命令。\n");
    printHelp();

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::thread ctrl(controlThread);

    char line[256];
    while (g_running.load() && std::fgets(line, sizeof line, stdin))
    {
        char cmd[32] = {0};
        if (std::sscanf(line, "%31s", cmd) != 1)
            continue;

        if (!std::strcmp(cmd, "quit") || !std::strcmp(cmd, "q"))
            break;
        else if (!std::strcmp(cmd, "help") || !std::strcmp(cmd, "h"))
            printHelp();
        else if (!std::strcmp(cmd, "hold"))
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_pending = Cmd{};
            g_pending.type = Cmd::Hold;
            g_pending_has = true;
        }
        else if (!std::strcmp(cmd, "goto"))
        {
            double a[4];
            double sec = kDefaultGotoSec;
            const int n = std::sscanf(line, "%*s %lf %lf %lf %lf %lf",
                                      &a[0], &a[1], &a[2], &a[3], &sec);
            if (n < 4)
            {
                std::printf("[错误] 用法: goto a b c d [s]\n");
                continue;
            }
            std::lock_guard<std::mutex> lk(g_mtx);
            g_pending = Cmd{};
            g_pending.type = Cmd::Goto;
            for (int j = 0; j < 4; ++j)
                g_pending.target_deg[j] = a[j];
            g_pending.sec = (n >= 5 && sec > 0.05) ? sec : kDefaultGotoSec;
            g_pending_has = true;
        }
        else if (!std::strcmp(cmd, "pos"))
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_pending = Cmd{};
            g_pending.type = Cmd::Pos;
            g_pending_has = true;
        }
        else if (!std::strcmp(cmd, "gains"))
        {
            int kp = 0, kd = 0;
            if (std::sscanf(line, "%*s %d %d", &kp, &kd) != 2 || kp <= 0 || kd < 0)
            {
                std::printf("[错误] 用法: gains kp kd\n");
                continue;
            }
            std::lock_guard<std::mutex> lk(g_mtx);
            g_pending = Cmd{};
            g_pending.type = Cmd::Gains;
            g_pending.kp = kp;
            g_pending.kd = kd;
            g_pending_has = true;
        }
        else if (!std::strcmp(cmd, "mon"))
        {
            char arg[8] = {0};
            std::sscanf(line, "%*s %7s", arg);
            std::lock_guard<std::mutex> lk(g_mtx);
            g_pending = Cmd{};
            g_pending.type = Cmd::Mon;
            g_pending.mon_on = !std::strcmp(arg, "on");
            g_pending_has = true;
        }
        else if (!std::strcmp(cmd, "estop"))
        {
            g_estop.store(true);
        }
        else
        {
            std::printf("[错误] 未知命令: %s（help 查看）\n", cmd);
        }
    }

    g_running.store(false);
    ctrl.join();
    g_can.close();
    std::printf("[退出] 全部电机已抱闸，CAN 已关闭\n");
    return 0;
}
