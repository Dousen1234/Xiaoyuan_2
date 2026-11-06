/**
 * @file lower_control.cpp
 * @brief 3588 下位机：从上位机收 16 个 ID 的目标帧 → 拆成 4 组 → 下发 4 条腿臂
 *
 * 与 examples/walk_control_V1.cpp 的定位相同、协议完全兼容，但做了两点调整：
 *   1. 腿臂命名改为 A/B/C/D（A↔can0, B↔can1, C↔can2, D↔can3），
 *      关节命名 A1~A4 … D1~D4（等价旧称 L1_J1 … L4_J4）；
 *   2. 上位机接口默认改成 can5（鲲弘第 6 路），与 3588 板载 can_mcu 对接。
 *
 * 三条总线相互独立，绝不混用：
 *   电机总线  can0 / can1 / can2 / can3   ← 各腿臂 4 个电机（ID 1~4）
 *   上位机总线 can5                        ← 与 3588 板载 CAN 对接
 * 若上位机接口与电机总线同名，上位机发的 ID 1~4 会被 A 腿真电机直接执行，
 * 程序会直接拒绝启动。
 *
 * 职责划分（精简版上位机的对侧）：
 *   下位机负责全部保护 —— 越限钳位、到达限位报警锁定、断流超时、急停抱闸。
 *   上位机只负责发轨迹和收反馈，不做任何限位判断。
 *
 * 用法：
 *   ./lower_control                          # 全部腿臂 A~D，上位机接口 can5
 *   ./lower_control --legs "A C"             # 只启用 A 和 C
 *   ./lower_control --legs all --host can5
 *   ./lower_control --dry-run                # 只打印配置不碰 CAN
 */

#include "proto.h"

#include <taihu/config.h>
#include <taihu/joint_module.h>
#include <taihu/socket_can.h>
#include <taihu/taihu_tools.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

using namespace x3588;

namespace
{

    // =========================================================================
    // 一、机械参数（与 walk_control_V1 完全一致）
    // =========================================================================

    /// 电机 CAN ID（每条腿上 J1~J4 固定为 1~4）
    const uint32_t kMotorIds[kJointCount] = {1, 2, 3, 4};

    /// 关节端总减速比（用于速度换算）
    double kJointGearRatios[kJointCount] = {101.0, 162.0, 162.0, 101.0};

    /// 电机输出端角 / 关节角
    ///   J1/J4 Y 型直驱 → 1:1
    ///   J2/J3 P 型锥齿轮 1:2 → 2:1
    double kMotorRadPerJointRad[kJointCount] = {1.0, 2.0, 2.0, 1.0};

    /// 关节方向符号（+1 或 -1）：电机输出端正转为逆时针，关节正向同为逆时针
    double kJointDirSign[kJointCount] = {1.0, 1.0, 1.0, 1.0};

    /// 各关节软限位（关节端，度）
    double kJointLimitDeg[kJointCount][2] = {
        {-120.0, 120.0}, {-120.0, 120.0}, {-30.0, 180.0}, {-90.0, 90.0}};

    /// 位置环增益
    int32_t kPositionKp = 4000;
    int32_t kPositionKd = 60;

    // =========================================================================
    // 二、运行时参数（可由命令行覆盖）
    // =========================================================================

    int kHeartbeatMs = 200;      ///< 心跳 0x161 周期
    int kLimitAlarmMs = 200;     ///< 报警 0x162 重发周期
    int kFeedbackHz = 10;        ///< 反馈回传频率
    int kStreamTimeoutMs = 1000; ///< 断流超时
    int kReadPeriodMs = 100;     ///< 电机 0x41 读取周期

    double kLimitTriggerDeg = 0.5; ///< 限位触发阈值（向内收缩，抵消 PD 稳态误差）
    double kLimitRearmDeg = 2.0;   ///< 触发后解除武装的回差

    // =========================================================================
    // 三、全局状态
    // =========================================================================

    std::atomic<bool> g_running{true};
    std::atomic<bool> g_estop{false};
    std::atomic<bool> g_estop_locked{false};

    std::atomic<bool> g_stream_armed{false};
    std::atomic<bool> g_stream_online{false};
    std::atomic<int64_t> g_stream_last_rx_ms{0};

    std::atomic<bool> g_limit_locked{false};
    std::atomic<uint16_t> g_limit_id{0};
    std::atomic<uint8_t> g_limit_side{0};
    std::atomic<uint32_t> g_limit_count{0};

    std::atomic<int64_t> g_rx_frames{0};
    std::atomic<int64_t> g_tx_frames{0};

    int64_t nowMs()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void onSignal(int) { g_running.store(false); }

    // =========================================================================
    // 四、单位换算
    // =========================================================================

    /// 关节端 rad → 电机输出端 rad
    double jointToMotorRad(int joint, double joint_rad)
    {
        return joint_rad * kMotorRadPerJointRad[joint] * kJointDirSign[joint];
    }

    /// 电机输出端 rad → 关节端 rad
    double motorToJointRad(int joint, double motor_rad)
    {
        const double k = kMotorRadPerJointRad[joint] * kJointDirSign[joint];
        return k == 0.0 ? 0.0 : motor_rad / k;
    }

    /// 关节名（"A1" 形式）打印用
    std::string jointName(int leg_idx, int joint_idx)
    {
        std::string s;
        s.push_back(legLetter(leg_idx));
        s.push_back(static_cast<char>('1' + joint_idx));
        return s;
    }

    // =========================================================================
    // 五、限位判定
    // =========================================================================

    bool withinLimits(int joint, double deg)
    {
        return deg >= kJointLimitDeg[joint][0] - 1e-9 &&
               deg <= kJointLimitDeg[joint][1] + 1e-9;
    }

    double clampToLimits(int joint, double deg)
    {
        if (deg < kJointLimitDeg[joint][0])
            return kJointLimitDeg[joint][0];
        if (deg > kJointLimitDeg[joint][1])
            return kJointLimitDeg[joint][1];
        return deg;
    }

    uint8_t detectLimitSide(int joint, double deg)
    {
        if (deg >= kJointLimitDeg[joint][1] - kLimitTriggerDeg)
            return kLimitSideMax;
        if (deg <= kJointLimitDeg[joint][0] + kLimitTriggerDeg)
            return kLimitSideMin;
        return 0;
    }

    // =========================================================================
    // 六、腿臂结构
    // =========================================================================

    struct Command
    {
        enum class Type
        {
            Move,
            Goto
        } type = Type::Move;
        int joint = 0;        ///< 关节下标 0~3
        bool relative = true; ///< Move=true 相对，Goto=false 绝对
        double angle_deg = 0.0;
        double duration_s = 2.0;
    };

    struct Leg
    {
        int index = 0;      ///< 0~3（A~D）
        std::string ifname; ///< "can0"…"can3"
        std::atomic<bool> enabled{false};

        taihu::SocketCanInterface can; ///< busThread 独占
        std::unique_ptr<taihu::JointModule> motors[kJointCount];

        std::atomic<bool> armed{false};
        std::atomic<bool> arm_req{false};
        std::atomic<bool> disarm_req{false};

        std::atomic<double> target_rad[kJointCount];
        std::atomic<bool> have_target{false};

        std::atomic<double> fb_rad[kJointCount];
        std::atomic<bool> motor_online[kJointCount];

        std::queue<Command> cmd_queue;
        std::mutex cmd_mutex;
        std::condition_variable cmd_cv;

        std::atomic<uint32_t> rx_frames{0};
        std::atomic<uint32_t> clamped{0};
    };

    Leg g_legs[kLegCount];

    // =========================================================================
    // 七、电机读写原语
    // =========================================================================

    /// 读一条腿四个关节的实际角度（关节端 rad）。单关节失败不中断。
    bool readLegJointRad(Leg &leg, double out[kJointCount])
    {
        bool all_ok = true;
        for (int j = 0; j < kJointCount; ++j)
        {
            int32_t cur_ma = 0;
            double vel = 0.0, motor_pos = 0.0;
            if (leg.motors[j]->readCurrentVelocityPosition(cur_ma, vel, motor_pos))
            {
                out[j] = motorToJointRad(j, motor_pos);
                leg.motor_online[j].store(true, std::memory_order_relaxed);
            }
            else
            {
                out[j] = leg.fb_rad[j].load(std::memory_order_relaxed); // 沿用旧值
                leg.motor_online[j].store(false, std::memory_order_relaxed);
                all_ok = false;
            }
        }
        return all_ok;
    }

    /// 对一条腿下发四个关节目标（关节端 rad → 电机输出端 → 0x1E cnt）
    void sendLegTargets(const Leg &leg, const double joint_rad[kJointCount])
    {
        for (int j = 0; j < kJointCount; ++j)
            leg.motors[j]->setTargetPosition(jointToMotorRad(j, joint_rad[j]));
    }

    /// 使能（解抱闸）。必须在 busThread 内调用（独占该腿 CAN）。
    ///
    /// 三步顺序不可交换：
    ///   1. 读实测位置 → 写入目标缓存与反馈缓存（避免使能后冲向旧目标）；
    ///   2. 逐电机 setTargetCurrent(0) = 使能 + 零转矩 = 解除抱闸；
    ///   3. 立即以当前位置为目标切回位置模式（电机内部 PD 闭环托住姿态）。
    void enableLeg(Leg &leg)
    {
        double cur[kJointCount];
        readLegJointRad(leg, cur);
        for (int j = 0; j < kJointCount; ++j)
        {
            leg.fb_rad[j].store(cur[j], std::memory_order_relaxed);
            leg.target_rad[j].store(cur[j], std::memory_order_relaxed);
        }
        leg.have_target.store(true, std::memory_order_relaxed);

        int ok_count = 0;
        for (int j = 0; j < kJointCount; ++j)
        {
            leg.motors[j]->clearError(); // 锁存错误会挡住使能
            if (leg.motors[j]->setTargetCurrent(0))
                ++ok_count;
        }

        sendLegTargets(leg, cur);
        leg.armed.store(true, std::memory_order_relaxed);

        std::printf("[腿臂 %c][使能] 抱闸已解除（%d/4 电机应答），当前姿态已作为保持目标\n",
                    legLetter(leg.index), ok_count);
        for (int j = 0; j < kJointCount; ++j)
            std::printf("            %s %+7.2f°\n",
                        jointName(leg.index, j).c_str(), cur[j] * kRadToDeg);
    }

    // =========================================================================
    // 八、软限位报警
    // =========================================================================

    void triggerLimitAlarm(int leg_idx, int joint_idx, uint8_t side, double deg)
    {
        if (g_limit_locked.load(std::memory_order_relaxed))
            return; // 保留首个报警源

        const uint16_t id = jointToId(leg_idx, joint_idx);
        g_limit_id.store(id, std::memory_order_relaxed);
        g_limit_side.store(side, std::memory_order_relaxed);
        g_limit_count.fetch_add(1, std::memory_order_relaxed);
        g_limit_locked.store(true, std::memory_order_relaxed);

        const char *side_txt = (side == kLimitSideMax) ? "上限" : "下限";
        const int li = (side == kLimitSideMax) ? 1 : 0;
        std::printf("\n[软限位][报警] %s（全局 ID %u）到达%s %.1f°（实际 %+.2f°）\n"
                    "              → 整机锁定，等待上位机 [0x03] 或键盘 clear 解除\n",
                    jointName(leg_idx, joint_idx).c_str(), id, side_txt,
                    kJointLimitDeg[joint_idx][li], deg);
    }

    void clearLimitAlarm()
    {
        if (!g_limit_locked.exchange(false, std::memory_order_relaxed))
            return;
        const uint16_t id = g_limit_id.load(std::memory_order_relaxed);
        g_limit_id.store(0, std::memory_order_relaxed);
        g_limit_side.store(0, std::memory_order_relaxed);
        std::printf("[软限位] 报警锁定已清除（原报警关节 ID %u），"
                    "恢复接收目标（各关节保持当前目标）\n",
                    id);
    }

    // =========================================================================
    // 九、手动运动（键盘 move / goto）
    // =========================================================================

    double quinticSmoothstep(double t)
    {
        return 6.0 * t * t * t * t * t - 15.0 * t * t * t * t + 10.0 * t * t * t;
    }

    bool executeMove(Leg &leg, const Command &cmd)
    {
        const int j = cmd.joint;

        double cur[kJointCount];
        if (!readLegJointRad(leg, cur))
        {
            std::printf("[腿臂 %c][%s][错误] 读取状态失败（电机无响应？），命令放弃\n",
                        legLetter(leg.index), jointName(leg.index, j).c_str());
            return false;
        }
        for (int k = 0; k < kJointCount; ++k)
            leg.fb_rad[k].store(cur[k]);

        const double cur_deg = cur[j] * kRadToDeg;
        const double target_deg =
            cmd.relative ? cur_deg + cmd.angle_deg : cmd.angle_deg;

        if (!withinLimits(j, target_deg))
        {
            std::printf("[腿臂 %c][%s][拒绝] 目标 %.1f° 超出限位 [%.0f°, %.0f°]（当前 %.1f°）\n",
                        legLetter(leg.index), jointName(leg.index, j).c_str(),
                        target_deg, kJointLimitDeg[j][0], kJointLimitDeg[j][1],
                        cur_deg);
            return false;
        }

        const double target_rad = target_deg * kDegToRad;
        const double q0[kJointCount] = {cur[0], cur[1], cur[2], cur[3]};

        std::printf("[腿臂 %c][%s] %s %.1f° → 目标 %.1f°（限位 [%.0f°, %.0f°]），"
                    "轨迹 %.1fs\n",
                    legLetter(leg.index), jointName(leg.index, j).c_str(),
                    cmd.relative ? "相对转动" : "绝对转角", cmd.angle_deg, target_deg,
                    kJointLimitDeg[j][0], kJointLimitDeg[j][1], cmd.duration_s);

        const auto t0 = std::chrono::steady_clock::now();
        while (g_running.load())
        {
            const double t = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - t0)
                                 .count();
            const double s = quinticSmoothstep(std::min(t / cmd.duration_s, 1.0));

            double q[kJointCount];
            for (int k = 0; k < kJointCount; ++k)
                q[k] = q0[k];
            q[j] = q0[j] + (target_rad - q0[j]) * s;

            if (g_estop.load() || g_estop_locked.load())
            {
                std::printf("[腿臂 %c][%s] 急停介入，轨迹中止\n",
                            legLetter(leg.index), jointName(leg.index, j).c_str());
                return false;
            }
            if (g_stream_armed.load(std::memory_order_relaxed))
            {
                std::printf("[腿臂 %c][%s] 上位机流模式开始，轨迹中止\n",
                            legLetter(leg.index), jointName(leg.index, j).c_str());
                return false;
            }

            for (int k = 0; k < kJointCount; ++k)
                leg.target_rad[k].store(q[k], std::memory_order_relaxed);
            leg.have_target.store(true, std::memory_order_relaxed);
            sendLegTargets(leg, q);

            if (t >= cmd.duration_s)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(taihu::kLoopPeriodMs));
        }

        std::printf("[腿臂 %c][%s] 已到目标 %.1f°\n", legLetter(leg.index),
                    jointName(leg.index, j).c_str(), target_deg);
        return true;
    }

    // =========================================================================
    // 十、总线线程（每腿一个，独占该腿 CAN）
    // =========================================================================

    void busThread(Leg &leg)
    {
        int64_t last_read_ms = 0;
        int offline_streak[kJointCount] = {0, 0, 0, 0};
        int64_t backoff_until_ms[kJointCount] = {0, 0, 0, 0};
        bool rearm_ok[kJointCount] = {true, true, true, true};

        while (g_running.load())
        {
            // 1. 急停（最高优先级）：抱闸 + 锁定
            if (g_estop.load())
            {
                for (int j = 0; j < kJointCount; ++j)
                    leg.motors[j]->stop();
                leg.armed.store(false, std::memory_order_relaxed);
                g_estop_locked.store(true);
                g_estop.store(false);
                std::printf("[腿臂 %c][急停] 4 电机已抱闸（0x02），锁定中（resume 解除）\n",
                            legLetter(leg.index));
                std::lock_guard<std::mutex> lock(leg.cmd_mutex);
                while (!leg.cmd_queue.empty())
                    leg.cmd_queue.pop();
            }
            else if (g_estop_locked.load())
            {
                std::lock_guard<std::mutex> lock(leg.cmd_mutex);
                while (!leg.cmd_queue.empty())
                {
                    leg.cmd_queue.pop();
                    std::printf("[腿臂 %c][拒绝] 急停锁定中，move/goto 无效（resume 解除）\n",
                                legLetter(leg.index));
                }
            }
            // 2. 使能请求
            else if (leg.arm_req.exchange(false))
            {
                enableLeg(leg);
            }
            // 2b. 抱闸请求
            else if (leg.disarm_req.exchange(false))
            {
                for (int j = 0; j < kJointCount; ++j)
                    leg.motors[j]->stop();
                leg.armed.store(false, std::memory_order_relaxed);
                leg.have_target.store(false, std::memory_order_relaxed);
                std::printf("[腿臂 %c][抱闸] 4 电机已抱闸（0x02）\n",
                            legLetter(leg.index));
            }
            // 3. 保持目标重发（流模式与非流模式一致：5ms 周期重发最新目标）
            else
            {
                if (leg.have_target.load(std::memory_order_relaxed))
                {
                    double q[kJointCount];
                    for (int j = 0; j < kJointCount; ++j)
                        q[j] = leg.target_rad[j].load(std::memory_order_relaxed);
                    sendLegTargets(leg, q);
                }

                if (g_stream_armed.load())
                {
                    // 流模式中：手动命令暂缓（清队列并提示）
                    std::lock_guard<std::mutex> lock(leg.cmd_mutex);
                    while (!leg.cmd_queue.empty())
                    {
                        leg.cmd_queue.pop();
                        std::printf("[腿臂 %c][流] 流模式进行中，move/goto 暂缓"
                                    "（上位机停发 1s 或发 0x02 退出流模式）\n",
                                    legLetter(leg.index));
                    }
                }
                else
                {
                    // 非流模式：等待手动命令
                    Command cmd;
                    bool got = false;
                    {
                        std::unique_lock<std::mutex> lock(leg.cmd_mutex);
                        if (leg.cmd_cv.wait_for(
                                lock, std::chrono::milliseconds(50), [&]
                                { return !leg.cmd_queue.empty() || g_estop.load() ||
                                         g_stream_armed.load(); }))
                        {
                            if (!leg.cmd_queue.empty())
                            {
                                cmd = leg.cmd_queue.front();
                                leg.cmd_queue.pop();
                                got = true;
                            }
                        }
                    }
                    if (got)
                        executeMove(leg, cmd);
                }
            }

            // 4. 10Hz 读实际位置 + 软限位检测
            const int64_t now_ms = nowMs();
            if (now_ms - last_read_ms >= kReadPeriodMs)
            {
                last_read_ms = now_ms;
                for (int j = 0; j < kJointCount; ++j)
                {
                    if (now_ms < backoff_until_ms[j])
                    {
                        leg.motor_online[j].store(false, std::memory_order_relaxed);
                        continue;
                    }
                    int32_t cur_ma = 0;
                    double vel = 0.0, motor_pos = 0.0;
                    if (leg.motors[j]->readCurrentVelocityPosition(cur_ma, vel,
                                                                   motor_pos))
                    {
                        const double jr = motorToJointRad(j, motor_pos);
                        leg.fb_rad[j].store(jr, std::memory_order_relaxed);
                        leg.motor_online[j].store(true, std::memory_order_relaxed);
                        offline_streak[j] = 0;

                        const double deg = jr * kRadToDeg;
                        if (!rearm_ok[j])
                        {
                            if (deg <= kJointLimitDeg[j][1] - kLimitRearmDeg &&
                                deg >= kJointLimitDeg[j][0] + kLimitRearmDeg)
                                rearm_ok[j] = true;
                        }
                        else if (g_stream_armed.load(std::memory_order_relaxed) &&
                                 !g_limit_locked.load(std::memory_order_relaxed))
                        {
                            const uint8_t side = detectLimitSide(j, deg);
                            if (side != 0)
                            {
                                triggerLimitAlarm(leg.index, j, side, deg);
                                rearm_ok[j] = false;
                            }
                        }
                    }
                    else
                    {
                        leg.motor_online[j].store(false, std::memory_order_relaxed);
                        if (++offline_streak[j] >= 3)
                            backoff_until_ms[j] = nowMs() + 1000;
                    }
                }
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(taihu::kLoopPeriodMs));
        }

        for (int j = 0; j < kJointCount; ++j)
            leg.motors[j]->stop();
    }

    // =========================================================================
    // 十一、上位机命令线程
    // =========================================================================

    void hostSend(taihu::SocketCanInterface &can, uint16_t id, uint8_t dlc,
                  const uint8_t *data)
    {
        taihu::CanFrame f;
        f.id = id;
        f.dlc = dlc;
        if (data)
            for (int i = 0; i < 8; ++i)
                f.data[i] = data[i];
        if (can.send(f))
            g_tx_frames.fetch_add(1);
    }

    /// 回传单个关节的 0x41 反馈帧（数据源 = busThread 刷新的反馈缓存）
    void sendFeedback(taihu::SocketCanInterface &can, uint16_t id)
    {
        const int leg_idx = idToLeg(id);
        const int joint_idx = idToJoint(id);
        const double rad =
            g_legs[leg_idx].fb_rad[joint_idx].load(std::memory_order_relaxed);
        uint8_t d[8] = {0};
        d[0] = kCmdGetState;
        putI32LE(&d[1], radToProto(rad));
        hostSend(can, id, 5, d);
    }

    /// 软限位报警帧（0x162，DLC=8）
    void sendLimitAlarm(taihu::SocketCanInterface &can)
    {
        const uint16_t jid = g_limit_id.load(std::memory_order_relaxed);
        const uint8_t side = g_limit_side.load(std::memory_order_relaxed);
        if (!isJointId(jid) || side == 0)
            return;

        const double rad = g_legs[idToLeg(jid)]
                               .fb_rad[idToJoint(jid)]
                               .load(std::memory_order_relaxed);

        uint8_t d[8] = {0};
        d[0] = kLimitAlarmCode;
        d[1] = static_cast<uint8_t>(jid);
        d[2] = side;
        d[3] = 0;
        putI32LE(&d[4], radToProto(rad));
        hostSend(can, kLimitAlarmId, 8, d);
    }

    void hostThread(const char *ifname)
    {
        taihu::SocketCanInterface can;
        if (!can.open(ifname, 0))
        {
            std::printf("[协议][错误] 打开 %s 失败，上位机链路不可用\n", ifname);
            return;
        }
        std::printf("[协议] 上位机链路就绪：%s\n", ifname);
        std::printf("       监听 ID %u~%u 目标帧 [0x%02X][int32 LE = 关节端 rad × %.0f]\n",
                    kJointIdBase, kJointIdBase + kTotalJoints - 1, kCmdSetPos,
                    kAngleScale);
        std::printf("       退出流 [0x%02X]  清限位 [0x%02X]  状态请求 [0x%02X]\n",
                    kCmdStop, kCmdClearLimit, kCmdGetState);
        std::printf("       上报 ID %u~%u 反馈 / 0x%03X 心跳 / 0x%03X 报警\n",
                    kJointIdBase, kJointIdBase + kTotalJoints - 1, kHeartbeatId,
                    kLimitAlarmId);

        const int feedback_ms = 1000 / (kFeedbackHz > 0 ? kFeedbackHz : 10);

        int64_t last_fb = nowMs();
        int64_t last_hb = nowMs();
        int64_t last_alarm = 0;
        int clamped_leg_reported[kLegCount] = {0, 0, 0, 0};

        while (g_running.load())
        {
            taihu::CanFrame rx;
            if (!can.receive(rx, 20))
            {
                const int64_t now = nowMs();

                // —— 周期反馈 ——
                if (now - last_fb >= feedback_ms)
                {
                    last_fb = now;
                    for (int i = 0; i < kLegCount; ++i)
                    {
                        if (!g_legs[i].enabled.load(std::memory_order_relaxed))
                            continue;
                        for (int j = 0; j < kJointCount; ++j)
                            sendFeedback(can, jointToId(i, j));
                    }
                }

                // —— 心跳 ——
                if (now - last_hb >= kHeartbeatMs)
                {
                    uint8_t d[1];
                    d[0] = static_cast<uint8_t>(
                        (g_stream_armed.load() ? kHbBitStreaming : 0) |
                        (g_stream_online.load() ? kHbBitOnline : 0) |
                        (g_limit_locked.load() ? kHbBitLimitLock : 0));
                    hostSend(can, kHeartbeatId, 1, d);
                    last_hb = now;
                }

                // —— 软限位报警重发 ——
                if (g_limit_locked.load())
                {
                    if (last_alarm == 0 || now - last_alarm >= kLimitAlarmMs)
                    {
                        sendLimitAlarm(can);
                        last_alarm = now;
                    }
                }
                else
                {
                    last_alarm = 0;
                }

                // —— 断流超时 ——
                if (g_stream_armed.load() &&
                    now - g_stream_last_rx_ms.load() > kStreamTimeoutMs)
                {
                    g_stream_armed.store(false);
                    g_stream_online.store(false);
                    std::printf("[流][警告] 超过 %dms 未收到新样点，自动退出流模式"
                                "（电机保持当前目标）\n",
                                kStreamTimeoutMs);
                }
                continue;
            }

            g_rx_frames.fetch_add(1);

            if (rx.is_extended)
                continue;
            const uint16_t id = static_cast<uint16_t>(rx.id & 0x7FF);
            if (!isJointId(id))
                continue; // 非关节 ID（如误接的总线回帧）忽略

            const int leg_idx = idToLeg(id);
            const int joint_idx = idToJoint(id);
            Leg &leg = g_legs[leg_idx];

            // —— [0x02] 退出流模式（电机保持，不抱闸）——
            if (rx.dlc == 1 && rx.data[0] == kCmdStop)
            {
                if (g_stream_armed.load())
                {
                    g_stream_armed.store(false);
                    g_stream_online.store(false);
                    std::printf("[流] 收到 0x02（ID %u）：退出流模式，电机保持当前目标\n",
                                id);
                }
                continue;
            }

            // —— [0x03] 清除软限位报警锁定 ——
            if (rx.dlc == 1 && rx.data[0] == kCmdClearLimit)
            {
                clearLimitAlarm();
                continue;
            }

            // —— [0x41] 状态请求 → 立即回该 ID 实际角 ——
            if (rx.dlc == 1 && rx.data[0] == kCmdGetState)
            {
                sendFeedback(can, id);
                continue;
            }

            // —— [0x1E] 目标帧 ——
            if (!(rx.dlc == 5 && rx.data[0] == kCmdSetPos))
                continue;

            if (g_estop_locked.load() || g_limit_locked.load())
                continue;

            const double rad = protoToRad(getI32LE(&rx.data[1]));
            const double deg = rad * kRadToDeg;

            leg.rx_frames.fetch_add(1, std::memory_order_relaxed);

            if (!withinLimits(joint_idx, deg))
            {
                // 越限：钳位后照常下发（关节走到限位停住，随后由实际位置触发报警）
                const double cdeg = clampToLimits(joint_idx, deg);
                leg.target_rad[joint_idx].store(cdeg * kDegToRad,
                                                std::memory_order_relaxed);
                leg.clamped.fetch_add(1, std::memory_order_relaxed);
                if (clamped_leg_reported[leg_idx] < 3)
                {
                    ++clamped_leg_reported[leg_idx];
                    std::printf("[流][腿臂 %c][%s][钳位] 目标 %+.2f° 越限"
                                "（限位 [%.0f°, %.0f°]）→ 钳位到 %+.2f°\n",
                                legLetter(leg_idx),
                                jointName(leg_idx, joint_idx).c_str(), deg,
                                kJointLimitDeg[joint_idx][0],
                                kJointLimitDeg[joint_idx][1], cdeg);
                }
            }
            else
            {
                leg.target_rad[joint_idx].store(rad, std::memory_order_relaxed);
            }

            leg.have_target.store(true, std::memory_order_relaxed);
            if (!g_stream_armed.load())
            {
                g_stream_armed.store(true);
                std::printf("[流] 收到首帧 0x1E（ID %u = %s）：进入流模式\n", id,
                            jointName(leg_idx, joint_idx).c_str());
            }
            g_stream_online.store(true);
            g_stream_last_rx_ms.store(nowMs());
        }

        can.close();
    }

    // =========================================================================
    // 十二、参数解析与帮助
    // =========================================================================

    struct Options
    {
        bool show_help = false;
        bool dry_run = false;
        bool legs_given = false;
        bool want[kLegCount] = {false, false, false, false};
        bool host_given = false;
        std::string host_ifname = "can5";
        bool auto_arm = true;
    };

    void printUsage(const char *prog)
    {
        std::printf(
            "用法: %s [选项]\n"
            "\n"
            "腿臂与通道:\n"
            "  --legs \"A B C D\"    启用的腿臂（字母或 1~4，空格/逗号分隔；all = 全部）\n"
            "                       默认 A~D 全启用，各腿臂固定占用 can0~can3\n"
            "  --host <接口名>      上位机 CAN 接口（默认 can5；不可用 can0~can3）\n"
            "  --no-host            不启用上位机链路（仅键盘手动命令）\n"
            "\n"
            "保护参数:\n"
            "  --limits \"a,b;c,d;e,f;g,h\"  J1~J4 软限位（度）\n"
            "                        默认 -120,120;-120,120;-30,180;-90,90\n"
            "  --trigger-deg <度>   限位触发阈值（默认 %.1f）\n"
            "  --rearm-deg <度>     解除锁定后重新武装的回差（默认 %.1f）\n"
            "  --stream-timeout-ms <ms>  断流超时（默认 %d）\n"
            "  --heartbeat-ms <ms>  心跳 0x161 周期（默认 %d）\n"
            "  --alarm-ms <ms>      报警 0x162 重发周期（默认 %d）\n"
            "  --feedback-hz <hz>   反馈回传频率（默认 %d，范围 1~100）\n"
            "\n"
            "机械参数:\n"
            "  --dir-sign \"+1,+1,+1,+1\"  关节方向符号（仅 ±1）\n"
            "  --gear \"101,162,162,101\"  关节端总减速比\n"
            "  --motor-scale \"1,2,2,1\"   电机输出端角 / 关节角\n"
            "  --kp <int>           位置环 KP（默认 %d）\n"
            "  --kd <int>           位置环 KD（默认 %d）\n"
            "\n"
            "其它:\n"
            "  --dry-run            只打印生效配置后退出（不碰 CAN）\n"
            "  -h, --help           显示本帮助\n"
            "\n"
            "示例:\n"
            "  %s                            # 全腿臂，上位机 can5\n"
            "  %s --legs \"A B\"               # 只启用 A 和 B\n"
            "  %s --legs all --host can5     # 显式指定\n",
            prog, kLimitTriggerDeg, kLimitRearmDeg, kStreamTimeoutMs, kHeartbeatMs,
            kLimitAlarmMs, kFeedbackHz, kPositionKp, kPositionKd, prog, prog, prog);
    }

    /// 解析逗号/分号/空格分隔的 4 个浮点数
    bool parseFourDoubles(const char *tok, double dst[kJointCount], double lo,
                          double hi)
    {
        if (tok == nullptr)
            return false;
        std::string s(tok);
        if (s.empty())
            return false;
        char *save = nullptr;
        int i = 0;
        for (char *t = strtok_r(&s[0], ",;: \t", &save); t != nullptr;
             t = strtok_r(nullptr, ",;: \t", &save), ++i)
        {
            if (i >= kJointCount)
                return false;
            char *end = nullptr;
            const double v = std::strtod(t, &end);
            if (end == t || *end != '\0' || v < lo || v > hi)
                return false;
            dst[i] = v;
        }
        return i == kJointCount;
    }

    /// 解析限位串 "a,b;c,d;e,f;g,h"
    bool parseLimits(const char *tok, double dst[kJointCount][2])
    {
        if (tok == nullptr)
            return false;
        std::string s(tok);
        char *save = nullptr;
        int i = 0;
        for (char *seg = strtok_r(&s[0], ";", &save); seg != nullptr;
             seg = strtok_r(nullptr, ";", &save), ++i)
        {
            if (i >= kJointCount)
                return false;
            char *save2 = nullptr;
            int k = 0;
            double v[2] = {0, 0};
            for (char *t = strtok_r(seg, ",: \t", &save2); t != nullptr;
                 t = strtok_r(nullptr, ",: \t", &save2), ++k)
            {
                if (k >= 2)
                    return false;
                v[k] = std::atof(t);
            }
            if (k != 2 || v[0] >= v[1])
                return false;
            dst[i][0] = v[0];
            dst[i][1] = v[1];
        }
        return i == kJointCount;
    }

    /// 解析腿臂标识："A"/"a"/"1" → 下标 0
    int parseLegToken(const char *tok)
    {
        if (tok == nullptr || tok[0] == '\0')
            return -1;
        const char c = tok[0];
        if (c >= 'A' && c <= 'D')
            return c - 'A';
        if (c >= 'a' && c <= 'd')
            return c - 'a';
        if (c >= '1' && c <= '4')
            return c - '1';
        return -1;
    }

    bool parseArgs(int argc, char **argv, Options &o)
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string a = argv[i];
            auto need = [&](const char *name) -> const char *
            {
                if (i + 1 >= argc)
                {
                    std::printf("[错误] 参数 %s 缺少取值\n", name);
                    return nullptr;
                }
                return argv[++i];
            };

            if (a == "-h" || a == "--help")
            {
                o.show_help = true;
            }
            else if (a == "--dry-run")
            {
                o.dry_run = true;
            }
            else if (a == "--no-host")
            {
                o.host_given = true;
                o.host_ifname.clear();
            }
            else if (a == "--host")
            {
                const char *v = need("--host");
                if (!v)
                    return false;
                o.host_given = true;
                o.host_ifname = v;
            }
            else if (a == "--legs")
            {
                const char *v = need("--legs");
                if (!v)
                    return false;
                o.legs_given = true;
                for (int k = 0; k < kLegCount; ++k)
                    o.want[k] = false;
                std::string s(v);
                if (s == "all" || s == "ALL")
                {
                    for (int k = 0; k < kLegCount; ++k)
                        o.want[k] = true;
                }
                else
                {
                    char *save = nullptr;
                    for (char *t = strtok_r(&s[0], ",; \t", &save); t != nullptr;
                         t = strtok_r(nullptr, ",; \t", &save))
                    {
                        const int idx = parseLegToken(t);
                        if (idx < 0)
                        {
                            std::printf("[错误] 腿臂标识非法：\"%s\"（应为 A~D 或 1~4）\n", t);
                            return false;
                        }
                        o.want[idx] = true;
                    }
                }
            }
            else if (a == "--limits")
            {
                const char *v = need("--limits");
                if (!v)
                    return false;
                if (!parseLimits(v, kJointLimitDeg))
                {
                    std::printf("[错误] --limits 格式应为 \"a,b;c,d;e,f;g,h\"\n");
                    return false;
                }
            }
            else if (a == "--trigger-deg")
            {
                const char *v = need("--trigger-deg");
                if (!v)
                    return false;
                kLimitTriggerDeg = std::atof(v);
                if (kLimitTriggerDeg <= 0.0 || kLimitTriggerDeg > 30.0)
                {
                    std::printf("[错误] --trigger-deg 应在 (0, 30] 度\n");
                    return false;
                }
            }
            else if (a == "--rearm-deg")
            {
                const char *v = need("--rearm-deg");
                if (!v)
                    return false;
                kLimitRearmDeg = std::atof(v);
                if (kLimitRearmDeg <= 0.0 || kLimitRearmDeg > 60.0)
                {
                    std::printf("[错误] --rearm-deg 应在 (0, 60] 度\n");
                    return false;
                }
            }
            else if (a == "--stream-timeout-ms")
            {
                const char *v = need("--stream-timeout-ms");
                if (!v)
                    return false;
                kStreamTimeoutMs = std::atoi(v);
            }
            else if (a == "--heartbeat-ms")
            {
                const char *v = need("--heartbeat-ms");
                if (!v)
                    return false;
                kHeartbeatMs = std::atoi(v);
            }
            else if (a == "--alarm-ms")
            {
                const char *v = need("--alarm-ms");
                if (!v)
                    return false;
                kLimitAlarmMs = std::atoi(v);
            }
            else if (a == "--feedback-hz")
            {
                const char *v = need("--feedback-hz");
                if (!v)
                    return false;
                kFeedbackHz = std::atoi(v);
                if (kFeedbackHz < 1 || kFeedbackHz > 100)
                {
                    std::printf("[错误] --feedback-hz 应在 1~100\n");
                    return false;
                }
            }
            else if (a == "--dir-sign")
            {
                const char *v = need("--dir-sign");
                if (!v)
                    return false;
                if (!parseFourDoubles(v, kJointDirSign, -1.0, 1.0))
                {
                    std::printf("[错误] --dir-sign 需要 4 个 ±1\n");
                    return false;
                }
                for (int j = 0; j < kJointCount; ++j)
                {
                    if (std::fabs(std::fabs(kJointDirSign[j]) - 1.0) > 1e-9)
                    {
                        std::printf("[错误] --dir-sign 只能取 +1 或 -1\n");
                        return false;
                    }
                }
            }
            else if (a == "--gear")
            {
                const char *v = need("--gear");
                if (!v)
                    return false;
                if (!parseFourDoubles(v, kJointGearRatios, 1.0, 1000.0))
                {
                    std::printf("[错误] --gear 需要 4 个正数\n");
                    return false;
                }
            }
            else if (a == "--motor-scale")
            {
                const char *v = need("--motor-scale");
                if (!v)
                    return false;
                if (!parseFourDoubles(v, kMotorRadPerJointRad, 0.01, 100.0))
                {
                    std::printf("[错误] --motor-scale 需要 4 个正数\n");
                    return false;
                }
            }
            else if (a == "--kp")
            {
                const char *v = need("--kp");
                if (!v)
                    return false;
                kPositionKp = std::atoi(v);
            }
            else if (a == "--kd")
            {
                const char *v = need("--kd");
                if (!v)
                    return false;
                kPositionKd = std::atoi(v);
            }
            else
            {
                std::printf("[错误] 未知参数：%s\n\n", a.c_str());
                return false;
            }
        }
        return true;
    }

    void printConfig()
    {
        std::printf("\n[参数] 腿臂配置\n");
        for (int i = 0; i < kLegCount; ++i)
        {
            std::printf("       腿臂 %c ↔ %s %s\n", legLetter(i),
                        ("can" + std::to_string(i)).c_str(),
                        g_legs[i].enabled.load() ? "" : "（未启用）");
        }
        std::printf("[参数] 关节软限位（关节端）\n");
        for (int j = 0; j < kJointCount; ++j)
            std::printf("       J%d: [%+.0f°, %+.0f°]  减速比 %.0f  电机/关节 %.0f\n",
                        j + 1, kJointLimitDeg[j][0], kJointLimitDeg[j][1],
                        kJointGearRatios[j], kMotorRadPerJointRad[j]);
        std::printf("[参数] KP=%d KD=%d  下发周期 %dms  反馈读取 %dms\n", kPositionKp,
                    kPositionKd, taihu::kLoopPeriodMs, kReadPeriodMs);
        std::printf("[参数] 心跳 %dms  报警重发 %dms  反馈 %dHz  断流超时 %dms\n",
                    kHeartbeatMs, kLimitAlarmMs, kFeedbackHz, kStreamTimeoutMs);
        std::printf("[参数] 限位触发 %.1f°  重武装回差 %.1f°\n", kLimitTriggerDeg,
                    kLimitRearmDeg);
    }

    void printKeyboardHelp()
    {
        std::printf(
            "命令（注意关节名用 A1~D4，也可用 1J1 这种旧写法）：\n"
            "  move <腿臂> <关节> cw|ccw <角度> [时长s]   相对转动（cw=负，ccw=正）\n"
            "  goto <腿臂> <关节> <角度> [时长s]          绝对转到指定角度\n"
            "  arm [腿臂]                                 使能（解抱闸）\n"
            "  disarm [腿臂]                              抱闸\n"
            "  estop                                      急停（全部抱闸并锁定）\n"
            "  resume                                     解除急停锁定\n"
            "  clear                                      清除软限位报警锁定\n"
            "  pos                                        打印各关节实际角度\n"
            "  stat                                       打印统计\n"
            "  help                                       帮助\n"
            "  quit                                       退出\n");
    }

    // =========================================================================
    // 十三、腿臂初始化
    // =========================================================================

    bool initLeg(int leg_idx, bool probe)
    {
        Leg &leg = g_legs[leg_idx];
        leg.index = leg_idx;
        leg.ifname = "can" + std::to_string(leg_idx);

        if (!leg.can.open(leg.ifname.c_str(), 0))
        {
            std::printf("[腿臂 %c][错误] 打开 %s 失败（接口需先 up："
                        "sudo ./build/kcanctl up %s 1000000）\n",
                        legLetter(leg_idx), leg.ifname.c_str(), leg.ifname.c_str());
            return false;
        }

        for (int j = 0; j < kJointCount; ++j)
        {
            taihu::MotorConfig cfg{
                "leg" + std::string(1, legLetter(leg_idx)) + "_J" + std::to_string(j + 1),
                kMotorIds[j], kJointGearRatios[j], kPositionKp, kPositionKd};
            leg.motors[j] = taihu::initMotor(leg.can, cfg);
            leg.motors[j]->clearError();
        }

        if (probe)
        {
            std::printf("[腿臂 %c] %s 电机在线性：", legLetter(leg_idx),
                        leg.ifname.c_str());
            for (int j = 0; j < kJointCount; ++j)
            {
                int32_t cur_ma = 0;
                double vel = 0.0, pos = 0.0;
                const bool ok = leg.motors[j]->readCurrentVelocityPosition(cur_ma, vel, pos);
                std::printf("%s%s", j > 0 ? " " : "",
                            ok ? jointName(leg_idx, j).c_str() : "×");
            }
            std::printf("（× = 无响应）\n");
        }

        enableLeg(leg); // 启动即解抱闸
        leg.enabled.store(true);
        return true;
    }

    /// 解析并路由一条键盘命令；返回 false 表示应退出
    bool handleCommand(char *line, std::string &host_ifname)
    {
        char *save = nullptr;
        char *cmd = strtok_r(line, " \t", &save);
        if (cmd == nullptr)
            return true;

        if (std::strcmp(cmd, "quit") == 0 || std::strcmp(cmd, "q") == 0)
            return false;

        if (std::strcmp(cmd, "help") == 0)
        {
            printKeyboardHelp();
            return true;
        }
        if (std::strcmp(cmd, "estop") == 0)
        {
            g_estop.store(true);
            std::printf("[急停] 已下发，各腿臂将立即抱闸并锁定\n");
            return true;
        }
        if (std::strcmp(cmd, "resume") == 0)
        {
            g_estop_locked.store(false);
            std::printf("[急停] 锁定已解除（各关节仍处抱闸状态，需 arm 重新使能）\n");
            return true;
        }
        if (std::strcmp(cmd, "clear") == 0)
        {
            clearLimitAlarm();
            return true;
        }
        if (std::strcmp(cmd, "pos") == 0)
        {
            for (int i = 0; i < kLegCount; ++i)
            {
                if (!g_legs[i].enabled.load())
                    continue;
                std::printf("腿臂 %c:", legLetter(i));
                for (int j = 0; j < kJointCount; ++j)
                {
                    const bool on = g_legs[i].motor_online[j].load();
                    if (on)
                        std::printf("  %s %+7.2f°", jointName(i, j).c_str(),
                                    g_legs[i].fb_rad[j].load() * kRadToDeg);
                    else
                        std::printf("  %s   --.--°", jointName(i, j).c_str());
                }
                std::printf("\n");
            }
            return true;
        }
        if (std::strcmp(cmd, "stat") == 0)
        {
            std::printf("流模式:%s  样点在线:%s  限位锁定:%s  急停锁定:%s\n",
                        g_stream_armed.load() ? "是" : "否",
                        g_stream_online.load() ? "是" : "否",
                        g_limit_locked.load() ? "是" : "否",
                        g_estop_locked.load() ? "是" : "否");
            std::printf("累计：收 %lld 帧，发 %lld 帧，限位报警 %u 次\n",
                        static_cast<long long>(g_rx_frames.load()),
                        static_cast<long long>(g_tx_frames.load()),
                        g_limit_count.load());
            for (int i = 0; i < kLegCount; ++i)
            {
                if (!g_legs[i].enabled.load())
                    continue;
                std::printf("  腿臂 %c: 目标帧 %u  钳位 %u  使能:%s\n", legLetter(i),
                            g_legs[i].rx_frames.load(), g_legs[i].clamped.load(),
                            g_legs[i].armed.load() ? "是" : "否");
            }
            return true;
        }
        if (std::strcmp(cmd, "arm") == 0 || std::strcmp(cmd, "disarm") == 0)
        {
            const bool arm = (std::strcmp(cmd, "arm") == 0);
            char *t = strtok_r(nullptr, " \t", &save);
            if (t == nullptr || std::strcmp(t, "all") == 0)
            {
                for (int i = 0; i < kLegCount; ++i)
                {
                    if (!g_legs[i].enabled.load())
                        continue;
                    if (arm)
                        g_legs[i].arm_req.store(true);
                    else
                        g_legs[i].disarm_req.store(true);
                }
                std::printf("[%s] 已请求全部腿臂%s\n", arm ? "使能" : "抱闸",
                            arm ? "解抱闸" : "");
                return true;
            }
            const int idx = parseLegToken(t);
            if (idx < 0 || !g_legs[idx].enabled.load())
            {
                std::printf("[命令] 腿臂标识非法或未启用：%s\n", t);
                return true;
            }
            if (arm)
            {
                g_legs[idx].arm_req.store(true);
                std::printf("[使能] 已请求腿臂 %c 解抱闸\n", legLetter(idx));
            }
            else
            {
                g_legs[idx].disarm_req.store(true);
                std::printf("[抱闸] 已请求腿臂 %c 抱闸\n", legLetter(idx));
            }
            return true;
        }
        if (std::strcmp(cmd, "move") == 0 || std::strcmp(cmd, "goto") == 0)
        {
            const bool relative = (std::strcmp(cmd, "move") == 0);

            char *leg_tok = strtok_r(nullptr, " \t", &save);
            char *joint_tok = strtok_r(nullptr, " \t", &save);
            if (leg_tok == nullptr || joint_tok == nullptr)
            {
                std::printf("[命令] 用法：move <腿臂> <关节> cw|ccw <角度> [时长s]\n");
                return true;
            }
            const int leg_idx = parseLegToken(leg_tok);
            if (leg_idx < 0)
            {
                std::printf("[命令] 腿臂标识需为 A~D 或 1~4，收到 \"%s\"\n", leg_tok);
                return true;
            }
            if (!g_legs[leg_idx].enabled.load())
            {
                std::printf("[命令] 腿臂 %c 未启用\n", legLetter(leg_idx));
                return true;
            }

            // 关节：支持 "J1"/"j1"/"A1"/"1"
            int joint_idx = -1;
            {
                std::string jt(joint_tok);
                if (jt.size() == 2 && (jt[0] == 'J' || jt[0] == 'j') && jt[1] >= '1' &&
                    jt[1] <= '4')
                    joint_idx = jt[1] - '1';
                else if (jt.size() == 2 && (jt[0] == 'A' || jt[0] == 'a') &&
                         jt[1] >= '1' && jt[1] <= '4')
                    joint_idx = jt[1] - '1';
                else if (jt.size() == 1 && jt[0] >= '1' && jt[0] <= '4')
                    joint_idx = jt[0] - '1';
            }
            if (joint_idx < 0)
            {
                std::printf("[命令] 关节需为 J1~J4 或 A1~A4 中的编号，收到 \"%s\"\n",
                            joint_tok);
                return true;
            }

            double sign = 1.0;
            char *angle_tok = nullptr;
            if (relative)
            {
                char *dir_tok = strtok_r(nullptr, " \t", &save);
                if (dir_tok == nullptr)
                {
                    std::printf("[命令] move 需要方向 cw 或 ccw\n");
                    return true;
                }
                if (std::strcmp(dir_tok, "cw") == 0 || std::strcmp(dir_tok, "CW") == 0)
                    sign = -1.0;
                else if (std::strcmp(dir_tok, "ccw") == 0 ||
                         std::strcmp(dir_tok, "CCW") == 0)
                    sign = 1.0;
                else
                {
                    std::printf("[命令] 方向只能是 cw 或 ccw\n");
                    return true;
                }
                angle_tok = strtok_r(nullptr, " \t", &save);
            }
            else
            {
                angle_tok = strtok_r(nullptr, " \t", &save);
            }
            if (angle_tok == nullptr)
            {
                std::printf("[命令] 缺少角度\n");
                return true;
            }

            Command c;
            c.type = relative ? Command::Type::Move : Command::Type::Goto;
            c.joint = joint_idx;
            c.relative = relative;
            c.angle_deg = sign * std::fabs(std::atof(angle_tok));
            if (!relative)
                c.angle_deg = std::atof(angle_tok);
            c.duration_s = 2.0;
            char *dur_tok = strtok_r(nullptr, " \t", &save);
            if (dur_tok != nullptr)
            {
                const double d = std::atof(dur_tok);
                if (d >= 0.1 && d <= 60.0)
                    c.duration_s = d;
            }

            {
                std::lock_guard<std::mutex> lock(g_legs[leg_idx].cmd_mutex);
                g_legs[leg_idx].cmd_queue.push(c);
            }
            g_legs[leg_idx].cmd_cv.notify_all();
            return true;
        }

        std::printf("[命令] 未知命令：%s（输入 help 查看）\n", cmd);
        (void)host_ifname;
        return true;
    }

} // namespace

// =========================================================================
// main
// =========================================================================

int main(int argc, char **argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt))
    {
        std::printf("\n");
        printUsage(argv[0]);
        return 2;
    }
    if (opt.show_help)
    {
        printUsage(argv[0]);
        return 0;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // 默认全部启用
    if (!opt.legs_given)
        for (int i = 0; i < kLegCount; ++i)
            opt.want[i] = true;
    for (int i = 0; i < kLegCount; ++i)
        g_legs[i].enabled.store(false);

    std::printf("════════════════════════════════════════════════════════════════\n");
    std::printf("  3588 下位机 · 腿臂 A~D 控制 · V2.0 协议\n");
    std::printf("════════════════════════════════════════════════════════════════\n");
    std::printf("  腿臂 A↔can0  B↔can1  C↔can2  D↔can3（各 4 个电机，ID 1~4）\n");
    std::printf("  上位机：独立 CAN 接口（默认 can5），ID 1~16 = (腿臂-1)×4+关节号\n");
    std::printf("  保护：越限钳位下发；实际角到限位 → 整机锁定 + 0x162 上报\n");
    std::printf("────────────────────────────────────────────────────────────────\n");

    // 上位机接口不能与电机总线重名
    if (!opt.host_ifname.empty())
    {
        for (int i = 0; i < kLegCount; ++i)
        {
            if (opt.host_ifname == "can" + std::to_string(i))
            {
                std::printf("[错误] 上位机接口不能与电机总线 %s 共用"
                            "（ID 1~4 会被腿臂 A 的真电机直接执行）\n",
                            opt.host_ifname.c_str());
                return 2;
            }
        }
    }

    if (opt.dry_run)
    {
        for (int i = 0; i < kLegCount; ++i)
            g_legs[i].enabled.store(opt.want[i]);
        std::printf("[dry-run] 启用腿臂：");
        for (int i = 0; i < kLegCount; ++i)
            if (opt.want[i])
                std::printf("%c ", legLetter(i));
        std::printf("\n[dry-run] 上位机接口：%s\n",
                    opt.host_ifname.empty() ? "（不启用）" : opt.host_ifname.c_str());
        printConfig();
        std::printf("[dry-run] 参数校验通过，未打开任何 CAN 接口。\n");
        return 0;
    }

    // —— 初始化各腿臂 ——
    int ok_count = 0;
    for (int i = 0; i < kLegCount; ++i)
    {
        if (!opt.want[i])
            continue;
        if (initLeg(i, /*probe=*/true))
            ++ok_count;
    }
    if (ok_count == 0)
    {
        std::printf("[错误] 没有任何腿臂初始化成功，退出\n");
        return 1;
    }
    std::printf("[信息] %d/4 条腿臂就绪\n", ok_count);

    printConfig();
    printKeyboardHelp();
    std::printf("────────────────────────────────────────────────────────────────\n");

    // —— 启动线程 ——
    std::vector<std::thread> bus_threads;
    for (int i = 0; i < kLegCount; ++i)
        if (g_legs[i].enabled.load())
            bus_threads.emplace_back(busThread, std::ref(g_legs[i]));

    std::thread host;
    if (!opt.host_ifname.empty())
        host = std::thread(hostThread, opt.host_ifname.c_str());
    else
        std::printf("[协议] 未配置上位机接口：仅键盘命令可用\n");

    // —— 主循环：键盘 ——
    char line[256];
    std::string host_copy = opt.host_ifname;
    while (g_running.load())
    {
        std::printf("> ");
        std::fflush(stdout);
        if (std::fgets(line, sizeof line, stdin) == nullptr)
            break;
        size_t n = std::strlen(line);
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'))
            line[--n] = '\0';
        if (line[0] == '\0')
            continue;
        if (!handleCommand(line, host_copy))
            break;
    }

    // —— 收尾 ——
    std::printf("\n[退出] 正在停止各线程并抱闸...\n");
    g_running.store(false);
    for (int i = 0; i < kLegCount; ++i)
        if (g_legs[i].enabled.load())
            g_legs[i].cmd_cv.notify_all();

    for (auto &t : bus_threads)
        if (t.joinable())
            t.join();
    if (host.joinable())
        host.join();

    for (int i = 0; i < kLegCount; ++i)
    {
        if (!g_legs[i].enabled.load())
            continue;
        for (int j = 0; j < kJointCount; ++j)
            g_legs[i].motors[j]->stop();
        g_legs[i].can.close();
    }

    std::printf("[退出] 完成。累计收 %lld 帧，发 %lld 帧。\n",
                static_cast<long long>(g_rx_frames.load()),
                static_cast<long long>(g_tx_frames.load()));
    return 0;
}