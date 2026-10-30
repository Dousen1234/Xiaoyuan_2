/**
 * @file walk_control_V1.cpp
 * @brief 四腿机器人下位机控制程序：
 *        ① 关节级键盘命令（move/goto/pos/estop）；
 *        ② 上位机 10Hz 轨迹流接收（虚拟电机 CAN 协议，与钛虎同构）。
 *
 * 【上位机 ⇄ 本机（下位机）协议】（协议全文见 docs/step_protocol.pdf）：
 *   物理层：CAN 2.0A 标准帧，1 Mbps，上位机 CAN 卡接入本机 can0 总线。
 *   帧映射：把本机抽象为 4 个「虚拟电机」，帧 ID 0x141~0x144 ↔ 腿 1~4。
 *   上位机 → 本机（ID = 0x141~0x144）：
 *     [0x1E][J1][J2][J3][J4 各 2 字节小端 int16，1 LSB = 0.01°]（DLC=8）
 *       —— 整腿目标帧：关节端绝对角（度），10Hz 周期发送，收到即执行；
 *     [0x02]（DLC=1）—— 退出流模式，电机保持当前目标；
 *     [0x41]（DLC=1）—— 状态请求，本机立即回同 ID 8 字节反馈帧。
 *   本机 → 上位机：
 *     ID 0x141~0x144（DLC=8）[0x41 回复：J1~J4 各 2 字节 0.01°/LSB]；
 *     ID 0x161（DLC=1）心跳，200ms 周期：bit0=流模式，bit1=样点在线。
 *   保护：样点越限丢弃；流模式 1s 无新样点自动退出（电机保持）；
 *         estop/quit 抱闸；move/goto 在流模式中暂缓。
 *
 * 命令一览（运行后输入，help 可随时查看）：
 *   move <腿1-4> <J1-J4> <cw|ccw> <角度°> [时长s]   相对转动（例：move 1 J1 cw 90）
 *   goto <腿1-4> <J1-J4> <角度°> [时长s]            转到绝对关节角（例：goto 1 J2 45）
 *   pos  [腿1-4]                                     查询关节当前位置（缺省=全部腿）
 *   stat                                             流模式统计（各腿收帧/丢弃）
 *   estop                                            急停：全部电机立即抱闸
 *   quit                                             退出（全部电机抱闸后退出）
 *
 * ============================= 单位与符号约定 =============================
 *
 * 本 demo 的角度一律按【关节端】计算（区别于 taihu_singleleg_step_xCAN 按
 * 电机输出端）。JointModule 的位置 API 是电机输出端弧度，换算在 demo 层完成：
 *
 *   电机输出端角 = 关节角 × kMotorRadPerJointRad × kJointDirSign
 *
 * 机器人构型：4 条腿，每腿 4 关节 YPPY（腿 i ↔ CAN 通道 can{i-1}）：
 *
 *   | 关节 | CAN ID | 构型 | 电机减速比 | 附加传动            | 关节端总减速比 |
 *   |------|--------|------|-----------|---------------------|----------------|
 *   | J1   | 1      | Y    | 101       | 直驱（无）          | 101            |
 *   | J2   | 2 (P1) | P    | 81        | 锥齿轮 1:2（关节减速）| 81×2 = 162   |
 *   | J3   | 3 (P2) | P    | 81        | 锥齿轮 1:2（关节减速）| 81×2 = 162   |
 *   | J4   | 4      | Y    | 101       | 直驱（无）          | 101            |
 *
 * 符号约定：与 taihu_singleleg_step_xCAN 保持一致——电机输出端正角度 = 逆时针。
 * 因此本 demo 约定：关节端正角度 = 逆时针（ccw），cw = 负角度。
 * 若现场发现某关节因锥齿轮装反而方向相反，把 kJointDirSign 对应元素改为 -1 即可。
 *
 * 软件限位（关节端，度）：
 *   J1: ±90°；J2(P1): ±120°；J3(P2): -30° ~ +180°；J4: ±90°
 *   相对/绝对命令的目标角越界时直接拒绝执行（不下发任何运动指令）。
 *
 * ============================= 线程模型 =============================
 *
 *   主线程：只做键盘输入与命令解析，绝不直接操作电机 CAN；
 *   执行线程：所有面向【真电机】的 CAN 收发（含 estop 抱闸）串行集中
 *     在此线程——流模式 5ms 周期下发；空闲时 50ms 轮询命令队列；
 *   接收线程（hostThread）：独立打开 can0 的第二个 socket（SocketCAN
 *     会把总线上每一帧复制给所有 raw socket），只处理上位机的虚拟
 *     电机帧（0x141~0x144），并承担心跳（0x161，200ms）与 0x41 回复。
 *   这样既避免多线程共享 SocketCanInterface 的并发问题，又把上位机
 *   协议与电机控制完全解耦。
 *
 * 命令执行期间其它关节保持当前实际位置（每 tick 重发），到位后不抱闸、
 * 由电机内部位置环持续保持（腿不塌）；estop / quit 才发 0x02 抱闸。
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "taihu/can_interface.h"
#include "taihu/config.h"
#include "taihu/joint_module.h"
#include "taihu/socket_can.h"
#include "taihu/taihu_tools.h"

namespace {

// config.h 的公共常量引入本命名空间（kLoopPeriodMs / kSettleThresholdRad /
// kSettleTimeoutMs / kPollPeriodMs / kDegToRad）
using taihu::kLoopPeriodMs;
using taihu::kSettleThresholdRad;
using taihu::kSettleTimeoutMs;
using taihu::kPollPeriodMs;
using taihu::kDegToRad;

// =====================================================================
// 机器人构型常量（4 腿 YPPY）
// =====================================================================

constexpr int    kLegCount = 4;     ///< 腿数（腿 1~4 ↔ can0~can3）
constexpr int    kJointCount = 4;   ///< 每腿关节数（J1~J4 ↔ 电机 ID 1~4）

/// 四个电机的 CAN ID（J1~J4 顺序）
const uint32_t kMotorIds[kJointCount] = {1, 2, 3, 4};

/// 关节端总减速比（速度换算用：转子 → 关节）
///   J1/J4 直驱 = 电机减速比 101；J2/J3 = 电机 81 × 锥齿轮 2 = 162
const double kJointGearRatios[kJointCount] = {101.0, 162.0, 162.0, 101.0};

/// 位置换算系数：电机输出端角 = 关节角 × 该系数
///   Y 直驱 = 1；P 锥齿轮 1:2（电机端转 2、关节转 1）= 2
const double kMotorRadPerJointRad[kJointCount] = {1.0, 2.0, 2.0, 1.0};

/// 关节方向符号（+1 = 关节正方向对应电机端正方向）。
/// 默认与 xCAN demo 的电机正方向一致（正 = 逆时针）；现场标定后按需改 -1。
const double kJointDirSign[kJointCount] = {1.0, 1.0, 1.0, 1.0};

/// 软件限位（关节端，度）：{min, max}
const double kJointLimitDeg[kJointCount][2] = {
    {-90.0,  90.0},   ///< J1（Y）
    {-120.0, 120.0},  ///< J2（P1，CAN ID 2）
    {-30.0,  180.0},  ///< J3（P2，CAN ID 3）
    {-90.0,  90.0},   ///< J4（Y）
};

// —— 位置环增益（与 taihu_singleleg_step_xCAN 一致） ——
constexpr int32_t kPositionKp = 4000;
constexpr int32_t kPositionKd = 60;

// =====================================================================
// 上位机 ⇄ 下位机 通讯协议常量（协议全文见 docs/step_protocol.pdf）
//
// 设计原则：与钛虎电机协议【同构】——把本机的 4 条腿抽象为 4 个
// 「虚拟电机」：上位机发来的 0x1E/0x02/0x41 帧与控制真电机的帧格式
// 同一套思路（功能码 + 小端定点数据），只是把帧 ID 映射到「腿号」
//（每条总线一腿）。上位机的发送代码与控制真实电机的风格一致。
//
//   上位机 → 本机：功能码 0x1E（整腿目标位置，DLC=8，J1~J4 各 2 字节
//                  小端 int16，定点 1 LSB = 0.01°，关节端绝对角）
//                  功能码 0x02（停止/退出流模式，1 字节）
//                  功能码 0x41（状态请求 → 本机回 8 字节反馈）
//   本机 → 上位机：功能码 0x41 回复（J1~J4 目标角，0.01°/LSB）
//                  心跳 0x161（1 字节状态位，200ms 周期）
//
// 帧 ID 映射（标准帧；0x141~0x144 避开每条总线真电机 ID 1~4）：
//   0x141~0x144 ↔ 腿 1~4 的命令/反馈帧
// =====================================================================

/// 虚拟电机帧 ID 基址：0x141~0x144 = 腿 1~4
constexpr uint16_t kVirtBaseId = 0x141;
/// 腿号 → 虚拟电机帧 ID
inline uint16_t virtId(int leg) { return kVirtBaseId + (leg - 1); }

/// 心跳帧 ID（0x161，DLC=1 状态字节）
constexpr uint16_t kHeartbeatId = kVirtBaseId + 0x20;
/// 心跳发送周期（ms）
constexpr int      kHeartbeatMs  = 200;

/// 每腿样点缓冲容量（环形，免锁单生产者-单消费者）
constexpr int      kStreamBufLen  = 8;

// —— 轨迹与执行参数 ——
constexpr double kDefaultMoveSec = 2.0;   ///< 默认轨迹时长 (s)
constexpr double kMinMoveSec     = 0.1;   ///< 最短轨迹时长 (s)
constexpr double kMaxMoveSec     = 60.0;  ///< 最长轨迹时长 (s)
constexpr double kMaxAngleDeg    = 360.0; ///< 单命令最大角度 (°)

/// 到位确认阈值（关节端 rad）
constexpr double kSettleRad = taihu::kSettleThresholdRad;
/// 执行线程空闲轮询周期（也决定空闲时 estop 的响应上限）(ms)
constexpr int kIdlePollMs = 50;

const char* kJointNames[kJointCount] = {"J1", "J2", "J3", "J4"};

// =====================================================================
// 单位换算（关节端 ⇄ 电机输出端）
// =====================================================================

/// 关节端弧度 → 电机输出端弧度
double jointToMotorRad(int joint, double joint_rad) {
    return joint_rad * kMotorRadPerJointRad[joint] * kJointDirSign[joint];
}

/// 电机输出端弧度 → 关节端弧度
double motorToJointRad(int joint, double motor_rad) {
    return motor_rad / (kMotorRadPerJointRad[joint] * kJointDirSign[joint]);
}

/// 五次多项式平滑函数（quintic smoothstep）：s(0)=0、s(1)=1，
/// 一阶、二阶导数在两端均为 0 —— 位置/速度/加速度连续，起停无冲击。
double quinticSmoothstep(double t) {
    return 6.0 * t * t * t * t * t - 15.0 * t * t * t * t + 10.0 * t * t * t;
}

// =====================================================================
// 上位机 ⇄ 下位机 CAN 帧编解码（小端定点，0.01°/LSB）
// =====================================================================

/// 16 位小端写（d[0] 为最低字节）。
inline void put_i16le(uint8_t* d, int16_t v) {
    d[0] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);
    d[1] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
}

/// 16 位小端读（符号扩展）。
inline int16_t get_i16le(const uint8_t* d) {
    return static_cast<int16_t>(static_cast<uint16_t>(d[0]) |
                                (static_cast<uint16_t>(d[1]) << 8));
}

// =====================================================================
// 一条腿：CAN 接口 + 四个电机
// =====================================================================

struct Leg {
    int  index = 0;               ///< 腿号 1~4
    std::string ifname;           ///< CAN 接口名（"can0"~"can3"）
    bool enabled = false;         ///< 是否成功打开并初始化

    taihu::SocketCanInterface can;                                  ///< 该腿独立总线
    std::unique_ptr<taihu::JointModule> motors[kJointCount];        ///< J1~J4
};

/// 一条待执行的命令
struct Command {
    enum class Type { Move, Goto, Pos } type;
    int    leg = 0;        ///< 腿号 1~4（Pos 且 leg=0 表示全部腿）
    int    joint = 0;      ///< 关节下标 0~3
    bool   relative = true;///< Move=true（相对），Goto=false（绝对）
    double angle_deg = 0.0;///< 角度（相对命令已含符号：cw 为负 / ccw 为正）
    double duration_s = kDefaultMoveSec; ///< 轨迹时长
};

// =====================================================================
// 样点流数据结构（接收线程 → 执行线程，单生产者-单消费者）
// =====================================================================

/// 单个轨迹样点：一条腿在一个时刻的 J1~J4 目标角（关节端 rad）
struct TrajSample {
    double   joint_rad[kJointCount] = {0, 0, 0, 0};  ///< J1~J4 关节端 rad
};

/// 一条腿的样点环形缓冲（免锁：单读单写，head/tail 各自原子）
struct TrajBuffer {
    std::array<TrajSample, kStreamBufLen> buf;
    std::atomic<size_t> head{0};       ///< 写指针（仅接收线程修改）
    std::atomic<size_t> tail{0};       ///< 读指针（仅执行线程修改）
    std::atomic<uint16_t> lost{0};     ///< 累计丢弃样点数
    std::atomic<int32_t> rx_frames{0}; ///< 累计接收帧数（诊断用）

    /// 写入一个样点（缓冲满则丢弃并计数）。
    void push(const TrajSample& s) {
        const size_t h = head.load(std::memory_order_relaxed);
        const size_t next = (h + 1) % kStreamBufLen;
        if (next == tail.load(std::memory_order_acquire)) {
            lost.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        buf[h] = s;
        head.store(next, std::memory_order_release);
        rx_frames.fetch_add(1, std::memory_order_relaxed);
    }
    /// 读取一个样点（空返回 false）。
    bool pop(TrajSample& s) {
        const size_t t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire)) return false;
        s = buf[t];
        tail.store((t + 1) % kStreamBufLen, std::memory_order_release);
        return true;
    }
    /// 当前样点数。
    size_t count() const {
        const size_t h = head.load(std::memory_order_acquire);
        const size_t t = tail.load(std::memory_order_acquire);
        return (h + kStreamBufLen - t) % kStreamBufLen;
    }
    void reset() {
        head.store(0); tail.store(0); lost.store(0); rx_frames.store(0);
    }
};

// =====================================================================
// 全局状态（主线程 / 执行线程共享）
// =====================================================================

Leg g_legs[kLegCount];                       ///< 四条腿（下标 = 腿号-1）
std::queue<Command> g_cmd_queue;             ///< 命令队列
std::mutex         g_cmd_mutex;
std::condition_variable g_cmd_cv;

std::atomic<bool> g_running{true};           ///< 执行线程运行标志
std::atomic<bool> g_estop{false};            ///< 急停标志（轨迹循环内每 tick 检查）

// —— 样点流共享状态（接收线程生产，执行线程消费） ——
TrajBuffer  g_traj[kLegCount];               ///< 每腿一条样点流缓冲
std::atomic<bool>  g_stream_armed{false};    ///< 流模式激活（收到首帧 0x1E 即置位）
std::atomic<bool>  g_stream_online{false};   ///< 最近 1s 内收到过样点（供心跳上报）
std::atomic<int64_t> g_stream_last_rx_ms{0}; ///< 最近一次收到样点的时刻（ms，单调钟）

// =====================================================================
// 执行线程内的原语（全部 CAN 收发集中在此线程）
// =====================================================================

/// 读取一条腿四个关节的当前角度（关节端 rad，0x41 三合一读）。
/// @return 全部读取成功返回 true；任一失败返回 false（电机离线/总线异常）。
bool readLegJointRad(const Leg& leg, double out[kJointCount]) {
    for (int j = 0; j < kJointCount; ++j) {
        int32_t cur_ma = 0;
        double vel = 0.0, motor_pos = 0.0;
        if (!leg.motors[j]->readCurrentVelocityPosition(cur_ma, vel, motor_pos)) {
            std::printf("[腿%d][%s][错误] 读取状态失败（电机无响应？CAN ID %u）\n",
                        leg.index, kJointNames[j], kMotorIds[j]);
            return false;
        }
        out[j] = motorToJointRad(j, motor_pos);
    }
    return true;
}

/// 所有已启用腿的全部电机立即停止并抱闸（0x02）。
void stopAllLegs() {
    for (int i = 0; i < kLegCount; ++i) {
        if (!g_legs[i].enabled) continue;
        for (int j = 0; j < kJointCount; ++j) g_legs[i].motors[j]->stop();
    }
}

/// 对一条腿下发四个关节目标位置（关节端 rad）。
void sendLegTargets(const Leg& leg, const double joint_rad[kJointCount]) {
    for (int j = 0; j < kJointCount; ++j)
        leg.motors[j]->setTargetPosition(jointToMotorRad(j, joint_rad[j]));
}

/// 软件限位检查（关节端度）。
bool withinLimits(int joint, double joint_deg) {
    return joint_deg >= kJointLimitDeg[joint][0] - 1e-9 &&
           joint_deg <= kJointLimitDeg[joint][1] + 1e-9;
}

/// 整条样点四个关节的软件限位检查（关节端 rad）。
bool withinLimitsJointAll(const double joint_rad[kJointCount]) {
    for (int j = 0; j < kJointCount; ++j) {
        const double deg = joint_rad[j] / kDegToRad;
        if (!withinLimits(j, deg)) return false;
    }
    return true;
}

/// 执行一条运动命令（Move / Goto）。返回前打印执行结果。
/// @return 成功到位返回 true；拒绝/中止/失败返回 false。
bool executeMove(const Command& cmd) {
    Leg& leg = g_legs[cmd.leg - 1];
    const int j = cmd.joint;

    // 1. 读取当前四个关节角度（同时校验电机在线）
    double cur[kJointCount];
    if (!readLegJointRad(leg, cur)) return false;

    // 2. 计算目标关节角（弧度）
    const double cur_deg = cur[j] / kDegToRad;
    double target_deg;
    if (cmd.relative) target_deg = cur_deg + cmd.angle_deg;
    else              target_deg = cmd.angle_deg;

    // 3. 软件限位检查：越界直接拒绝，不下发任何运动指令
    if (!withinLimits(j, target_deg)) {
        std::printf("[腿%d][%s][拒绝] 目标 %.1f° 超出限位 [%.0f°, %.0f°]（当前 %.1f°）\n",
                    leg.index, kJointNames[j], target_deg,
                    kJointLimitDeg[j][0], kJointLimitDeg[j][1], cur_deg);
        return false;
    }

    // 4. 轨迹：指定关节 从当前 → 目标，其余关节保持当前位置
    const double target_rad = target_deg * kDegToRad;
    const double q0[kJointCount] = {cur[0], cur[1], cur[2], cur[3]};

    std::printf("[腿%d][%s] %s %.1f° → 目标 %.1f°（限位 [%.0f°, %.0f°]），"
                "轨迹 %.1fs，其余关节保持\n",
                leg.index, kJointNames[j],
                cmd.relative ? "相对转动" : "绝对转角",
                cmd.angle_deg, target_deg,
                kJointLimitDeg[j][0], kJointLimitDeg[j][1],
                cmd.duration_s);

    // 5. 五次多项式轨迹逐点下发（kLoopPeriodMs 周期）
    const auto t0 = std::chrono::steady_clock::now();
    while (g_running.load()) {
        const double t = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - t0).count();
        const double s = quinticSmoothstep(std::min(t / cmd.duration_s, 1.0));

        double q[kJointCount];
        for (int k = 0; k < kJointCount; ++k)
            q[k] = q0[k] + (k == j ? (target_rad - q0[j]) * s : 0.0);
        sendLegTargets(leg, q);

        if (t >= cmd.duration_s) break;

        // 急停：立即跳出（抱闸在循环外统一处理）
        if (g_estop.load()) {
            std::printf("[腿%d][%s][中止] 收到急停，轨迹中止\n",
                        leg.index, kJointNames[j]);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
    }

    // 6. 到位确认：轮询 0x41，误差 < kSettleRad（关节端）且稳定即成功
    const auto settle_start = std::chrono::steady_clock::now();
    while (g_running.load() && !g_estop.load()) {
        double now[kJointCount];
        if (readLegJointRad(leg, now)) {
            if (std::fabs(now[j] - target_rad) < kSettleRad) {
                std::printf("[腿%d][%s] 到位确认：目标 %.3f rad，实际 %.3f rad，"
                            "误差 %.3f rad ✓\n",
                            leg.index, kJointNames[j], target_rad, now[j],
                            std::fabs(now[j] - target_rad));
                return true;
            }
        }
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - settle_start).count();
        if (elapsed_ms > kSettleTimeoutMs) {
            std::printf("[腿%d][%s][警告] 到位超时（目标 %.3f rad），电机保持位置环\n",
                        leg.index, kJointNames[j], target_rad);
            return false;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(kPollPeriodMs)));
    }
    return false;
}

/// 执行位置查询命令（Pos）。
void executePos(const Command& cmd) {
    const int leg_lo = (cmd.leg == 0) ? 1 : cmd.leg;
    const int leg_hi = (cmd.leg == 0) ? kLegCount : cmd.leg;
    for (int i = leg_lo; i <= leg_hi; ++i) {
        const Leg& leg = g_legs[i - 1];
        if (!leg.enabled) {
            std::printf("[腿%d] 未启用（CAN %s 打开失败）\n", i, leg.ifname.c_str());
            continue;
        }
        double pos[kJointCount];
        if (!readLegJointRad(leg, pos)) continue;
        std::printf("[腿%d] %s：", i, leg.ifname.c_str());
        for (int j = 0; j < kJointCount; ++j)
            std::printf("%s %8.2f°", kJointNames[j], pos[j] / taihu::kDegToRad);
        std::printf("\n");
    }
}

/// 执行线程主循环：所有面向电机的 CAN 收发集中在此线程。
/// 流模式期间：5ms 周期执行 streamTick；键盘命令（pos/estop）在
/// 流模式中仍可响应（move/goto 在流模式中提示暂缓）。
void streamTick();   // 前向声明（定义在本文件后面）

void executorThread() {
    while (g_running.load()) {
        // 急停（优先级最高）：空闲与命令间隙都能在 50ms 内响应
        if (g_estop.load()) {
            stopAllLegs();
            g_estop.store(false);
            g_stream_armed.store(false);          // 流模式一并退出
            std::printf("[急停] 全部电机已停止并抱闸（0x02）\n");
            continue;
        }

        // 流模式：消费样点缓冲并下发（5ms 周期）
        if (g_stream_armed.load()) {
            streamTick();
            // 流模式中只放行 pos/estop 类即时查询，运动命令暂缓
            if (!g_cmd_queue.empty()) {
                Command cmd;
                {
                    std::lock_guard<std::mutex> lock(g_cmd_mutex);
                    if (!g_cmd_queue.empty()) {
                        cmd = g_cmd_queue.front();
                        g_cmd_queue.pop();
                    } else cmd.type = Command::Type::Pos;  // 空占位
                }
                if (cmd.type == Command::Type::Pos) executePos(cmd);
                else if (cmd.type == Command::Type::Move ||
                         cmd.type == Command::Type::Goto) {
                    std::printf("[流] 流模式进行中，move/goto 暂缓"
                                "（上位机停止发送或发 0x02 退出流模式）\n");
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
            continue;
        }

        // 取命令（50ms 超时轮询，保证急停标志被及时检查）
        Command cmd;
        {
            std::unique_lock<std::mutex> lock(g_cmd_mutex);
            g_cmd_cv.wait_for(lock, std::chrono::milliseconds(kIdlePollMs),
                              [] { return !g_cmd_queue.empty() || g_estop.load(); });
            if (g_cmd_queue.empty()) continue;
            cmd = g_cmd_queue.front();
            g_cmd_queue.pop();
        }

        if (cmd.type == Command::Type::Move || cmd.type == Command::Type::Goto)
            executeMove(cmd);
        else if (cmd.type == Command::Type::Pos)
            executePos(cmd);
    }
}

// =====================================================================
// 样点流执行（流模式：10Hz 接收，执行线程 5ms 周期下发）
// =====================================================================

/// 流模式单步：消费样点缓冲，向四条腿下发目标位置。
/// 策略（详见协议文档“插值策略”一节）：
///   - 上位机 10Hz 下发绝对关节角样点，本机收到即转存；
///   - 执行线程 5ms 周期把「最新样点」作为目标重发——电机固件内
///     位置环（KP=4000/KD=60）会平滑追踪 10Hz 阶梯目标，
///     样点间的过渡由电机 PD 完成，不引入额外软件插值；
///   - 样点越限 → 丢弃该帧并计数（电机保持上一个目标）；
///   - 断流超过 1s（协议规定）→ 自动退出流模式，电机保持（不抱闸）。
/// 执行线程维护的关节角反馈缓存（rad，关节端）：0x41 回复的数据源。
/// streamTick 每次下发后更新。
std::atomic<double> g_fb_cache[kLegCount][kJointCount];

void streamTick() {
    static double q_target[kLegCount][kJointCount] = {{0}};
    static bool   have_target = false;

    // 1. 消费各腿缓冲：只保留最新样点（覆盖式）
    for (int i = 0; i < kLegCount; ++i) {
        TrajSample s;
        while (g_traj[i].pop(s)) {
            if (!withinLimitsJointAll(s.joint_rad)) {
                std::printf("[流][腿%d][拒绝] 样点越限，已丢弃\n", i + 1);
                continue;
            }
            for (int j = 0; j < kJointCount; ++j)
                q_target[i][j] = s.joint_rad[j];
            have_target = true;
        }
    }

    // 2. 下发（5ms 周期重发，电机位置环保持；断流检测在心跳线程）
    if (!have_target) return;
    for (int i = 0; i < kLegCount; ++i) {
        if (!g_legs[i].enabled) continue;
        sendLegTargets(g_legs[i], q_target[i]);
        for (int j = 0; j < kJointCount; ++j)
            g_fb_cache[i][j].store(q_target[i][j]);
    }
}

// =====================================================================
// 上位机命令接收线程（独立 CAN 接口，只收虚拟电机帧 + 发心跳/回复）
// =====================================================================

/// 判断帧是否来自上位机的虚拟电机 ID（0x141~0x144）。
inline bool isVirtLegFrame(uint16_t id) {
    return id >= kVirtBaseId && id < kVirtBaseId + kLegCount;
}

/// 将浮点值截断到 int16 范围（定点编码用）。
inline int16_t clamp_s16(double v) {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(std::lround(v));
}

/// 单调毫秒时间戳（steady_clock）。
inline int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// 构造并发送一帧（接收线程专用 CAN 接口）。
void hostSend(taihu::SocketCanInterface& can, uint16_t id, uint8_t dlc,
              const uint8_t data[8]) {
    taihu::CanFrame f;
    f.id = id;
    f.dlc = dlc;
    for (int i = 0; i < 8; ++i) f.data[i] = data[i];
    can.send(f);
}

/// 回复 0x41 状态请求帧：回发该腿最近下发的目标关节角。
/// 编码：J1~J4 各 2 字节小端 int16，定点 1 LSB = 0.01°（±327.67°）。
void replyStatusRequest(taihu::SocketCanInterface& can, int leg) {
    uint8_t data[8] = {0};
    for (int j = 0; j < kJointCount; ++j) {
        const double deg = g_fb_cache[leg - 1][j].load() / kDegToRad;
        put_i16le(&data[j * 2], clamp_s16(deg * 100.0));  // 0.01°/LSB
    }
    hostSend(can, virtId(leg), 8, data);
}

/// 接收线程主函数：监听一条独立 CAN 接口，解析上位机的虚拟电机帧。
///
/// 帧格式（与钛虎电机协议同构，详见 docs/step_protocol.pdf）：
///   上位机 → 本机（标准帧，ID = 0x141~0x144 = 腿 1~4）：
///     [0x1E][J1][J2][J3][J4 各 2 字节小端 int16，定点 1 LSB = 0.01°]
///       DLC=8 整腿帧：一帧携带一腿四关节目标角（关节端度）
///     [0x02]                        DLC=1：退出流模式（电机保持）
///     [0x41]                        DLC=1：状态请求 → 本机回 8 字节
///   本机 → 上位机：
///     ID 0x161（DLC=1 状态字节）200ms 周期心跳
///     ID 0x141~0x144（DLC=8）0x41 回复：J1~J4 各 2 字节 0.01°/LSB
///
/// 断流保护：流模式下 1s 无新样点 → 自动退出流模式并告警。
void hostThread(const char* ifname) {
    taihu::SocketCanInterface can;
    if (!can.open(ifname, 0)) {
        std::printf("[协议][错误] 打开 %s 失败，上位机链路不可用\n", ifname);
        return;
    }
    std::printf("[协议] 上位机链路就绪：%s（监听虚拟电机 ID 0x%X~0x%X，"
                "退出流模式发 [0x02]，状态请求发 [0x41]）\n",
                ifname, kVirtBaseId, kVirtBaseId + kLegCount - 1);

    int64_t last_heartbeat_ms = nowMs();

    while (g_running.load()) {
        taihu::CanFrame rx;
        if (!can.receive(rx, /*timeout_ms=*/20)) {
            // 心跳发送（200ms 周期，ID 0x161，1 字节状态）
            if (nowMs() - last_heartbeat_ms >= kHeartbeatMs) {
                uint8_t d[1];
                d[0] = static_cast<uint8_t>(
                    (g_stream_armed.load() ? 0x01 : 0x00) |   // bit0：流模式中
                    (g_stream_online.load() ? 0x02 : 0x00));   // bit1：样点在线
                hostSend(can, kHeartbeatId, 1, d);
                last_heartbeat_ms = nowMs();
            }
            // 断流退出：流模式中 1s 无新样点
            if (g_stream_armed.load() &&
                nowMs() - g_stream_last_rx_ms.load() > 1000) {
                g_stream_armed.store(false);
                g_stream_online.store(false);
                std::printf("[流][警告] 超过 1s 未收到新样点，自动退出流模式"
                            "（电机保持当前目标）\n");
            }
            continue;
        }

        // 只处理上位机虚拟电机 ID 的标准帧
        if (rx.is_extended || !isVirtLegFrame(static_cast<uint16_t>(rx.id)))
            continue;
        const int leg = static_cast<int>(rx.id) - kVirtBaseId + 1;

        if (rx.dlc == 1 && rx.data[0] == 0x02) {
            // 退出流模式（电机保持当前角，不抱闸）
            if (g_stream_armed.load()) {
                g_stream_armed.store(false);
                g_stream_online.store(false);
                std::printf("[流] 收到 0x02：退出流模式，电机保持当前目标\n");
            }
            continue;
        }

        if (rx.dlc == 1 && rx.data[0] == 0x41) {
            replyStatusRequest(can, leg);
            continue;
        }

        if (rx.dlc == 8 && rx.data[0] == 0x1E) {
            // 整腿目标帧：J1~J4 各 2 字节小端 int16，1 LSB = 0.01°
            TrajSample s;
            for (int j = 0; j < kJointCount; ++j)
                s.joint_rad[j] = get_i16le(&rx.data[1 + j * 2]) * 0.01 * kDegToRad;
            g_traj[leg - 1].push(s);

            if (!g_stream_armed.load()) {
                g_stream_armed.store(true);
                std::printf("[流] 收到首帧 0x1E：进入流模式（腿%d）\n", leg);
            }
            g_stream_online.store(true);
            g_stream_last_rx_ms.store(nowMs());
        }
    }
    can.close();
}

// =====================================================================
// 命令解析（主线程）
// =====================================================================

/// 去掉行尾换行与回车。
void chomp(char* s) {
    size_t n = std::strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) s[--n] = '\0';
}

/// 解析腿号（1~4），成功返回 true。
bool parseLeg(const char* tok, int& leg) {
    if (tok == nullptr) return false;
    char* end = nullptr;
    const long v = std::strtol(tok, &end, 10);
    if (end == tok || *end != '\0' || v < 1 || v > kLegCount) return false;
    leg = static_cast<int>(v);
    return true;
}

/// 解析关节名（J1~J4，不分大小写），返回下标 0~3；失败返回 -1。
int parseJoint(const char* tok) {
    if (tok == nullptr || std::strlen(tok) != 2) return -1;
    if (tok[0] != 'J' && tok[0] != 'j') return -1;
    const int v = tok[1] - '1';
    return (v >= 0 && v < kJointCount) ? v : -1;
}

/// 解析浮点数（附带范围检查），失败返回 false。
bool parseDouble(const char* tok, double lo, double hi, double& out) {
    if (tok == nullptr) return false;
    char* end = nullptr;
    const double v = std::strtod(tok, &end);
    if (end == tok || *end != '\0') return false;
    if (v < lo || v > hi) return false;
    out = v;
    return true;
}

/// 解析一行命令，成功返回 true（cmd 为输出）。
bool parseCommand(char* line, Command& cmd) {
    // strtok 线程不安全，但只在主线程使用（demo 级别可接受）
    char* save = nullptr;
    const char* verb = strtok_r(line, " \t", &save);
    if (verb == nullptr) return false;

    if (std::strcmp(verb, "move") == 0 || std::strcmp(verb, "goto") == 0) {
        const bool relative = (std::strcmp(verb, "move") == 0);

        int leg = 0;
        if (!parseLeg(strtok_r(nullptr, " \t", &save), leg)) {
            std::printf("[格式] 腿号需为 1~4\n"); return false;
        }
        const int joint = parseJoint(strtok_r(nullptr, " \t", &save));
        if (joint < 0) {
            std::printf("[格式] 关节需为 J1~J4\n"); return false;
        }

        // move 带 cw/ccw 方向词；goto 直接跟带符号角度
        double angle_deg = 0.0;
        if (relative) {
            const char* dir = strtok_r(nullptr, " \t", &save);
            if (dir == nullptr ||
                (std::strcmp(dir, "cw") != 0 && std::strcmp(dir, "ccw") != 0)) {
                std::printf("[格式] 方向需为 cw（顺时针）或 ccw（逆时针）\n");
                return false;
            }
            if (!parseDouble(strtok_r(nullptr, " \t", &save), 0.1, kMaxAngleDeg,
                             angle_deg)) {
                std::printf("[格式] 角度需为 0.1~%.0f°\n", kMaxAngleDeg);
                return false;
            }
            if (std::strcmp(dir, "cw") == 0) angle_deg = -angle_deg; // cw = 负
        } else {
            if (!parseDouble(strtok_r(nullptr, " \t", &save),
                             -kMaxAngleDeg, kMaxAngleDeg, angle_deg)) {
                std::printf("[格式] 角度需为 ±%.0f° 以内\n", kMaxAngleDeg);
                return false;
            }
        }

        // 可选：轨迹时长
        double duration = kDefaultMoveSec;
        const char* dur_tok = strtok_r(nullptr, " \t", &save);
        if (dur_tok != nullptr &&
            !parseDouble(dur_tok, kMinMoveSec, kMaxMoveSec, duration)) {
            std::printf("[格式] 时长需为 %.1f~%.0fs\n", kMinMoveSec, kMaxMoveSec);
            return false;
        }
        if (strtok_r(nullptr, " \t", &save) != nullptr) {
            std::printf("[格式] 多余参数\n"); return false;
        }

        cmd.type = relative ? Command::Type::Move : Command::Type::Goto;
        cmd.leg = leg;
        cmd.joint = joint;
        cmd.relative = relative;
        cmd.angle_deg = angle_deg;
        cmd.duration_s = duration;
        return true;
    }

    if (std::strcmp(verb, "pos") == 0) {
        int leg = 0;
        const char* leg_tok = strtok_r(nullptr, " \t", &save);
        if (leg_tok != nullptr && !parseLeg(leg_tok, leg)) {
            std::printf("[格式] 腿号需为 1~4（缺省=全部腿）\n"); return false;
        }
        cmd.type = Command::Type::Pos;
        cmd.leg = leg;   // 0 = 全部
        return true;
    }

    std::printf("[格式] 未知命令（help 查看用法）\n");
    return false;
}

/// 打印帮助。
void printHelp() {
    std::printf(
        "命令格式（角度均按【关节端】计算）：\n"
        "  move <腿1-4> <J1-J4> <cw|ccw> <角度°> [时长s]  相对转动，例：move 1 J1 cw 90\n"
        "  goto <腿1-4> <J1-J4> <角度°> [时长s]           绝对关节角，例：goto 1 J2 45\n"
        "  pos  [腿1-4]                                    查询当前位置（缺省=全部腿）\n"
        "  stat                                            流模式统计（各腿收帧/丢弃）\n"
        "  estop                                           急停（全部电机抱闸+退出流模式）\n"
        "  help                                            帮助\n"
        "  quit                                            退出\n"
        "说明：cw=顺时针（负方向），ccw=逆时针（正方向）；默认轨迹时长 %.1fs。\n"
        "      上位机 10Hz 轨迹流：can0 上发 0x141~0x144 帧（协议见 docs/step_protocol.pdf）。\n",
        kDefaultMoveSec);
}

// =====================================================================
// 初始化与主循环
// =====================================================================

/// 打开一条腿的 CAN 总线并初始化四个电机。
/// @param probe 是否读一次状态探测电机在线（打印结果）
/// @return 初始化成功返回 true（个别电机离线仅警告，不判失败）
bool initLeg(int leg_index, bool probe) {
    Leg& leg = g_legs[leg_index - 1];
    leg.index = leg_index;
    char ifbuf[8];
    std::snprintf(ifbuf, sizeof ifbuf, "can%d", leg_index - 1);
    leg.ifname = ifbuf;

    if (!leg.can.open(leg.ifname.c_str(), 0)) {
        std::printf("[腿%d][错误] 打开 %s 失败（接口需先 up："
                    "sudo ./build/kcanctl up %s 1000000）\n",
                    leg_index, leg.ifname.c_str(), leg.ifname.c_str());
        return false;
    }

    // 初始化四个电机：减速比（关节端总减速比，速度换算用）+ 位置环增益
    for (int j = 0; j < kJointCount; ++j) {
        taihu::MotorConfig cfg{"leg" + std::to_string(leg_index) + "_" + kJointNames[j],
                               kMotorIds[j], kJointGearRatios[j], kPositionKp, kPositionKd};
        leg.motors[j] = taihu::initMotor(leg.can, cfg);
        leg.motors[j]->clearError();
    }

    // 探测：逐电机读一次 0x41，确认在线（离线仅警告）
    if (probe) {
        std::printf("[腿%d] %s 电机在线性：", leg_index, leg.ifname.c_str());
        for (int j = 0; j < kJointCount; ++j) {
            int32_t cur_ma = 0;
            double vel = 0.0, pos = 0.0;
            const bool ok =
                leg.motors[j]->readCurrentVelocityPosition(cur_ma, vel, pos);
            std::printf("%s%s", j > 0 ? " " : "", ok ? kJointNames[j] : "×");
        }
        std::printf("（× = 无响应）\n");
    }

    leg.enabled = true;
    return true;
}

} // namespace

int main() {
    std::printf("============================================================\n");
    std::printf("  四腿机器人关节级命令控制 walk_control_V1\n");
    std::printf("============================================================\n");
    std::printf("  构型：4 腿 × 4 关节 YPPY（腿 i ↔ can%d~can%d）\n", 0, kLegCount - 1);
    std::printf("        J1/J4 = Y 直驱（减速比 101）；J2/J3 = P 锥齿轮 1:2（162）\n");
    std::printf("  角度：按【关节端】计算；ccw（逆时针）= 正，与 xCAN 电机方向一致\n");
    std::printf("  限位：J1 ±90°，J2 ±120°，J3 -30°~+180°，J4 ±90°\n");
    std::printf("  例：move 1 J1 cw 90  →  腿1 的 J1 顺时针转 90°（平滑轨迹）\n");
    std::printf("------------------------------------------------------------\n");

    // 1. 选择要启用的腿（直接回车 = 全部；现场只接部分腿时可只输入如 1）
    std::printf("请输入要启用的腿号（1~4，空格分隔，直接回车=全部）：");
    char line[256] = {0};
    if (std::fgets(line, sizeof line, stdin) == nullptr) return -1;
    chomp(line);

    bool want[kLegCount + 1] = {false, false, false, false, false};
    if (line[0] == '\0') {
        for (int i = 1; i <= kLegCount; ++i) want[i] = true;
    } else {
        char* save = nullptr;
        for (char* tok = strtok_r(line, " \t", &save); tok != nullptr;
             tok = strtok_r(nullptr, " \t", &save)) {
            int leg = 0;
            if (!parseLeg(tok, leg)) {
                std::printf("[格式] 腿号需为 1~4，收到 \"%s\"\n", tok);
                return -1;
            }
            want[leg] = true;
        }
    }

    // 2. 初始化各腿（打开 CAN + 电机清错/增益 + 在线探测）
    int enabled_count = 0;
    for (int i = 1; i <= kLegCount; ++i) {
        if (!want[i]) continue;
        if (initLeg(i, /*probe=*/true)) ++enabled_count;
    }
    if (enabled_count == 0) {
        std::printf("[错误] 没有任何腿初始化成功，退出\n");
        return -1;
    }
    std::printf("[信息] %d/%d 条腿就绪（KP=%d KD=%d，轨迹周期 %dms）\n",
                enabled_count, static_cast<int>(
                    std::count(want + 1, want + kLegCount + 1, true)),
                kPositionKp, kPositionKd, taihu::kLoopPeriodMs);
    printHelp();
    std::printf("------------------------------------------------------------\n");

    // 3. 启动执行线程（电机侧 CAN 收发集中于此）与上位机协议线程
    //    （监听 can0 虚拟电机帧 0x141~0x144，回复状态与心跳；协议全文
    //     见 docs/step_protocol.pdf。SocketCAN 允许同接口多 socket，
    //     接收帧被复制到每个 raw socket，不影响执行线程。）
    std::thread executor(executorThread);
    std::thread host(hostThread, "can0");

    // 4. 主循环：读命令 → 解析 → 入队（estop 直接置标志）
    while (true) {
        std::printf("> ");
        std::fflush(stdout);
        if (std::fgets(line, sizeof line, stdin) == nullptr) break; // EOF
        chomp(line);
        if (line[0] == '\0') continue;

        if (std::strcmp(line, "quit") == 0 || std::strcmp(line, "q") == 0) break;
        if (std::strcmp(line, "estop") == 0) {
            g_estop.store(true);
            g_cmd_cv.notify_all();          // 唤醒执行线程立即处理
            continue;
        }
        if (std::strcmp(line, "help") == 0 || std::strcmp(line, "h") == 0) {
            printHelp();
            continue;
        }
        if (std::strcmp(line, "stat") == 0) {
            std::printf("[流状态] 模式=%s 在线=%s\n",
                        g_stream_armed.load() ? "流模式" : "空闲",
                        g_stream_online.load() ? "是" : "否");
            for (int i = 0; i < kLegCount; ++i)
                std::printf("  腿%d：收帧 %u，丢弃 %u，缓冲 %u\n", i + 1,
                            g_traj[i].rx_frames.load(),
                            g_traj[i].lost.load(),
                            static_cast<unsigned>(g_traj[i].count()));
            continue;
        }

        Command cmd;
        if (!parseCommand(line, cmd)) continue;

        // 未启用的腿：直接提示（不入队）
        if (cmd.leg != 0 && !g_legs[cmd.leg - 1].enabled) {
            std::printf("[腿%d] 未启用（%s）\n", cmd.leg,
                        g_legs[cmd.leg - 1].ifname.c_str());
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(g_cmd_mutex);
            g_cmd_queue.push(cmd);
        }
        g_cmd_cv.notify_all();
    }

    // 5. 退出清理：停止线程 → 全部电机抱闸 → 关闭总线
    g_running.store(false);
    g_cmd_cv.notify_all();
    executor.join();
    if (host.joinable()) host.join();
    stopAllLegs();
    for (int i = 0; i < kLegCount; ++i) {
        if (g_legs[i].enabled) g_legs[i].can.close();
    }
    std::printf("[信息] 已退出：全部电机停止抱闸，总线关闭\n");
    return 0;
}
