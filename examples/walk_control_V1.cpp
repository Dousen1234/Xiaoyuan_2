/**
 * @file walk_control_V1.cpp
 * @brief 四腿机器人关节级命令控制 demo：上位机语义命令 → 限位 → 平滑轨迹 → 执行。
 *
 * 使用场景（对应"腿1的J1关节顺时针旋转90度"式的关节级遥操作）：
 *   上位机发送文本命令，指定 腿号 + 关节 + 方向/绝对角 + 角度 + 时长，
 *   程序自动完成：读当前角度 → 计算目标（关节端）→ 软件限位检查 →
 *   五次多项式平滑轨迹 → 5ms 周期下发 → 到位确认 → 打印结果。
 *
 * 命令一览（运行后输入，help 可随时查看）：
 *   move <腿1-4> <J1-J4> <cw|ccw> <角度°> [时长s]   相对转动（例：move 1 J1 cw 90）
 *   goto <腿1-4> <J1-J4> <角度°> [时长s]            转到绝对关节角（例：goto 1 J2 45）
 *   pos  [腿1-4]                                     查询关节当前位置（缺省=全部腿）
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
 *   主线程：只做键盘输入与命令解析，绝不直接操作 CAN；
 *   执行线程：所有 CAN 收发（含 estop 抱闸）串行集中在此线程——
 *     轨迹 5ms 周期循环内检查急停标志，延迟不超过 5ms；
 *     空闲时以 50ms 周期轮询标志与命令队列。
 *   这样避免了多线程共享 SocketCanInterface 的并发发送问题。
 *
 * 命令执行期间其它关节保持当前实际位置（每 tick 重发），到位后不抱闸、
 * 由电机内部位置环持续保持（腿不塌）；estop / quit 才发 0x02 抱闸。
 */

#include <algorithm>
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
// 全局状态（主线程 / 执行线程共享）
// =====================================================================

Leg g_legs[kLegCount];                       ///< 四条腿（下标 = 腿号-1）
std::queue<Command> g_cmd_queue;             ///< 命令队列
std::mutex         g_cmd_mutex;
std::condition_variable g_cmd_cv;

std::atomic<bool> g_running{true};           ///< 执行线程运行标志
std::atomic<bool> g_estop{false};            ///< 急停标志（轨迹循环内每 tick 检查）

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

/// 执行线程主循环：所有 CAN 收发集中在此线程。
void executorThread() {
    while (g_running.load()) {
        // 急停（优先级最高）：空闲与命令间隙都能在 50ms 内响应
        if (g_estop.load()) {
            stopAllLegs();
            g_estop.store(false);
            std::printf("[急停] 全部电机已停止并抱闸（0x02）\n");
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
        "  estop                                           急停（全部电机抱闸）\n"
        "  help                                            帮助\n"
        "  quit                                            退出\n"
        "说明：cw=顺时针（负方向），ccw=逆时针（正方向）；默认轨迹时长 %.1fs。\n",
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

    // 3. 启动执行线程（所有 CAN 收发集中于此）
    std::thread executor(executorThread);

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

    // 5. 退出清理：停止执行线程 → 全部电机抱闸 → 关闭总线
    g_running.store(false);
    g_cmd_cv.notify_all();
    executor.join();
    stopAllLegs();
    for (int i = 0; i < kLegCount; ++i) {
        if (g_legs[i].enabled) g_legs[i].can.close();
    }
    std::printf("[信息] 已退出：全部电机停止抱闸，总线关闭\n");
    return 0;
}
