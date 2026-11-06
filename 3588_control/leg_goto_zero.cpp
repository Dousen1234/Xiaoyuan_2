/**
 * @file leg_goto_zero.cpp
 * @brief 3588 上位机工具：把指定腿臂的 4 个关节用五次多项式平滑送回 0 位
 *
 * 运行角色（与 upper_control 一致）：
 *   上位机 → 打开 3588 自带的板载 CAN（默认 can_mcu）
 *   下位机 → lower_control 打开鲲弘 can5
 *   两者之间一根 CAN 线（H/L + 两端 120Ω），CAN 口名不同所以互不冲突。
 *
 * 工作流程：
 *   1) 交互输入（或 --leg/--time 指定）要回零的腿臂与整条轨迹的运行时间；
 *   2) 读该腿 4 个关节的当前实际角（0x41 反馈）作为轨迹起点；
 *   3) 每个关节按五次多项式  s(u) = 10u³ − 15u⁴ + 6u⁵  从 p₀ 平滑走到 0，
 *      两端速度、加速度均为 0（起停无冲击）；
 *   4) 16 帧全发：选中腿下发插值结果，其余 3 条腿持续下发「启动时刻的实际位置」
 *      原地保持不动，避免下位机断流或跳动；
 *   5) 到位后继续保持 0 位 N 秒（默认 2 s），再发 0x02 退出流模式。
 *
 * 关于软限位（本工具必须自己保证，不能只靠下位机钳位）：
 *   五次多项式的一阶导为  s'(u) = 30u²(1−u)² ≥ 0，即 s(u) 在 [0,1] 上严格单调，
 *   所以 p₀ → 0 的路径不会过冲；只要 p₀ 和 0 都在 [下限, 上限] 内，
 *   路径上每一个中间点也必然在限位内。限位因此是"自动满足"的。
 *   代码里仍然做三重保险：
 *     a) 启动前校验 16 个关节的实际角是否都在限位内（越限直接拒绝，--force 可强跑）；
 *     b) 运行前按「峰值每帧变化 ≤ --max-step 度」反算最短可用时间，太短就拒绝；
 *     c) 运行中每 tick 对下发的 4×16 个目标再做一次限位断言，越限立即中止。
 *
 * 关于关节方向：
 *   本工具【不做任何符号取反】。取反只发生在 upper_control 读 xlsx 轨迹的预处理阶段
 *   （轨迹文件用设计方向，实机第 1/3/4 关节相反，第 2 关节一致）。本工具直接读实机
 *   反馈、直接下发同一坐标系，所以读到的 30° 就按 30° 处理、下发 30°，中间不翻符号。
 *
 * 典型用法：
 *   ./leg_goto_zero                                  # 交互输入腿臂与时间
 *   ./leg_goto_zero --leg A --time 8                 # 命令行指定
 *   ./leg_goto_zero --leg A --time 8 --dry-run       # 只打印计划，不碰 CAN
 *   ./leg_goto_zero --leg A --time 8 --hold 3        # 到位后保持 3 秒再退出
 *
 * 退出码：
 *   0 = 正常完成          1 = 环境/链路错误
 *   2 = 用户取消或参数不合法   3 = 软限位报警中止
 */

#include "proto.h"

#include <taihu/socket_can.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

using namespace x3588;

namespace
{

    // =========================================================================
    // 全局状态
    // =========================================================================

    std::atomic<bool> g_running{true}; ///< 主动运动循环开关
    std::atomic<bool> g_rx_run{true};  ///< 收帧线程开关（收尾阶段还要收反馈，所以单独一个）

    /// 单个关节的上报状态
    struct JointState
    {
        std::atomic<bool> online{false}; ///< 是否收到过反馈
        std::atomic<int> last_proto{0};  ///< 最近一次反馈的协议值
        std::atomic<int64_t> last_ms{0}; ///< 最近一次反馈时刻
    };

    JointState g_joints[kTotalJoints];

    /// 下位机心跳最新值
    std::atomic<uint8_t> g_hb{0};
    std::atomic<int64_t> g_hb_ms{0};

    /// 软限位报警
    std::atomic<bool> g_limit_locked{false};
    std::atomic<int> g_limit_joint{-1};
    std::atomic<int> g_limit_side{0};
    std::atomic<int> g_limit_proto{0};
    std::atomic<int64_t> g_limit_ms{0};

    /// 统计
    std::atomic<int64_t> g_tx_frames{0};
    std::atomic<int64_t> g_rx_frames{0};

    int64_t nowMs()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void onSignal(int) { g_running.store(false); }

    // =========================================================================
    // 参数
    // =========================================================================

    /// 关节端软件限位（度），与 lower_control 的 kJointLimitDeg 严格一致。
    /// 说明：本工具自己也要判限位，所以这里必须内置一份；改限位时两边一起改。
    const double kDefaultLimitDeg[kJointCount][2] = {
        {-120.0, 120.0}, ///< J1 Y 直驱，电机端 ±120°
        {-120.0, 120.0}, ///< J2 P1 锥齿轮 1:2，电机端 ±240°
        {-60.0, 180.0},  ///< J3 P2 锥齿轮 1:2，电机端 −120°~+360°
        {-90.0, 90.0},   ///< J4 Y 直驱，电机端 ±90°
    };

    struct Options
    {
        int leg = -1;        ///< 0..3，-1 = 未指定（交互输入）
        double time_s = 0.0; ///< 轨迹时长，<=0 = 未指定（交互输入）
        std::string can = "can_mcu";
        double hz = 10.0;
        double hold_s = 2.0; ///< 到位后保持时长
        double wait_link_s = 10.0;
        double feedback_timeout_s = 3.0;       ///< 等 16 个关节反馈的最长时间
        double max_step_deg = 2.0;             ///< 允许的最大单帧变化量，0 = 不检查
        std::string start_deg = "60,30,45,30"; ///< 仅 --dry-run 下模拟的起始角
        double limit_deg[kJointCount][2] = {};
        bool auto_start = false;
        bool dry_run = false;
        bool force = false; ///< 越限也强跑
    };

    void printUsage(const char *prog)
    {
        std::printf(
            "用法: %s [选项]\n"
            "\n"
            "回零目标:\n"
            "  --leg <A|B|C|D|1|2|3|4>  要回零的腿臂（不给则交互输入）\n"
            "  --time <秒>              整条轨迹的运行时间，> 0（不给则交互输入）\n"
            "  --hold <秒>              到位后继续发 0 位保持的时间（默认 %.1f）\n"
            "\n"
            "通道与频率:\n"
            "  --can <接口名>           上位机 CAN 接口（默认 %s）\n"
            "  --hz <值>                发送频率 Hz（默认 %.1f，范围 1~100）\n"
            "\n"
            "安全检查:\n"
            "  --limits \"a,b;c,d;e,f;g,h\"   关节端软件限位（度）\n"
            "                               默认 -120,120;-120,120;-60,180;-90,90\n"
            "  --max-step <度>          允许的单帧最大变化量（默认 %.1f；0 = 不检查）\n"
            "                           时间太短、单帧跳变过大时直接拒绝并给出最短时间\n"
            "  --force                  起始角越限 / 时间过短也强制运行（谨慎使用）\n"
            "\n"
            "流程控制:\n"
            "  --wait-link <秒>         链路检测最长等待（默认 %.1f；0 = 跳过检测）\n"
            "  --feedback-timeout <秒>  等全部关节反馈的最长时间（默认 %.1f）\n"
            "  --dry-run                只打印回零计划，不打开 CAN（用 --start-deg 模拟起始角）\n"
            "  --start-deg \"a,b,c,d\"   仅 --dry-run 下模拟选中腿的起始角（默认 %s）\n"
            "  --auto                   不按回车直接开始（脚本化用）\n"
            "  -h, --help               显示本帮助\n"
            "\n"
            "说明:\n"
            "  选中腿的 4 个关节按五次多项式 s(u)=10u³-15u⁴+6u⁵ 从当前角平滑走到 0，\n"
            "  其余 3 条腿持续下发启动时刻的实际位置原地保持（16 帧全发）。\n"
            "  因 s'(u)=30u²(1-u)² >= 0 单调，路径不过冲，起点与 0 都在限位内即全程在限位内。\n"
            "  本工具不做任何关节符号取反（取反只属于读 xlsx 轨迹的预处理阶段）。\n"
            "\n"
            "示例:\n"
            "  %s\n"
            "  %s --leg A --time 8\n"
            "  %s --leg B --time 12 --hold 3 --hz 20\n"
            "  %s --leg A --time 8 --dry-run --start-deg 120,30,90,45\n",
            prog, Options().hold_s, Options().can.c_str(), Options().hz,
            Options().max_step_deg, Options().wait_link_s,
            Options().feedback_timeout_s, Options().start_deg.c_str(),
            prog, prog, prog, prog);
    }

    /// 解析 "a,b;c,d;e,f;g,h" 形式的 4 组限位（度）
    bool parseLimits(const std::string &s, double out[kJointCount][2], std::string &err)
    {
        std::vector<std::string> parts;
        size_t pos = 0;
        for (;;)
        {
            const size_t p = s.find(';', pos);
            if (p == std::string::npos)
            {
                parts.push_back(s.substr(pos));
                break;
            }
            parts.push_back(s.substr(pos, p - pos));
            pos = p + 1;
        }

        if (parts.size() != static_cast<size_t>(kJointCount))
        {
            err = "--limits 需要 4 组「下限,上限」，分号分隔，"
                  "例如 -120,120;-120,120;-60,180;-90,90";
            return false;
        }
        for (int j = 0; j < kJointCount; ++j)
        {
            const size_t c = parts[j].find(',');
            if (c == std::string::npos)
            {
                err = "--limits 第 " + std::to_string(j + 1) + " 组缺少逗号";
                return false;
            }
            const double lo = std::atof(parts[j].substr(0, c).c_str());
            const double hi = std::atof(parts[j].substr(c + 1).c_str());
            if (!(lo < hi))
            {
                err = "--limits 第 " + std::to_string(j + 1) + " 组下限必须小于上限";
                return false;
            }
            out[j][0] = lo;
            out[j][1] = hi;
        }
        return true;
    }

    /// 解析 "A"/"a"/"1" 等形式的腿臂编号
    bool resolveLeg(const std::string &s, int &leg)
    {
        if (s.empty())
            return false;
        if (s.size() == 1)
        {
            const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(s[0])));
            if (c >= 'A' && c <= 'D')
            {
                leg = c - 'A';
                return true;
            }
            if (c >= '1' && c <= '4')
            {
                leg = c - '1';
                return true;
            }
        }
        return false;
    }

    bool parseArgs(int argc, char **argv, Options &o, std::string &err)
    {
        for (int j = 0; j < kJointCount; ++j)
        {
            o.limit_deg[j][0] = kDefaultLimitDeg[j][0];
            o.limit_deg[j][1] = kDefaultLimitDeg[j][1];
        }

        for (int i = 1; i < argc; ++i)
        {
            const std::string a = argv[i];
            auto need = [&](const char *name) -> const char *
            {
                if (i + 1 >= argc)
                {
                    err = std::string("参数 ") + name + " 缺少取值";
                    return nullptr;
                }
                return argv[++i];
            };

            if (a == "-h" || a == "--help")
            {
                printUsage(argv[0]);
                std::exit(0);
            }
            else if (a == "--leg")
            {
                const char *v = need("--leg");
                if (!v)
                    return false;
                if (!resolveLeg(v, o.leg))
                {
                    err = "--leg 只能是 A/B/C/D 或 1/2/3/4";
                    return false;
                }
            }
            else if (a == "--time")
            {
                const char *v = need("--time");
                if (!v)
                    return false;
                o.time_s = std::atof(v);
                if (!(o.time_s > 0.0))
                {
                    err = "--time 必须大于 0";
                    return false;
                }
            }
            else if (a == "--can")
            {
                const char *v = need("--can");
                if (!v)
                    return false;
                o.can = v;
            }
            else if (a == "--hz")
            {
                const char *v = need("--hz");
                if (!v)
                    return false;
                o.hz = std::atof(v);
                if (!(o.hz >= 1.0 && o.hz <= 100.0))
                {
                    err = "--hz 必须在 1~100 之间";
                    return false;
                }
            }
            else if (a == "--hold")
            {
                const char *v = need("--hold");
                if (!v)
                    return false;
                o.hold_s = std::atof(v);
                if (o.hold_s < 0.0)
                {
                    err = "--hold 不能为负";
                    return false;
                }
            }
            else if (a == "--limits")
            {
                const char *v = need("--limits");
                if (!v)
                    return false;
                if (!parseLimits(v, o.limit_deg, err))
                    return false;
            }
            else if (a == "--max-step")
            {
                const char *v = need("--max-step");
                if (!v)
                    return false;
                o.max_step_deg = std::atof(v);
                if (o.max_step_deg < 0.0)
                {
                    err = "--max-step 不能为负";
                    return false;
                }
            }
            else if (a == "--wait-link")
            {
                const char *v = need("--wait-link");
                if (!v)
                    return false;
                o.wait_link_s = std::atof(v);
            }
            else if (a == "--feedback-timeout")
            {
                const char *v = need("--feedback-timeout");
                if (!v)
                    return false;
                o.feedback_timeout_s = std::atof(v);
                if (o.feedback_timeout_s < 0.1)
                {
                    err = "--feedback-timeout 至少 0.1 秒";
                    return false;
                }
            }
            else if (a == "--start-deg")
            {
                const char *v = need("--start-deg");
                if (!v)
                    return false;
                o.start_deg = v;
            }
            else if (a == "--auto")
            {
                o.auto_start = true;
            }
            else if (a == "--dry-run")
            {
                o.dry_run = true;
            }
            else if (a == "--force")
            {
                o.force = true;
            }
            else
            {
                err = "未知参数：" + a;
                return false;
            }
        }
        return true;
    }

    // =========================================================================
    // 行输入小工具
    // =========================================================================

    std::string trimStr(const std::string &s)
    {
        size_t b = 0;
        size_t e = s.size();
        while (b < e && std::isspace(static_cast<unsigned char>(s[b])))
            ++b;
        while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
            --e;
        return s.substr(b, e - b);
    }

    /// 读一行并去掉首尾空白；返回 false 表示 EOF
    bool readLineTrim(std::string &out)
    {
        char line[256] = {0};
        if (std::fgets(line, sizeof line, stdin) == nullptr)
            return false;
        out = trimStr(std::string(line));
        return true;
    }

    // =========================================================================
    // CAN 收发小工具
    // =========================================================================

    void sendFrame(taihu::SocketCanInterface &can, uint16_t id, uint8_t dlc,
                   const uint8_t *data)
    {
        taihu::CanFrame f;
        f.id = id;
        f.dlc = dlc;
        if (data != nullptr)
            for (int i = 0; i < 8; ++i)
                f.data[i] = data[i];
        if (can.send(f))
            g_tx_frames.fetch_add(1);
    }

    /// 收帧线程：解析下位机上报的 0x41 / 0x161 / 0x162
    void rxThread(taihu::SocketCanInterface *can)
    {
        while (g_rx_run.load())
        {
            taihu::CanFrame rx;
            if (!can->receive(rx, 20))
                continue;
            g_rx_frames.fetch_add(1);

            if (rx.is_extended)
                continue;
            const uint16_t id = static_cast<uint16_t>(rx.id & 0x7FF);
            const int64_t t = nowMs();

            // —— ID 1~16：关节实际角反馈 ——
            if (isJointId(id) && rx.dlc == 5 && rx.data[0] == kCmdGetState)
            {
                const int idx = (id - kJointIdBase);
                g_joints[idx].last_proto.store(getI32LE(&rx.data[1]));
                g_joints[idx].online.store(true);
                g_joints[idx].last_ms.store(t);
                continue;
            }

            // —— 0x161：心跳 ——
            if (id == kHeartbeatId && rx.dlc >= 1)
            {
                g_hb.store(rx.data[0]);
                g_hb_ms.store(t);
                if ((rx.data[0] & kHbBitLimitLock) == 0 && g_limit_locked.exchange(false))
                {
                    std::printf("[限位] 下位机心跳显示报警已清除，本地警报状态复位\n");
                    g_limit_joint.store(-1);
                    g_limit_side.store(0);
                    g_limit_proto.store(0);
                }
                continue;
            }

            // —— 0x162：软限位报警 ——
            if (id == kLimitAlarmId && rx.dlc == 8 && rx.data[0] == kLimitAlarmCode)
            {
                g_limit_joint.store(rx.data[1]);
                g_limit_side.store(rx.data[2]);
                g_limit_proto.store(getI32LE(&rx.data[4]));
                g_limit_ms.store(t);
                g_limit_locked.store(true);
                continue;
            }
        }
    }

    /// 下位机心跳是否新鲜（1s 内）
    bool heartbeatFresh() { return nowMs() - g_hb_ms.load() < 1000; }

    /// 关节 i 的反馈是否新鲜（1s 内）
    bool jointFresh(int i)
    {
        return g_joints[i].online.load() &&
               nowMs() - g_joints[i].last_ms.load() < 1000;
    }

    /// 主动点名：向 16 个关节各发一帧 0x41 状态请求
    void pingAll(taihu::SocketCanInterface &can)
    {
        uint8_t d[8] = {kCmdGetState, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < kTotalJoints; ++i)
            sendFrame(can, jointToId(i / kJointCount, i % kJointCount), 1, d);
    }

    // =========================================================================
    // 链路检测
    // =========================================================================

    /// 被动等待下位机心跳 + 主动点名，确认链路可用
    bool checkLink(taihu::SocketCanInterface &can, double wait_s)
    {
        if (wait_s <= 0.0)
            return true;

        std::printf("[2/6] 链路检测：等待下位机心跳（最多 %.0f 秒）...\n", wait_s);
        std::fflush(stdout);

        const int64_t deadline = nowMs() + static_cast<int64_t>(wait_s * 1000.0);
        int64_t last_ping = 0;

        while (nowMs() < deadline && g_running.load())
        {
            if (heartbeatFresh())
            {
                std::printf("      ✓ 收到下位机心跳 0x%03X，链路正常\n", kHeartbeatId);
                return true;
            }
            if (nowMs() - last_ping >= 300)
            {
                last_ping = nowMs();
                pingAll(can);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        if (heartbeatFresh())
            return true;

        std::printf("      ✗ 未收到下位机心跳。\n"
                    "        请确认：\n"
                    "          1) 下位机程序 lower_control 已在另一个终端启动；\n"
                    "          2) 两边的 CAN 线已接好（H/L + 两端 120Ω）；\n"
                    "          3) 下位机监听的口与本机 --can 指定的口在物理上是同一条总线。\n");
        return false;
    }

    // =========================================================================
    // 反馈读取
    // =========================================================================

    /// 等全部 16 个关节的反馈，成功返回 true 并把关节端弧度写入 out。
    /// 未收齐的关节编号写入 missing（0~15）。
    bool readAllJointRad(taihu::SocketCanInterface &can, double timeout_s,
                         double out[kTotalJoints], std::vector<int> &missing)
    {
        const int64_t deadline = nowMs() + static_cast<int64_t>(timeout_s * 1000.0);
        int64_t last_ping = 0;

        for (;;)
        {
            int n_fresh = 0;
            for (int i = 0; i < kTotalJoints; ++i)
                if (jointFresh(i))
                    ++n_fresh;
            if (n_fresh == kTotalJoints)
                break;
            if (nowMs() >= deadline || !g_running.load())
                break;

            if (nowMs() - last_ping >= 300)
            {
                last_ping = nowMs();
                pingAll(can);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        missing.clear();
        for (int i = 0; i < kTotalJoints; ++i)
        {
            if (jointFresh(i))
                out[i] = protoToRad(g_joints[i].last_proto.load());
            else
            {
                out[i] = 0.0;
                missing.push_back(i);
            }
        }
        return missing.empty();
    }

    /// 打印某个关节的简写编号，例如 idx=5 → "B2"
    std::string jointLabel(int idx)
    {
        const int leg = idx / kJointCount;
        const int j = idx % kJointCount;
        std::string s;
        s += legLetter(leg);
        s += static_cast<char>('1' + j);
        return s;
    }

    // =========================================================================
    // 限位校验
    // =========================================================================

    /// 校验 16 个关节的当前角是否都在限位内；返回 false 时 err 里列出全部越限项
    bool validateStartPosition(const double cur[kTotalJoints],
                               const double limit_deg[kJointCount][2],
                               int sel_leg, std::string &err)
    {
        std::string msg;
        int n_sel = 0, n_other = 0;

        for (int leg = 0; leg < kLegCount; ++leg)
        {
            for (int j = 0; j < kJointCount; ++j)
            {
                const int idx = leg * kJointCount + j;
                const double deg = cur[idx] * kRadToDeg;
                const double lo = limit_deg[j][0];
                const double hi = limit_deg[j][1];
                if (deg < lo || deg > hi)
                {
                    char buf[256];
                    std::snprintf(buf, sizeof buf,
                                  "        %s %-6s 当前 %+8.3f°  限位 [%+.1f°, %+.1f°]%s\n",
                                  (leg == sel_leg) ? "【选中腿】" : "           ",
                                  jointLabel(idx).c_str(), deg, lo, hi,
                                  (leg == sel_leg) ? "  ← 回零目标腿" : "");
                    msg += buf;
                    if (leg == sel_leg)
                        ++n_sel;
                    else
                        ++n_other;
                }
            }
        }

        if (msg.empty())
            return true;

        err = "启动位置越界：\n" + msg;
        if (n_sel > 0)
            err += "      选中腿有 " + std::to_string(n_sel) + " 个关节已经超出软限位。\n";
        if (n_other > 0)
            err += "      其他腿有 " + std::to_string(n_other) +
                   " 个关节已经超出软限位（保持帧同样会被拒绝）。\n";
        err += "      处理建议：先用 lower_control 把这些关节手动挪回限位内，或加 --force 强制回零。\n";
        return false;
    }

    // =========================================================================
    // 五次多项式
    // =========================================================================

    /// s(u) = 10u³ − 15u⁴ + 6u⁵
    /// 边界条件 s(0)=0, s(1)=1, s'(0)=s'(1)=s''(0)=s''(1)=0（两端速度、加速度都为 0）
    inline double polyS(double u)
    {
        const double u2 = u * u;
        return u2 * u * (10.0 - 15.0 * u + 6.0 * u2);
    }

    /// ds/du = 30u²(1−u)²  —— 恒 ≥ 0，所以 s 单调，路径不会过冲
    inline double polyDS(double u)
    {
        const double one_minus = 1.0 - u;
        return 30.0 * u * u * one_minus * one_minus;
    }

    /// d²s/du² = 60u − 180u² + 120u³
    inline double polyDDS(double u) { return 60.0 * u - 180.0 * u * u + 120.0 * u * u * u; }

    /// 五次多项式的峰值速度系数：s'(0.5) = 30·0.25·0.25 = 1.875
    const double kPolyPeakDS = 1.875;

    /// 峰值加速度系数：|s''| 在 u = (1 ± 1/√3)/2 处取到 10/√3 ≈ 5.7735
    const double kPolyPeakDDS = polyDDS(0.5 * (1.0 - 1.0 / std::sqrt(3.0)));

    /// 单个关节在某时刻的插值结果
    struct JointTrack
    {
        double p0 = 0.0; ///< 起点（rad）
        double p1 = 0.0; ///< 终点（rad），本工具恒为 0
        double at(double t, double T) const
        {
            if (T <= 0.0)
                return p1;
            if (t <= 0.0)
                return p0;
            if (t >= T)
                return p1;
            const double u = t / T;
            return p0 + (p1 - p0) * polyS(u);
        }
    };

    /// 按「单帧最大变化量」反算最短可用时间（秒）
    double minTimeForStep(const double start[kJointCount], double hz, double max_step_deg)
    {
        if (max_step_deg <= 0.0 || hz <= 0.0)
            return 0.0;
        double max_delta_deg = 0.0; ///< 目标恒为 0，行程就是 |起点|
        for (int j = 0; j < kJointCount; ++j)
            max_delta_deg = std::max(max_delta_deg, std::fabs(start[j] * kRadToDeg));
        // 峰值速度(度/秒) = 行程 · 1.875 / T ；峰值每帧变化 = 峰值速度 / hz
        // 要求 峰值速度/hz ≤ max_step_deg  →  T ≥ 行程 · 1.875 / (max_step_deg · hz)
        return max_delta_deg * kPolyPeakDS / (max_step_deg * hz);
    }

    // =========================================================================
    // 打印
    // =========================================================================

    /// 打印回零计划摘要
    void printPlan(const Options &opt, const double start_rad[kJointCount])
    {
        const int leg = opt.leg;
        const int base = leg * kJointCount;
        const int frames = static_cast<int>(std::lround(opt.time_s * opt.hz)) + 1;

        std::printf("\n┌─ 回零计划 ───────────────────────────────────────────────────\n");
        std::printf("│ 目标腿臂 : %c（关节 CAN ID %d~%d）\n",
                    legLetter(leg), jointToId(leg, 0), jointToId(leg, kJointCount - 1));
        std::printf("│ 轨迹时长 : %.2f 秒\n", opt.time_s);
        std::printf("│ 发送频率 : %.1f Hz → 共约 %d 帧/关节\n", opt.hz, frames);
        std::printf("│ 到位保持 : %.1f 秒（继续发 0 位，随后发 0x02 退出流模式）\n", opt.hold_s);
        std::printf("│ 插值曲线 : s(u) = 10u³ − 15u⁴ + 6u⁵（两端速度/加速度均为 0）\n");
        std::printf("├──────────────────────────────────────────────────────────────\n");

        for (int j = 0; j < kJointCount; ++j)
        {
            const double p0 = start_rad[j];
            const double deg = p0 * kRadToDeg;
            const double travel = std::fabs(deg);
            const double v_peak = travel * kPolyPeakDS / opt.time_s;    ///< 度/秒
            const double step = (opt.hz > 0.0) ? v_peak / opt.hz : 0.0; ///< 度/帧
            std::printf("│ [%s] 当前 %+9.3f° → 目标 +0.000°   行程 %8.3f°   "
                        "峰值 %7.2f°/s（%.3f°/帧）  限位 [%+.1f°, %+.1f°]\n",
                        jointLabel(base + j).c_str(), deg, travel, v_peak, step,
                        opt.limit_deg[j][0], opt.limit_deg[j][1]);
        }

        std::printf("├──────────────────────────────────────────────────────────────\n");
        double max_travel_deg = 0.0;
        for (int j = 0; j < kJointCount; ++j)
            max_travel_deg = std::max(max_travel_deg, std::fabs(start_rad[j] * kRadToDeg));
        if (opt.time_s > 0.0)
        {
            const double a_peak = max_travel_deg * kPolyPeakDDS / (opt.time_s * opt.time_s);
            std::printf("│ 最大行程 %8.3f°  峰值速度 %7.2f°/s  峰值加速度 %8.2f°/s²\n",
                        max_travel_deg,
                        max_travel_deg * kPolyPeakDS / opt.time_s, a_peak);
        }
        const double t_min = minTimeForStep(start_rad, opt.hz, opt.max_step_deg);
        if (opt.max_step_deg > 0.0)
            std::printf("│ 单帧步长 : 限制 ≤ %.2f°/帧 @ %.1f Hz ⇒ 最短可用时间 %.2f 秒\n",
                        opt.max_step_deg, opt.hz, t_min);
        else
            std::printf("│ 单帧步长 : 未限制（--max-step 0）\n");
        std::printf("├──────────────────────────────────────────────────────────────\n");
        std::printf("│ 其余腿臂 : 持续下发启动时刻的实际位置，原地保持不动\n");
        std::printf("│ 软限位   : 五次多项式单调（s'(u)=30u²(1−u)² ≥ 0），路径不过冲，\n");
        std::printf("│            起点与 0 位都在限位内 ⇒ 全程必然在限位内\n");
        std::printf("└──────────────────────────────────────────────────────────────\n");
    }

    /// 运行中打印一次进度（带当前 4 个关节的目标角与实际角）
    void printProgress(double t, double T, const double target[kJointCount],
                       const double cur[kJointCount], int leg)
    {
        std::printf("  t=%6.2f/%.2f s  │", t, T);
        for (int j = 0; j < kJointCount; ++j)
        {
            std::printf("  %s 目标%+7.2f° 实际%+7.2f° │",
                        jointLabel(leg * kJointCount + j).c_str(),
                        target[j] * kRadToDeg, cur[j] * kRadToDeg);
        }
        std::printf("\n");
    }

} // namespace

// =========================================================================
// main
// =========================================================================

int main(int argc, char **argv)
{
    Options opt;
    std::string err;
    if (!parseArgs(argc, argv, opt, err))
    {
        std::printf("[错误] %s\n\n", err.c_str());
        printUsage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::printf("════════════════════════════════════════════════════════════════\n");
    std::printf("  3588 上位机 · 腿臂回零 · 五次多项式 · V2.0 协议\n");
    std::printf("════════════════════════════════════════════════════════════════\n");

    const bool interactive = (::isatty(STDIN_FILENO) != 0);

    // —— [0/6] 腿臂 + 时间 ——
    std::printf("\n[0/6] 回零参数\n");

    if (opt.leg < 0)
    {
        if (!interactive)
        {
            std::printf("[错误] 未指定 --leg，且标准输入不是终端，无法交互输入。\n");
            return 2;
        }
        for (int tries = 0; tries < 5; ++tries)
        {
            std::printf("      请输入要回零的腿臂（A/B/C/D 或 1/2/3/4）> ");
            std::fflush(stdout);
            std::string s;
            if (!readLineTrim(s))
            {
                std::printf("\n已取消。\n");
                return 2;
            }
            if (s == "q" || s == "Q")
            {
                std::printf("已取消。\n");
                return 2;
            }
            if (resolveLeg(s, opt.leg))
                break;
            std::printf("      ✗ 输入无效，请输入 A/B/C/D 或 1/2/3/4。\n");
        }
        if (opt.leg < 0)
        {
            std::printf("[错误] 腿臂编号输入次数过多，退出。\n");
            return 2;
        }
    }
    std::printf("      目标腿臂 : %c（CAN ID %d~%d）\n",
                legLetter(opt.leg), jointToId(opt.leg, 0),
                jointToId(opt.leg, kJointCount - 1));

    if (opt.time_s <= 0.0)
    {
        if (!interactive)
        {
            std::printf("[错误] 未指定 --time，且标准输入不是终端，无法交互输入。\n");
            return 2;
        }
        for (int tries = 0; tries < 5; ++tries)
        {
            std::printf("      请输入整条轨迹的运行时间（秒，> 0）> ");
            std::fflush(stdout);
            std::string s;
            if (!readLineTrim(s))
            {
                std::printf("\n已取消。\n");
                return 2;
            }
            if (s == "q" || s == "Q")
            {
                std::printf("已取消。\n");
                return 2;
            }
            const double v = std::atof(s.c_str());
            if (v > 0.0)
            {
                opt.time_s = v;
                break;
            }
            std::printf("      ✗ 输入无效，请输入大于 0 的秒数。\n");
        }
        if (opt.time_s <= 0.0)
        {
            std::printf("[错误] 运行时间输入次数过多，退出。\n");
            return 2;
        }
    }
    std::printf("      轨迹时长 : %.2f 秒\n", opt.time_s);

    // =========================================================================
    // --dry-run：用 --start-deg 模拟起始角，只打印计划
    // =========================================================================
    if (opt.dry_run)
    {
        double sim[kJointCount] = {0, 0, 0, 0};
        {
            std::string s = opt.start_deg;
            for (char &c : s)
                if (c == ';' || c == '/' || c == ' ')
                    c = ',';
            std::vector<double> vals;
            size_t pos = 0;
            for (;;)
            {
                const size_t p = s.find(',', pos);
                const std::string tok = s.substr(pos, (p == std::string::npos) ? std::string::npos : p - pos);
                if (!tok.empty())
                    vals.push_back(std::atof(tok.c_str()));
                if (p == std::string::npos)
                    break;
                pos = p + 1;
            }
            if (vals.size() != static_cast<size_t>(kJointCount))
            {
                std::printf("[错误] --start-deg 需要 4 个数，例如 60,30,45,30\n");
                return 2;
            }
            for (int j = 0; j < kJointCount; ++j)
                sim[j] = vals[j] / kRadToDeg;
        }

        std::printf("\n[--dry-run] 用 --start-deg 模拟起始角（%s），只打印计划，不打开 CAN。\n",
                    opt.start_deg.c_str());

        // 限位校验
        double all[kTotalJoints] = {0};
        for (int leg = 0; leg < kLegCount; ++leg)
            for (int j = 0; j < kJointCount; ++j)
                all[leg * kJointCount + j] =
                    (leg == opt.leg) ? sim[j] : 0.0;
        std::string lim_err;
        if (!validateStartPosition(all, opt.limit_deg, opt.leg, lim_err))
        {
            std::printf("\n[限位检查] ✗ %s", lim_err.c_str());
            if (!opt.force)
            {
                std::printf("\n[--dry-run] 真实运行会在这里被拒绝。\n");
                return 0;
            }
            std::printf("\n[--dry-run] 已指定 --force，真实运行会带警告继续。\n");
        }
        else
        {
            std::printf("\n[限位检查] ✓ 全部起点都在软件限位内\n");
        }

        printPlan(opt, sim);

        const double t_min = minTimeForStep(sim, opt.hz, opt.max_step_deg);
        if (opt.max_step_deg > 0.0)
        {
            if (opt.time_s + 1e-9 < t_min)
                std::printf("\n[帧步长检查] ✗ 时间太短：按单帧 ≤ %.2f° 计算，最短需要 %.2f 秒；"
                            "当前只给了 %.2f 秒。\n",
                            opt.max_step_deg, t_min, opt.time_s);
            else
                std::printf("\n[帧步长检查] ✓ 时间充足（按单帧 ≤ %.2f° 计算，最短需 %.2f 秒）\n",
                            opt.max_step_deg, t_min);
        }

        std::printf("\n[--dry-run] 正常退出。\n");
        return 0;
    }

    // =========================================================================
    // [1/6] 打开 CAN
    // =========================================================================
    std::printf("\n[1/6] 打开上位机 CAN 接口：%s\n", opt.can.c_str());
    taihu::SocketCanInterface can;
    if (!can.open(opt.can.c_str(), 0))
    {
        std::printf("      ✗ 打开 %s 失败。\n"
                    "        该接口可能不存在或未 up。请先执行（需 sudo）：\n"
                    "          sudo ip link set %s up\n"
                    "        如果接口名不对，用下面命令查看已有接口：\n"
                    "          ip -br link | grep -E 'can'\n",
                    opt.can.c_str(), opt.can.c_str());
        return 1;
    }
    std::printf("      ✓ %s 已打开\n", opt.can.c_str());

    g_rx_run.store(true);
    std::thread rx(rxThread, &can);

    // —— [2/6] 链路检测 ——
    if (!checkLink(can, opt.wait_link_s))
    {
        std::printf("\n继续启动会盲发回零帧（下位机不在线时电机不会动作）。\n");
        if (!opt.auto_start && interactive)
        {
            std::printf("按回车继续 / 输入 q 回车退出 > ");
            std::fflush(stdout);
            std::string s;
            if (!readLineTrim(s) || s == "q" || s == "Q")
            {
                g_running.store(false);
                g_rx_run.store(false);
                if (rx.joinable())
                    rx.join();
                can.close();
                std::printf("已退出。\n");
                return 1;
            }
        }
    }

    // =========================================================================
    // [3/6] 读当前位置 + 限位校验
    // =========================================================================
    std::printf("\n[3/6] 读取 16 个关节的当前位置（最多等 %.1f 秒）...\n",
                opt.feedback_timeout_s);
    double all_rad[kTotalJoints] = {0};
    std::vector<int> missing;
    if (!readAllJointRad(can, opt.feedback_timeout_s, all_rad, missing))
    {
        std::printf("      ✗ %zu 个关节没有收到 0x41 反馈：", missing.size());
        for (size_t i = 0; i < missing.size(); ++i)
            std::printf(" %s(ID %d)", jointLabel(missing[i]).c_str(),
                        jointToId(missing[i] / kJointCount, missing[i] % kJointCount));
        std::printf("\n        没有起点就无法保证轨迹在限位内，已中止。\n"
                    "        请确认：\n"
                    "          1) 四条腿臂总线（can0~can3）都已 up，且下位机正常运行；\n"
                    "          2) 对应电机已上电、CAN 线连接正常；\n"
                    "          3) 需要更长的等待时间时可加大 --feedback-timeout。\n");
        g_running.store(false);
        g_rx_run.store(false);
        if (rx.joinable())
            rx.join();
        can.close();
        return 1;
    }
    std::printf("      ✓ 16/16 个关节反馈齐全\n");

    // 选中腿的起点
    double start_rad[kJointCount] = {0};
    for (int j = 0; j < kJointCount; ++j)
        start_rad[j] = all_rad[opt.leg * kJointCount + j];

    // 其余腿的冻结保持值
    double frozen[kTotalJoints] = {0};
    for (int i = 0; i < kTotalJoints; ++i)
        frozen[i] = all_rad[i];

    std::printf("      选中腿当前角：");
    for (int j = 0; j < kJointCount; ++j)
        std::printf("  %s %+8.3f°", jointLabel(opt.leg * kJointCount + j).c_str(),
                    start_rad[j] * kRadToDeg);
    std::printf("\n");

    // —— 限位校验 ——
    std::printf("\n      限位检查：");
    std::string lim_err;
    if (!validateStartPosition(all_rad, opt.limit_deg, opt.leg, lim_err))
    {
        std::printf("✗\n%s", lim_err.c_str());
        if (!opt.force)
        {
            g_running.store(false);
            g_rx_run.store(false);
            if (rx.joinable())
                rx.join();
            can.close();
            return 1;
        }
        std::printf("      ⚠ 已指定 --force，继续执行（风险自负）。\n");
    }
    else
    {
        std::printf("✓ 16 个关节全部在限位内\n");
    }

    // =========================================================================
    // [4/6] 生成计划 + 帧步长校验 + 确认
    // =========================================================================
    const double t_min = minTimeForStep(start_rad, opt.hz, opt.max_step_deg);
    if (opt.max_step_deg > 0.0 && opt.time_s + 1e-9 < t_min)
    {
        std::printf("\n[4/6] ✗ 时间太短：行程最大的关节需要 %.2f 秒"
                    "（按单帧 ≤ %.2f° @ %.1f Hz 计算），当前只给了 %.2f 秒。\n",
                    t_min, opt.max_step_deg, opt.hz, opt.time_s);
        if (opt.force)
        {
            std::printf("      ⚠ 已指定 --force，继续执行（可能跟不上目标）。\n");
        }
        else if (interactive && !opt.auto_start)
        {
            bool fixed = false;
            for (int tries = 0; tries < 5 && !fixed; ++tries)
            {
                std::printf("      请输入不小于 %.2f 的运行时间（秒，输入 q 退出）> ", t_min);
                std::fflush(stdout);
                std::string s;
                if (!readLineTrim(s) || s == "q" || s == "Q")
                    break;
                const double v = std::atof(s.c_str());
                if (v + 1e-9 >= t_min)
                {
                    opt.time_s = v;
                    fixed = true;
                }
                else
                {
                    std::printf("      ✗ 仍然太短，至少 %.2f 秒。\n", t_min);
                }
            }
            if (!fixed)
            {
                g_running.store(false);
                g_rx_run.store(false);
                if (rx.joinable())
                    rx.join();
                can.close();
                std::printf("已取消。\n");
                return 2;
            }
        }
        else
        {
            g_running.store(false);
            g_rx_run.store(false);
            if (rx.joinable())
                rx.join();
            can.close();
            return 2;
        }
    }

    std::printf("\n[4/6] 回零计划\n");
    printPlan(opt, start_rad);

    std::printf("\n      ⚠ 确认腿臂周边安全、无人员阻挡后再开始。\n");
    if (!opt.auto_start)
    {
        if (!interactive)
        {
            std::printf("[错误] 需要按回车确认，但标准输入不是终端；脚本场景请加 --auto。\n");
            g_running.store(false);
            g_rx_run.store(false);
            if (rx.joinable())
                rx.join();
            can.close();
            return 2;
        }
        std::printf("\n按回车开始回零（输入 q 回车退出）> ");
        std::fflush(stdout);
        std::string s;
        if (!readLineTrim(s) || s == "q" || s == "Q")
        {
            g_running.store(false);
            g_rx_run.store(false);
            if (rx.joinable())
                rx.join();
            can.close();
            std::printf("已取消，未做任何动作。\n");
            return 2;
        }
    }

    // =========================================================================
    // [5/6] 执行五次多项式轨迹
    // =========================================================================
    const int leg = opt.leg;
    const int base = leg * kJointCount;

    JointTrack track[kJointCount];
    for (int j = 0; j < kJointCount; ++j)
    {
        track[j].p0 = start_rad[j];
        track[j].p1 = 0.0;
    }

    std::printf("\n[5/6] 开始回零：%c 腿 %d 个关节 → 0 位，%.2f 秒 @ %.1f Hz\n",
                legLetter(leg), kJointCount, opt.time_s, opt.hz);

    const double dt = 1.0 / opt.hz;
    const int64_t t0 = nowMs();
    uint8_t data[8] = {0};

    int64_t last_progress = 0;
    bool aborted_limit = false;
    bool aborted_signal = false;
    bool aborted_step = false;
    double last_t = -1.0;

    while (g_running.load())
    {
        const double elapsed = double(nowMs() - t0) / 1000.0;
        const double t = (elapsed >= opt.time_s) ? opt.time_s : elapsed;

        // 到点才发，避免同一时刻重复发多帧
        if (t <= last_t + 1e-9 && elapsed < opt.time_s)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        last_t = t;

        // —— 计算本 tick 16 个关节的目标角 ——
        double target[kTotalJoints];
        for (int i = 0; i < kTotalJoints; ++i)
            target[i] = frozen[i]; // 其余腿原地保持
        for (int j = 0; j < kJointCount; ++j)
            target[base + j] = track[j].at(t, opt.time_s);

        // —— 保险 b/c：下发前逐点断言限位 ——
        for (int j = 0; j < kJointCount; ++j)
        {
            const double deg = target[base + j] * kRadToDeg;
            const double lo = opt.limit_deg[j][0];
            const double hi = opt.limit_deg[j][1];
            if (deg < lo - 1e-6 || deg > hi + 1e-6)
            {
                std::printf("\n✗ [中止] %s 目标 %+.3f° 越出限位 [%+.1f°, %+.1f°]（t=%.3fs）。\n",
                            jointLabel(base + j).c_str(), deg, lo, hi, t);
                aborted_step = true;
                g_running.store(false);
                break;
            }
        }
        if (aborted_step)
            break;

        // —— 下发 16 帧 0x1E ——
        for (int i = 0; i < kTotalJoints; ++i)
        {
            data[0] = kCmdSetPos;
            putI32LE(&data[1], radToProto(target[i]));
            sendFrame(can, jointToId(i / kJointCount, i % kJointCount), 5, data);
        }

        // —— 限位报警监视：一收到 0x162 立即中止 ——
        if (g_limit_locked.load())
        {
            const int jidx = g_limit_joint.load();
            std::printf("\n✗ [中止] 下位机报软限位报警：关节 ID %d，侧 %s，实际 %.3f°（t=%.3fs）\n",
                        jidx,
                        (g_limit_side.load() == kLimitSideMax) ? "上限" : "下限",
                        protoToRad(g_limit_proto.load()) * kRadToDeg, t);
            aborted_limit = true;
            g_running.store(false);
            break;
        }

        // —— 进度（每 1 秒）——
        if (nowMs() - last_progress >= 1000)
        {
            last_progress = nowMs();
            double cur[kJointCount] = {0};
            for (int j = 0; j < kJointCount; ++j)
                cur[j] = jointFresh(base + j) ? protoToRad(g_joints[base + j].last_proto.load())
                                              : 0.0;
            double tgt[kJointCount] = {0};
            for (int j = 0; j < kJointCount; ++j)
                tgt[j] = target[base + j];
            printProgress(t, opt.time_s, tgt, cur, leg);
            std::fflush(stdout);
        }

        if (elapsed >= opt.time_s)
            break;

        // 对齐到下一个发送时刻
        const int64_t next = t0 + static_cast<int64_t>((last_t + dt) * 1000.0);
        const int64_t remain = next - nowMs();
        if (remain > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(remain));
    }

    if (!g_running.load() && !aborted_limit && !aborted_step)
        aborted_signal = true;

    // =========================================================================
    // [6/6] 到位后保持 + 退出流模式
    // =========================================================================
    if (!aborted_limit && !aborted_step && !aborted_signal)
    {
        std::printf("\n[6/6] 已到 0 位，继续保持 %.1f 秒...\n", opt.hold_s);
        const int64_t hold_end = nowMs() + static_cast<int64_t>(opt.hold_s * 1000.0);
        while (nowMs() < hold_end && g_running.load())
        {
            for (int i = 0; i < kTotalJoints; ++i)
            {
                const double v = (i / kJointCount == leg) ? 0.0 : frozen[i];
                data[0] = kCmdSetPos;
                putI32LE(&data[1], radToProto(v));
                sendFrame(can, jointToId(i / kJointCount, i % kJointCount), 5, data);
            }
            if (g_limit_locked.load())
            {
                std::printf("      ✗ 保持期间收到软限位报警，中止保持。\n");
                aborted_limit = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int64_t>(1000.0 / std::max(1.0, opt.hz))));
        }
    }

    // —— 收尾：发 0x02 退出流模式（电机保持，不抱闸）——
    std::printf("\n[收尾] 发送退出流模式命令 0x02（电机保持当前目标，不抱闸）\n");
    {
        uint8_t d[8] = {kCmdStop, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < kTotalJoints; ++i)
            sendFrame(can, jointToId(i / kJointCount, i % kJointCount), 1, d);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    // —— 最终误差 ——
    std::printf("\n[结果] 选中腿 %c 的 4 个关节最终位置 vs 0 位：\n", legLetter(leg));
    for (int j = 0; j < kJointCount; ++j)
    {
        const int idx = base + j;
        if (!jointFresh(idx))
        {
            std::printf("       %s  无反馈\n", jointLabel(idx).c_str());
            continue;
        }
        const double deg = protoToRad(g_joints[idx].last_proto.load()) * kRadToDeg;
        std::printf("       %s  实际 %+9.3f°   误差 %+9.3f°%s\n",
                    jointLabel(idx).c_str(), deg, deg,
                    (std::fabs(deg) <= 0.5) ? "  ✓" : "  ⚠ 超过 0.5°");
    }

    std::printf("\n════════════════════════════════════════════════════════════════\n");
    std::printf("  回零结束：共发 %lld 帧，收 %lld 帧\n",
                static_cast<long long>(g_tx_frames.load()),
                static_cast<long long>(g_rx_frames.load()));
    if (aborted_limit)
        std::printf("  ⚠ 因软限位报警中止\n");
    else if (aborted_step)
        std::printf("  ⚠ 因本地限位断言失败中止\n");
    else if (aborted_signal)
        std::printf("  ⚠ 被用户中断（Ctrl+C）\n");
    else
        std::printf("  ✓ 正常完成\n");
    std::printf("════════════════════════════════════════════════════════════════\n");

    g_running.store(false);
    g_rx_run.store(false);
    if (rx.joinable())
        rx.join();
    can.close();

    if (aborted_limit)
        return 3;
    if (aborted_step || aborted_signal)
        return 1;
    return 0;
}