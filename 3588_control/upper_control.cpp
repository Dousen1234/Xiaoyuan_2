/**
 * @file upper_control.cpp
 * @brief 3588 上位机：读 xlsx 轨迹 → 打包 CAN 帧 → 发给下位机 → 收状态反馈
 *
 * 定位（精简版）：上位机只做两件事 —— 发轨迹、收反馈。
 * 软限位、越限钳位、报警锁定、断流保护、急停等全部由下位机负责，
 * 上位机不做任何前置限位判断，只如实展示下位机上报的状态。
 *
 * 通道分配（两台程序同跑在 3588 上）：
 *   上位机 → 打开 3588 自带的板载 CAN（默认 can_mcu）
 *   下位机 → 打开 鲲弘 can5
 *   两者之间一根 CAN 线（H/L + 两端 120Ω），CAN 口名不同所以互不冲突。
 *
 * 启动后一律先进入交互菜单选择轨迹文件（扫 ../step_position/ 下的 .xlsx），
 * 也可以直接输入任意路径；命令行 --xlsx 只作为非交互场景的兜底。
 *
 * 典型用法：
 *   ./upper_control                                   # 交互式选文件
 *   ./upper_control --can can_mcu --hz 10 --loop 1
 *   ./upper_control --dry-run                         # 只解析不碰 CAN
 *   ./upper_control --list-sheets                     # 看有哪些工作表
 *   ./upper_control --scan-dir /home/rk/traj          # 换扫描目录
 *
 * 关节方向修正：
 *   实机上第 1 / 3 / 4 关节（A1 B1 C1 D1、A3 B3 C3 D3、A4 B4 C4 D4，共 12 个）
 *   转动方向与轨迹文件数值符号相反，第 2 关节（A2 B2 C2 D2）方向一致、不处理。
 *   程序在"读入 xlsx 之后、打包 CAN 之前"对这 12 个关节统一取反，之后的
 *   下发、摘要、状态表全部按真实关节方向显示。
 */

#include "proto.h"
#include "xlsx_reader.h"

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

#include <dirent.h>
#include <limits.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace x3588;

namespace
{

    // =========================================================================
    // 全局状态
    // =========================================================================

    std::atomic<bool> g_running{true};

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
    std::atomic<int> g_limit_count{0};

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

    struct Options
    {
        std::string xlsx = "step_position/traj.xlsx";
        std::string sheet;
        std::string can = "can_mcu";
        std::string scan_dir = "../step_position,../../step_position,.,.."; ///< 扫描目录
        double hz = 10.0;
        int loop = 1; ///< 0 = 无限循环
        bool dry_run = false;
        bool auto_start = false;
        double wait_link_s = 10.0;
        AngleUnit unit = AngleUnit::Auto;
        bool list_sheets = false;
        bool quiet = false;
    };

    void printUsage(const char *prog)
    {
        std::printf(
            "用法: %s [选项]\n"
            "\n"
            "轨迹与通道:\n"
            "  --scan-dir <目录>   交互菜单扫描 .xlsx 的目录，可逗号分隔多个\n"
            "                      （默认 ../step_position,../../step_position,.,..）\n"
            "  --xlsx <路径>       仅当标准输入不是终端时使用的兜底路径（默认 %s）\n"
            "  --sheet <名称>      工作表名（默认取第一张）\n"
            "  --list-sheets       选好文件后只列出工作表名并退出\n"
            "  --can <接口名>      上位机 CAN 接口（默认 can_mcu）\n"
            "  --hz <值>           发送频率 Hz（默认 10.0，范围 1~100）\n"
            "  --loop <次>         循环播放次数（默认 1；0 = 无限）\n"
            "  --unit <rad|deg|auto>  表内角度单位（默认 auto 自动判别）\n"
            "\n"
            "流程控制:\n"
            "  --wait-link <秒>    链路检测最长等待（默认 10.0；0 = 跳过检测）\n"
            "  --auto              不按回车直接开始（脚本化用）\n"
            "  --dry-run           只解析轨迹并打印摘要，不打开 CAN\n"
            "  --quiet             精简输出（不逐帧打印关节表）\n"
            "  -h, --help          显示本帮助\n"
            "\n"
            "说明:\n"
            "  程序启动后一律先进入【选择轨迹文件】菜单，扫描 --scan-dir 目录并编号，\n"
            "  输入序号回车即可；也可直接输入任意路径。\n"
            "  方向修正：第 1/3/4 关节（A1 B1 C1 D1、A3 B3 C3 D3、A4 B4 C4 D4，\n"
            "            共 12 个）读入后自动取反，与实机方向对齐；\n"
            "            第 2 关节（A2/B2/C2/D2）方向一致，不做任何处理。\n"
            "\n"
            "示例:\n"
            "  %s\n"
            "  %s --can can_mcu --hz 10 --loop 3\n"
            "  %s --dry-run\n"
            "  %s --scan-dir /home/rk/traj\n",
            prog, Options().xlsx.c_str(), prog, prog, prog, prog);
    }

    bool parseArgs(int argc, char **argv, Options &o, std::string &err)
    {
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
            else if (a == "--xlsx")
            {
                const char *v = need("--xlsx");
                if (!v)
                    return false;
                o.xlsx = v;
            }
            else if (a == "--scan-dir")
            {
                const char *v = need("--scan-dir");
                if (!v)
                    return false;
                o.scan_dir = v;
            }
            else if (a == "--sheet")
            {
                const char *v = need("--sheet");
                if (!v)
                    return false;
                o.sheet = v;
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
            else if (a == "--loop")
            {
                const char *v = need("--loop");
                if (!v)
                    return false;
                o.loop = std::atoi(v);
                if (o.loop < 0)
                {
                    err = "--loop 不能为负";
                    return false;
                }
            }
            else if (a == "--unit")
            {
                const char *v = need("--unit");
                if (!v)
                    return false;
                const std::string u = v;
                if (u == "rad" || u == "RAD")
                    o.unit = AngleUnit::Radian;
                else if (u == "deg" || u == "DEG")
                    o.unit = AngleUnit::Degree;
                else if (u == "auto" || u == "AUTO")
                    o.unit = AngleUnit::Auto;
                else
                {
                    err = "--unit 只能是 rad / deg / auto";
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
            else if (a == "--auto")
            {
                o.auto_start = true;
            }
            else if (a == "--dry-run")
            {
                o.dry_run = true;
            }
            else if (a == "--list-sheets")
            {
                o.list_sheets = true;
            }
            else if (a == "--quiet")
            {
                o.quiet = true;
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
    // 轨迹文件选择（交互式）
    // =========================================================================

    /// 一条候选轨迹文件
    struct XlsxItem
    {
        std::string path;   ///< 可直接打开的完整路径
        std::string name;   ///< 仅文件名
        long long size = 0; ///< 字节数
    };

    bool endsWithNoCase(const std::string &s, const char *suf)
    {
        const size_t n = std::strlen(suf);
        if (s.size() < n)
            return false;
        for (size_t i = 0; i < n; ++i)
        {
            const char a = static_cast<char>(
                std::tolower(static_cast<unsigned char>(s[s.size() - n + i])));
            const char b = static_cast<char>(
                std::tolower(static_cast<unsigned char>(suf[i])));
            if (a != b)
                return false;
        }
        return true;
    }

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

    bool fileIsRegular(const std::string &p)
    {
        struct stat st;
        return ::stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
    }

    std::string humanSize(long long n)
    {
        char buf[32];
        if (n < 1024LL)
            std::snprintf(buf, sizeof buf, "%lld B", n);
        else if (n < 1024LL * 1024LL)
            std::snprintf(buf, sizeof buf, "%.1f KB", n / 1024.0);
        else
            std::snprintf(buf, sizeof buf, "%.1f MB", n / (1024.0 * 1024.0));
        return std::string(buf);
    }

    /// 终端显示宽度（ASCII 宽 1，中文等多字节字符宽 2）
    int displayWidth(const std::string &s)
    {
        int w = 0;
        for (size_t i = 0; i < s.size();)
        {
            const unsigned char c = static_cast<unsigned char>(s[i]);
            size_t len = 1;
            if (c >= 0xF0)
                len = 4;
            else if (c >= 0xE0)
                len = 3;
            else if (c >= 0xC0)
                len = 2;
            i += len;
            w += (len == 1) ? 1 : 2;
        }
        return w;
    }

    /// 按显示宽度截断（超宽时末尾用 ... 表示），并右补空格到 width
    std::string fitWidth(const std::string &s, int width)
    {
        std::string out;
        int w = 0;
        for (size_t i = 0; i < s.size();)
        {
            const unsigned char c = static_cast<unsigned char>(s[i]);
            size_t len = 1;
            if (c >= 0xF0)
                len = 4;
            else if (c >= 0xE0)
                len = 3;
            else if (c >= 0xC0)
                len = 2;
            const int cw = (len == 1) ? 1 : 2;
            if (w + cw > width)
            {
                // 留 3 格放 ...
                while (!out.empty() && displayWidth(out) + 3 > width)
                {
                    // 从尾部去掉一个完整字符
                    size_t cut = out.size() - 1;
                    while (cut > 0 &&
                           (static_cast<unsigned char>(out[cut]) & 0xC0) == 0x80)
                        --cut;
                    out.erase(cut);
                }
                out += "...";
                return out;
            }
            out += s.substr(i, len);
            i += len;
            w += cw;
        }
        // 右补空格
        w = displayWidth(out);
        for (int k = w; k < width; ++k)
            out.push_back(' ');
        return out;
    }

    /// 扫描单个目录下的 .xlsx（跳过隐藏文件与 Excel 的 ~$ 临时锁文件），按文件名排序
    std::vector<XlsxItem> scanXlsxDir(const std::string &dir)
    {
        std::vector<XlsxItem> out;
        DIR *dp = ::opendir(dir.c_str());
        if (dp == nullptr)
            return out;

        std::string prefix = dir;
        if (!prefix.empty() && prefix.back() != '/')
            prefix += '/';

        struct dirent *de = nullptr;
        while ((de = ::readdir(dp)) != nullptr)
        {
            const std::string nm = de->d_name;
            if (nm.empty() || nm[0] == '.')
                continue;
            if (nm.rfind("~$", 0) == 0) ///< Excel 打开文件时生成的锁文件
                continue;
            if (!endsWithNoCase(nm, ".xlsx"))
                continue;

            const std::string full = prefix + nm;
            struct stat st;
            if (::stat(full.c_str(), &st) != 0 || !S_ISREG(st.st_mode))
                continue;

            XlsxItem it;
            it.path = full;
            it.name = nm;
            it.size = static_cast<long long>(st.st_size);
            out.push_back(it);
        }
        ::closedir(dp);

        std::sort(out.begin(), out.end(),
                  [](const XlsxItem &a, const XlsxItem &b)
                  { return a.name < b.name; });
        return out;
    }

    /// 按逗号拆目录列表，逐目录扫描并合并去重（同名以先扫到的为准）
    std::vector<XlsxItem> scanXlsxDirs(const std::string &dir_list)
    {
        std::vector<XlsxItem> all;
        std::vector<std::string> seen;
        std::string cur;
        for (size_t i = 0; i <= dir_list.size(); ++i)
        {
            if (i == dir_list.size() || dir_list[i] == ',')
            {
                const std::string d = trimStr(cur);
                cur.clear();
                if (!d.empty())
                    seen.push_back(d);
                continue;
            }
            cur.push_back(dir_list[i]);
        }
        if (seen.empty())
            seen.push_back(".");

        std::vector<std::string> paths;
        for (size_t k = 0; k < seen.size(); ++k)
        {
            const std::vector<XlsxItem> one = scanXlsxDir(seen[k]);
            for (size_t j = 0; j < one.size(); ++j)
            {
                // 用 realpath 去重，避免 ./a.xlsx 与 a.xlsx 重复列出
                char rp[PATH_MAX];
                std::string key = one[j].path;
                if (::realpath(one[j].path.c_str(), rp) != nullptr)
                    key = rp;
                bool dup = false;
                for (size_t q = 0; q < all.size(); ++q)
                {
                    char rq[PATH_MAX];
                    std::string k2 = all[q].path;
                    if (::realpath(all[q].path.c_str(), rq) != nullptr)
                        k2 = rq;
                    if (k2 == key)
                    {
                        dup = true;
                        break;
                    }
                }
                if (!dup)
                    all.push_back(one[j]);
            }
        }

        std::sort(all.begin(), all.end(),
                  [](const XlsxItem &a, const XlsxItem &b)
                  { return a.name < b.name; });
        return all;
    }

    /// 交互式选择轨迹文件。
    /// 标准输入不是终端时（管道/重定向）直接沿用 path 里的值，便于脚本化。
    /// 返回 false 表示用户要求退出。
    bool chooseTrajectoryFile(std::string &path, const std::string &scan_dir)
    {
        if (::isatty(STDIN_FILENO) == 0)
        {
            std::printf("[0/4] 标准输入不是终端，沿用命令行路径：%s\n", path.c_str());
            return true;
        }

        // 扫描目录列表里第一个存在的目录，用于相对路径兜底查找
        std::string first_dir = ".";
        {
            std::string cur;
            for (size_t i = 0; i <= scan_dir.size(); ++i)
            {
                if (i == scan_dir.size() || scan_dir[i] == ',')
                {
                    const std::string d = trimStr(cur);
                    cur.clear();
                    if (!d.empty() && scanXlsxDir(d).size() > 0)
                    {
                        first_dir = d;
                        break;
                    }
                    continue;
                }
                cur.push_back(scan_dir[i]);
            }
        }

        for (;;)
        {
            const std::vector<XlsxItem> items = scanXlsxDirs(scan_dir);

            std::printf("\n[0/4] 请选择要下发的轨迹文件\n");
            std::printf("      扫描目录：%s\n", scan_dir.c_str());

            if (items.empty())
            {
                std::printf("      （该目录下没有找到 .xlsx 文件，可直接输入完整路径）\n");
            }
            else
            {
                std::printf(
                    "      ┌──────┬──────────────────────────────────────────┬────────┐\n");
                std::printf(
                    "      │ 序号 │ 文件名                                   │ 大小   │\n");
                std::printf(
                    "      ├──────┼──────────────────────────────────────────┼────────┤\n");
                for (size_t i = 0; i < items.size(); ++i)
                {
                    const std::string nm = fitWidth(items[i].name, 40);
                    std::string sz = humanSize(items[i].size);
                    if (sz.size() < 8)
                        sz.insert(sz.begin(), 8 - sz.size(), ' ');
                    std::printf("      │ %4zu │ %s │ %s │\n", i + 1, nm.c_str(),
                                sz.c_str());
                }
                std::printf(
                    "      └──────┴──────────────────────────────────────────┴────────┘\n");
            }

            std::printf("      输入 序号 回车选择；也可直接输入文件路径；\n");
            std::printf("      输入 r 回车重新扫描，输入 q 回车退出。\n");
            std::printf("      > ");
            std::fflush(stdout);

            char line[512] = {0};
            if (std::fgets(line, sizeof line, stdin) == nullptr)
            {
                std::printf("\n[0/4] 读取输入失败（EOF），退出。\n");
                return false;
            }

            const std::string in = trimStr(line);
            if (in.empty())
                continue;
            if (in == "q" || in == "Q")
                return false;
            if (in == "r" || in == "R")
                continue;

            // —— 纯数字：按序号选 ——
            bool all_digits = true;
            for (size_t i = 0; i < in.size(); ++i)
            {
                if (!std::isdigit(static_cast<unsigned char>(in[i])))
                {
                    all_digits = false;
                    break;
                }
            }
            if (all_digits)
            {
                const long n = std::atol(in.c_str());
                if (n >= 1 && static_cast<size_t>(n) <= items.size())
                {
                    path = items[static_cast<size_t>(n - 1)].path;
                    std::printf("      ✓ 已选择：%s\n", path.c_str());
                    return true;
                }
                std::printf("      ✗ 序号 %ld 超出范围（可选 1~%zu）。\n", n, items.size());
                continue;
            }

            // —— 其他：当作路径 ——
            if (fileIsRegular(in))
            {
                path = in;
                std::printf("      ✓ 已选择：%s\n", path.c_str());
                return true;
            }

            // 相对路径在扫描目录下再找一次
            std::string alt = first_dir;
            if (!alt.empty() && alt.back() != '/')
                alt += '/';
            alt += in;
            if (fileIsRegular(alt))
            {
                path = alt;
                std::printf("      ✓ 已选择：%s\n", path.c_str());
                return true;
            }

            std::printf("      ✗ 找不到文件：%s\n", in.c_str());
        }
    }

    // =========================================================================
    // 关节方向修正
    // =========================================================================

    /// 实机上第 1 / 3 / 4 关节（A1 B1 C1 D1、A3 B3 C3 D3、A4 B4 C4 D4）的
    /// 转动方向与轨迹文件数值符号相反，第 2 关节（A2 B2 C2 D2）方向一致。
    /// 这里在读入 xlsx 之后立刻对那 12 个关节取反，第 2 关节保持原值不动；
    /// 之后的下发、摘要、状态表统一按"真实关节方向"处理，避免对照表时符号错乱。
    void applyJointDirectionFix(Trajectory &traj)
    {
        for (size_t i = 0; i < traj.pos_rad.size(); ++i)
        {
            for (int leg = 0; leg < kLegCount; ++leg)
            {
                for (int j = 0; j < kJointCount; ++j)
                {
                    // 第 2 关节（下标 1）不取反，其余 3 个关节取反
                    if (j == 1)
                        continue;
                    double &v = traj.pos_rad[i][leg * kJointCount + j];
                    v = -v;
                }
            }
        }
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
        while (g_running.load())
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
                // 下位机已清除报警（bit2 清零）→ 同步复位本地警报显示，
                // 否则下位机 clear 后上位机界面会一直挂着旧报警。
                if ((rx.data[0] & kHbBitLimitLock) == 0 &&
                    g_limit_locked.exchange(false))
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
                g_limit_count.fetch_add(1);
                continue;
            }
        }
    }

    /// 下位机心跳是否新鲜（1s 内）
    bool heartbeatFresh() { return nowMs() - g_hb_ms.load() < 1000; }

    /// 当前在线的关节数
    int onlineCount()
    {
        int n = 0;
        for (int i = 0; i < kTotalJoints; ++i)
            if (g_joints[i].online.load() &&
                nowMs() - g_joints[i].last_ms.load() < 1000)
                ++n;
        return n;
    }

    // =========================================================================
    // 轨迹插值
    // =========================================================================

    /// 在时间 t（秒）处线性插值出 16 个关节角（rad）
    void sampleAt(const Trajectory &traj, double t, double out[kTotalJoints])
    {
        const size_t n = traj.sampleCount();
        if (n == 0)
        {
            for (int j = 0; j < kTotalJoints; ++j)
                out[j] = 0.0;
            return;
        }
        if (t <= traj.time_s.front())
        {
            for (int j = 0; j < kTotalJoints; ++j)
                out[j] = traj.pos_rad.front()[j];
            return;
        }
        if (t >= traj.time_s.back())
        {
            for (int j = 0; j < kTotalJoints; ++j)
                out[j] = traj.pos_rad.back()[j];
            return;
        }

        // 二分找区间
        size_t lo = 0, hi = n - 1;
        while (hi - lo > 1)
        {
            const size_t mid = (lo + hi) / 2;
            if (traj.time_s[mid] <= t)
                lo = mid;
            else
                hi = mid;
        }
        const double t0 = traj.time_s[lo];
        const double t1 = traj.time_s[hi];
        const double u = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
        for (int j = 0; j < kTotalJoints; ++j)
        {
            const double a = traj.pos_rad[lo][j];
            const double b = traj.pos_rad[hi][j];
            out[j] = a + (b - a) * u;
        }
    }

    // =========================================================================
    // 状态打印
    // =========================================================================

    /// 打印一次完整状态表
    void printStatus(const Options &opt, double traj_t, int loop_i)
    {
        const uint8_t hb = g_hb.load();
        const int n_online = onlineCount();

        std::printf("\n┌─ 状态 @%.2fs（第 %d 轮）───────────────────────────────────────┐\n",
                    traj_t, loop_i);
        std::printf("│ 下位机心跳 0x%03X: %s  流模式[%s] 样点[%s] 限位锁定[%s]  在线 %2d/16 │\n",
                    kHeartbeatId, heartbeatFresh() ? "正常" : "丢失",
                    (hb & kHbBitStreaming) ? "是" : "否",
                    (hb & kHbBitOnline) ? "是" : "否",
                    (hb & kHbBitLimitLock) ? "是" : "否", n_online);

        for (int leg = 0; leg < kLegCount; ++leg)
        {
            std::printf("│ 腿臂 %c  ", legLetter(leg));
            for (int j = 0; j < kJointCount; ++j)
            {
                const int idx = leg * kJointCount + j;
                const bool on = g_joints[idx].online.load() &&
                                nowMs() - g_joints[idx].last_ms.load() < 1000;
                if (on)
                {
                    const double deg = protoToRad(g_joints[idx].last_proto.load()) *
                                       kRadToDeg;
                    std::printf("%c%d %+8.2f°  ", legLetter(leg), j + 1, deg);
                }
                else
                {
                    std::printf("%c%d   --.--°  ", legLetter(leg), j + 1);
                }
            }
            std::printf("│\n");
        }

        if (g_limit_locked.load())
        {
            const int jid = g_limit_joint.load();
            const double deg = protoToRad(g_limit_proto.load()) * kRadToDeg;
            std::string name = "??";
            if (isJointId(static_cast<uint16_t>(jid)))
            {
                const int l = idToLeg(static_cast<uint16_t>(jid));
                const int j = idToJoint(static_cast<uint16_t>(jid));
                name = std::string(1, legLetter(l)) + std::string(1, char('1' + j));
            }
            std::printf("│ ⚠ 软限位报警锁定：#%d 关节 %s 撞%s限，当前 %+.2f°          │\n",
                        jid, name.c_str(),
                        g_limit_side.load() == kLimitSideMax ? "上" : "下", deg);
        }
        std::printf("└───────────────────────────────────────────────────────────────┘\n");
        (void)opt;
    }

    /// 把「取反后的值」还原成 xlsx 源文件里的原值，供摘要对照用。
    /// 第 2 关节（下标 1）不取反，直接返回；其余 3 个关节取反回去。
    inline double srcAngleOf(double fixed, int joint)
    {
        return (joint == 1) ? fixed : -fixed;
    }

    /// 打印轨迹摘要
    void printTrajectorySummary(const Trajectory &traj, const Options &opt)
    {
        std::printf("\n┌─ 轨迹摘要 ────────────────────────────────────────────────────┐\n");
        std::printf("│ 文件      : %s\n", traj.source.c_str());
        std::printf("│ 工作表    : %s\n", traj.sheet.c_str());
        std::printf("│ 方向修正  : 第 1/3/4 关节共 12 个已取反，第 2 关节（A2/B2/C2/D2）不处理\n");
        std::printf("│             （下发给下位机的是取反后的值，本摘要显示源文件原值）\n");
        std::printf("│ 样点数    : %zu\n", traj.sampleCount());
        std::printf("│ 总时长    : %.2f s\n", traj.durationS());
        std::printf("│ 平均间隔  : %.4f s（约 %.2f Hz）\n", traj.intervalS(),
                    traj.intervalS() > 0 ? 1.0 / traj.intervalS() : 0.0);
        std::printf("│ 相邻样点最大单关节变化 : %.4f rad（%.2f°）\n", traj.maxStepRad(),
                    traj.maxStepRad() * kRadToDeg);

        // 每个关节的行程（按源文件原值显示，方便逐列对照 xlsx）
        std::printf("│ 各关节行程（度，源文件原值）:\n");
        for (int leg = 0; leg < kLegCount; ++leg)
        {
            std::printf("│   %c: ", legLetter(leg));
            for (int j = 0; j < kJointCount; ++j)
            {
                double lo = 1e30, hi = -1e30;
                for (size_t i = 0; i < traj.sampleCount(); ++i)
                {
                    const double v = srcAngleOf(traj.pos_rad[i][leg * kJointCount + j], j);
                    if (v < lo)
                        lo = v;
                    if (v > hi)
                        hi = v;
                }
                std::printf("%c%d [%+6.1f, %+6.1f] ", legLetter(leg), j + 1,
                            lo * kRadToDeg, hi * kRadToDeg);
            }
            std::printf("\n");
        }

        // 发送频率检查
        const double send_interval = 1.0 / opt.hz;
        if (traj.maxStepRad() * kRadToDeg > 2.0)
        {
            const double needed_hz = opt.hz * (traj.maxStepRad() * kRadToDeg) / 2.0;
            std::printf("│ ⚠ 提示：相邻样点最大变化超过 2°，建议提高 --hz 到 %.0f 以上，\n"
                        "│         否则关节可能跟随不及时。当前 %0.1f Hz / 帧间隔 %.3fs。\n",
                        needed_hz, opt.hz, send_interval);
        }
        std::printf("└───────────────────────────────────────────────────────────────┘\n");
    }

    // =========================================================================
    // 链路检测
    // =========================================================================

    /// 被动等待下位机心跳 + 主动点名查询，确认链路可用
    bool checkLink(taihu::SocketCanInterface &can, double wait_s)
    {
        if (wait_s <= 0.0)
            return true;

        std::printf("[3/4] 链路检测：等待下位机心跳（最多 %.0f 秒）...\n", wait_s);
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
            // 每秒主动点名一次：向上位机侧 16 个 ID 各发一帧 0x41 状态请求
            if (nowMs() - last_ping >= 1000)
            {
                last_ping = nowMs();
                uint8_t d[8] = {kCmdGetState, 0, 0, 0, 0, 0, 0, 0};
                for (int i = 0; i < kTotalJoints; ++i)
                    sendFrame(can, jointToId(i / kJointCount, i % kJointCount), 1, d);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
    // 主流程
    // =========================================================================

    /// 播放一轮轨迹；返回 false 表示用户中断
    bool playOnce(taihu::SocketCanInterface &can, const Trajectory &traj,
                  const Options &opt, int loop_i)
    {
        const double dt = 1.0 / opt.hz;
        const double dur = traj.durationS();
        const int64_t t0 = nowMs();
        uint8_t data[8] = {0};

        int64_t last_status = 0;
        double last_t = -1.0;

        while (g_running.load())
        {
            const double elapsed = double(nowMs() - t0) / 1000.0;
            const double t = (elapsed >= dur) ? dur : elapsed;

            // 到点才发（避免同一时刻重复发多帧）
            if (t <= last_t + 1e-9 && elapsed < dur)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            last_t = t;

            // —— 下发 16 帧 0x1E ——
            double pos[kTotalJoints];
            sampleAt(traj, t, pos);
            for (int leg = 0; leg < kLegCount; ++leg)
            {
                for (int j = 0; j < kJointCount; ++j)
                {
                    data[0] = kCmdSetPos;
                    putI32LE(&data[1], radToProto(pos[leg * kJointCount + j]));
                    sendFrame(can, jointToId(leg, j), 5, data);
                }
            }

            // —— 状态表（每 500ms 一次）——
            if (nowMs() - last_status >= 500)
            {
                last_status = nowMs();
                printStatus(opt, t, loop_i);
            }

            if (elapsed >= dur)
                break; // 收到末尾样点，结束本轮

            // 对齐到下一个发送时刻
            const int64_t next = t0 + static_cast<int64_t>((last_t + dt) * 1000.0);
            const int64_t remain = next - nowMs();
            if (remain > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(remain));
        }

        return g_running.load();
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
    std::printf("  3588 上位机 · 轨迹下发 · V2.0 协议\n");
    std::printf("════════════════════════════════════════════════════════════════\n");

    // —— [0/4] 交互式选择轨迹文件 ——
    if (!chooseTrajectoryFile(opt.xlsx, opt.scan_dir))
    {
        std::printf("\n已取消，未做任何动作。\n");
        return 0;
    }

    // —— 只列工作表 ——
    if (opt.list_sheets)
    {
        std::vector<std::string> names;
        if (!listXlsxSheets(opt.xlsx, names, err))
        {
            std::printf("[错误] %s\n", err.c_str());
            return 1;
        }
        std::printf("文件 %s 共 %zu 张工作表：\n", opt.xlsx.c_str(), names.size());
        for (size_t i = 0; i < names.size(); ++i)
            std::printf("  [%zu] %s\n", i, names[i].c_str());
        return 0;
    }

    // —— [1/4] 加载轨迹 ——
    std::printf("[1/4] 加载轨迹文件：%s\n", opt.xlsx.c_str());
    Trajectory traj;
    std::string warn;
    if (!loadTrajectory(opt.xlsx, opt.sheet, opt.unit, traj, err, warn))
    {
        std::printf("\n[错误] %s\n", err.c_str());
        std::printf("       正确的格式为 17 列：\n"
                    "         第 1 行表头：Time  A1 A2 A3 A4  B1 B2 B3 B4  "
                    "C1 C2 C3 C4  D1 D2 D3 D4\n"
                    "         第 2 行起：每行一个时间点，第 1 列时间(秒)，"
                    "后 16 列关节角\n");
        return 1;
    }
    if (!warn.empty())
        std::printf("      %s", warn.c_str());

    // —— 关节方向修正：第 1 / 3 / 4 关节（共 12 个）取反 ——
    // 实机上这 12 个关节的转动方向与轨迹文件数值符号相反，第 2 关节方向一致。
    // 这里在读入后立刻取反，之后下发/摘要/状态表全部按真实方向处理。
    applyJointDirectionFix(traj);

    double lo_all = 1e30, hi_all = -1e30;
    for (size_t i = 0; i < traj.sampleCount(); ++i)
        for (int j = 0; j < kTotalJoints; ++j)
        {
            // 按源文件原值统计，避免摘要与 xlsx 内容对不上
            const double v =
                srcAngleOf(traj.pos_rad[i][j], j % kJointCount) * kRadToDeg;
            if (v < lo_all)
                lo_all = v;
            if (v > hi_all)
                hi_all = v;
        }
    std::printf("      ✓ %zu 个样点，%.2f 秒，%.1f Hz，关节角范围 [%+.1f°, %+.1f°]（源文件原值）\n",
                traj.sampleCount(), traj.durationS(),
                traj.intervalS() > 0 ? 1.0 / traj.intervalS() : 0.0, lo_all, hi_all);
    printTrajectorySummary(traj, opt);

    if (opt.dry_run)
    {
        std::printf("\n[--dry-run] 只解析轨迹，不打开 CAN，正常退出。\n");
        return 0;
    }

    // —— [2/4] 打开 CAN ——
    std::printf("\n[2/4] 打开上位机 CAN 接口：%s\n", opt.can.c_str());
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

    // —— [3/4] 链路检测 ——
    std::atomic<bool> rx_run{true};
    std::thread rx;
    {
        g_running.store(true);
        rx = std::thread(rxThread, &can);
    }

    if (!checkLink(can, opt.wait_link_s))
    {
        std::printf("\n继续启动会盲发轨迹（下位机不在线时电机不会动作）。\n");
        if (!opt.auto_start)
        {
            std::printf("按回车继续 / 输入 q 回车退出 > ");
            std::fflush(stdout);
            char line[64] = {0};
            if (std::fgets(line, sizeof(line), stdin) == nullptr ||
                line[0] == 'q' || line[0] == 'Q')
            {
                g_running.store(false);
                rx_run.store(false);
                if (rx.joinable())
                    rx.join();
                can.close();
                std::printf("已退出。\n");
                return 1;
            }
        }
    }

    // —— [4/4] 开始发送 ——
    std::printf("\n[4/4] 准备发送\n");
    std::printf("      CAN 接口 : %s\n", opt.can.c_str());
    std::printf("      发送频率 : %.1f Hz\n", opt.hz);
    std::printf("      循环次数 : %s\n", opt.loop == 0 ? "无限" : std::to_string(opt.loop).c_str());
    std::printf("      总时长   : %.2f 秒/轮\n", traj.durationS());
    std::printf("\n      ⚠ 确认机械臂周边安全后再开始。\n");

    if (!opt.auto_start)
    {
        std::printf("\n按回车开始发送（输入 q 回车退出）> ");
        std::fflush(stdout);
        char line[64] = {0};
        if (std::fgets(line, sizeof(line), stdin) == nullptr || line[0] == 'q' ||
            line[0] == 'Q')
        {
            g_running.store(false);
            rx_run.store(false);
            if (rx.joinable())
                rx.join();
            can.close();
            std::printf("已退出。\n");
            return 0;
        }
    }

    // —— 播放 ——
    int done = 0;
    while (g_running.load() && (opt.loop == 0 || done < opt.loop))
    {
        ++done;
        if (opt.loop != 1)
            std::printf("\n═══ 第 %d 轮开始 ═══\n", done);
        if (!playOnce(can, traj, opt, done))
            break;
        if (!opt.quiet)
            printStatus(opt, traj.durationS(), done);
        if (g_limit_locked.load())
        {
            std::printf("\n⚠ 检测到软限位报警锁定，提前结束发送。\n");
            break;
        }
        if (opt.loop == 0 || done < opt.loop)
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    // —— 收尾：发 0x02 退出流模式（电机保持，不抱闸）——
    if (g_running.load() || done > 0)
    {
        std::printf("\n[收尾] 发送退出流模式命令 0x02（电机保持当前目标，不抱闸）\n");
        uint8_t d[8] = {kCmdStop, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < kTotalJoints; ++i)
            sendFrame(can, jointToId(i / kJointCount, i % kJointCount), 1, d);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::printf("\n════════════════════════════════════════════════════════════════\n");
    std::printf("  发送结束：共发 %lld 帧，收 %lld 帧，完成 %d 轮\n",
                static_cast<long long>(g_tx_frames.load()),
                static_cast<long long>(g_rx_frames.load()), done);
    std::printf("════════════════════════════════════════════════════════════════\n");

    g_running.store(false);
    rx_run.store(false);
    if (rx.joinable())
        rx.join();
    can.close();
    return g_limit_locked.load() ? 3 : 0;
}