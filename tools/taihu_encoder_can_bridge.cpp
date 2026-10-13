/**
 * @file taihu_encoder_can_bridge.cpp
 * @brief 金钢 MPT 磁编码器 → CAN 总线软件网关。
 *
 * 背景：MPT 全系列没有 CAN 输出型号（接口只有 SSI/BiSS-C/RS485/RS422/BUS），
 * 且 R 款（RS485）一条总线只能挂 1 台。本工具把编码器的 RS485 读数
 * "翻译"成 CAN 帧打到 SocketCAN 总线上，实现：
 *   - 编码器数据以固定周期（默认 200Hz）发布到电机所在的 CAN 总线；
 *   - 总线上任何节点/程序都能用标准 CAN 帧读到关节绝对角度；
 *   - 与电机、扭矩传感器共总线互不干扰（帧 ID 甄别，同 taihu_torque_sensor 模式）。
 *
 * 硬件连接（无需新购硬件，复用现有设备）：
 *
 *   MPT 编码器 ──RS485(2.5M)── FT232H USB转485 ── USB ──┐
 *                                                        ├── 同一台 Linux 主机
 *   电机/扭矩传感器 ──CAN── 鲲弘 USB-CANFD(canX) ── USB ──┘
 *
 * CAN 帧协议（标准帧，ID 可通过命令行参数覆盖）：
 *   - 0x20 数据帧（8 字节，网关按 rate_hz 周期发送）：
 *       [0]    状态字节 S（b7=错误，数据不可信；b6=警告，数据仍有效）
 *       [1..3] 24 位单圈原始值，大端（A2 A1 A0）
 *       [4..7] 单圈角度，单位 0.001°，int32 大端（0~360000 表示 0°~360°）
 *     candump 直读示例： 0x20 [8] 00 7F FF F0 05 A0 00 1E
 *     解析：状态 0x00 正常；raw=0x7FFF0F？——raw=0x7FFFF0；角度=0x05A0001E=9437182 → 9.437°
 *   - 0x21 命令帧（≥1 字节，总线上任何节点可发，网关监听）：
 *       [0] 0x01 = 设置零位（触发 RS485 0x31/0x30 交替序列；期间约 50ms 无数据帧）
 *       其它值暂未定义，忽略
 *
 * 用法：
 *   ./build/taihu_encoder_can_bridge <串口> <CAN接口> [频率Hz] [数据帧ID] [命令帧ID]
 *   ./build/taihu_encoder_can_bridge /dev/ttyUSB0 can0              # 默认 200Hz、0x20/0x21
 *   ./build/taihu_encoder_can_bridge /dev/ttyUSB0 can0 100          # 100Hz
 *   ./build/taihu_encoder_can_bridge /dev/ttyUSB0 can0 200 0x28 0x29  # 自定义 ID
 *
 * 总线触发设零位（外部程序/脚本示例）：
 *   cansend can0 021#01
 *
 * 键盘命令（网关终端）：z=设零位  w=CSV日志开关  q=退出
 *
 * @note 与 taihu_mpt_encoder（纯 RS485 查看工具）的区别：本工具把数据
 *       桥接到 CAN 总线供其它节点消费；两者不要同时读同一台编码器。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <termios.h>
#include <unistd.h>

#include "taihu/can_interface.h"
#include "taihu/mpt_encoder.h"
#include "taihu/socket_can.h"

namespace {

using taihu::CanFrame;
using taihu::MptEncoder;
using taihu::Rs485Port;
using taihu::SocketCanInterface;

// =====================================================================
// 终端 raw 模式（单键命令，无需回车；析构自动恢复）
// =====================================================================
class RawTerminal {
public:
    ~RawTerminal() { leave(); }
    bool enter() {
        if (!::isatty(STDIN_FILENO)) return false;
        if (::tcgetattr(STDIN_FILENO, &saved_) != 0) return false;
        struct termios raw = saved_;
        ::cfmakeraw(&raw);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        if (::tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) return false;
        active_ = true;
        return true;
    }
    void leave() {
        if (!active_) return;
        ::tcsetattr(STDIN_FILENO, TCSAFLUSH, &saved_);
        active_ = false;
    }
private:
    struct termios saved_{};
    bool active_ = false;
};

int getKey() { return ::getchar(); }

// =====================================================================
// CSV 日志（与 taihu_mpt_encoder 同格式，方便 plot_motor_log.py 类工具扩展）
// =====================================================================
class BridgeLogger {
public:
    explicit BridgeLogger(const std::string& path) {
        file_ = std::fopen(path.c_str(), "w");
        if (file_) std::fprintf(file_, "timestamp_s,angle_deg,raw,status,error,warning\n");
    }
    ~BridgeLogger() { if (file_) std::fclose(file_); }
    bool ok() const { return file_ != nullptr; }
    void log(double t, const MptEncoder::Sample& s) {
        if (!file_) return;
        std::fprintf(file_, "%.6f,%.4f,%u,%u,%d,%d\n",
                     t, s.angle_deg, s.raw, s.status,
                     s.error ? 1 : 0, s.warning ? 1 : 0);
    }
    void flush() { if (file_) std::fflush(file_); }
private:
    std::FILE* file_ = nullptr;
};

constexpr auto kPrintPeriod = std::chrono::milliseconds(200);

/// 把一次采样打包成 0x20 数据帧
void packDataFrame(const MptEncoder::Sample& s, CanFrame& f) {
    f.id = 0x20;   // 调用方随后按参数覆盖
    f.is_extended = false;
    f.dlc = 8;
    const uint32_t mdeg = static_cast<uint32_t>(
        static_cast<int32_t>(std::lround(s.angle_deg * 1000.0)) & 0x7FFFFFFF);
    f.data[0] = s.status;
    f.data[1] = static_cast<uint8_t>((s.raw >> 16) & 0xFF);
    f.data[2] = static_cast<uint8_t>((s.raw >> 8) & 0xFF);
    f.data[3] = static_cast<uint8_t>(s.raw & 0xFF);
    f.data[4] = static_cast<uint8_t>((mdeg >> 24) & 0xFF);
    f.data[5] = static_cast<uint8_t>((mdeg >> 16) & 0xFF);
    f.data[6] = static_cast<uint8_t>((mdeg >> 8) & 0xFF);
    f.data[7] = static_cast<uint8_t>(mdeg & 0xFF);
}

} // namespace

int main(int argc, char** argv) {
    // —— 参数解析：串口、CAN 接口、频率、帧 ID ——
    const char* device  = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const char* ifname  = (argc > 2) ? argv[2] : "can0";
    int rate_hz         = (argc > 3) ? std::atoi(argv[3]) : 200;
    uint32_t data_id    = (argc > 4) ? static_cast<uint32_t>(std::strtoul(argv[4], nullptr, 0)) : 0x20;
    uint32_t cmd_id     = (argc > 5) ? static_cast<uint32_t>(std::strtoul(argv[5], nullptr, 0)) : 0x21;
    if (rate_hz <= 0 || rate_hz > 1000) rate_hz = 200;

    std::printf("==============================================\n");
    std::printf("  金钢 MPT 编码器 → CAN 网关\n");
    std::printf("==============================================\n");
    std::printf("  RS485: %s (2.5Mbps)   CAN: %s\n", device, ifname);
    std::printf("  发布: %d Hz  数据帧 0x%02X  命令帧 0x%02X\n", rate_hz, data_id, cmd_id);
    std::printf("  命令: z=设零位  w=日志开关  q=退出\n");
    std::printf("  总线设零: cansend %s %03X#01\n", ifname, cmd_id);
    std::printf("----------------------------------------------\n");

    // —— 打开 RS485 串口 ——
    Rs485Port port;
    std::string err;
    if (!port.open(device, 2500000, &err)) {
        std::fprintf(stderr, "[错误] 打开串口 %s 失败：%s\n", device, err.c_str());
        return 1;
    }

    // —— 打开 CAN ——
    SocketCanInterface can;
    if (!can.open(ifname, 1000000)) {
        std::fprintf(stderr, "[错误] 打开 %s 失败（外部终端 sudo ./build/kcanctl up %s 1000000）\n",
                     ifname, ifname);
        return 1;
    }

    // —— 探测编码器 ——
    uint8_t found_addr = MptEncoder::kFactoryAddr;
    std::printf("正在探测编码器（BUS 0x1F / RS485）...\n");
    MptEncoder::Protocol proto = MptEncoder::probe(port, &found_addr,
                                                    MptEncoder::Output::Single24);
    MptEncoder enc(port, proto, found_addr);
    MptEncoder::Sample s;
    const bool online = enc.readPosition(s);
    const char* proto_name = (proto == MptEncoder::Protocol::BUS) ? "BUS(D款)" : "RS485(R款)";
    if (online) {
        std::printf("✓ 编码器在线: %s  当前角度 %.3f°\n", proto_name, s.angle_deg);
    } else {
        std::printf("⚠ 探测未确认（按 %s 继续尝试读取）\n", proto_name);
    }

    // —— CAN 接收线程：监听命令帧（0x21），置 atomic 标志 ——
    std::atomic<bool> zero_requested{false};
    std::atomic<bool> rx_running{true};
    std::thread rx_thread([&]() {
        while (rx_running) {
            CanFrame f;
            if (!can.receive(f, 100)) continue;   // 100ms 超时轮询退出标志
            if (f.id == cmd_id && f.dlc >= 1 && f.data[0] == 0x01)
                zero_requested = true;            // 设零位命令
            // 其余帧（电机/扭矩传感器/未知）一律忽略——共总线零冲突
        }
    });

    RawTerminal term;
    if (!term.enter()) std::printf("[提示] stdin 非 tty，改为逐行命令模式\n");

    BridgeLogger logger("record/encoder_can_bridge.csv");
    if (logger.ok()) std::printf("[日志] record/encoder_can_bridge.csv 待 w 开启写入\n");
    bool logging = false;

    const auto t0 = std::chrono::steady_clock::now();
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    auto next_sample = t0;
    auto last_stat = t0;
    auto last_print = t0;
    int published = 0, read_fail = 0, stat_pub = 0, stat_fail = 0;
    MptEncoder::Sample last{};
    bool have_sample = online;
    if (online) last = s;

    bool running = true;
    while (running) {
        const auto now = std::chrono::steady_clock::now();

        // —— 设零位（键盘 z 或总线命令 0x21#01 都触发这里）——
        if (zero_requested.exchange(false)) {
            std::printf("\n[命令] 总线请求设零位...\n");
            if (enc.setZero()) std::printf("[信息] 设零位成功（编码器 50ms 不响应属正常）\n");
            else std::printf("[警告] 设零位未确认（继续运行）\n");
            next_sample = std::chrono::steady_clock::now();  // 重置节拍
        }

        // —— 周期采样 + 发布 ——
        if (now >= next_sample) {
            // 落后超过 3 个周期则重置节拍，防追赶风暴
            if (now - next_sample > period * 3) next_sample = now;
            else next_sample += period;

            MptEncoder::Sample cur;
            if (enc.readPosition(cur)) {
                last = cur;
                have_sample = true;
                ++published;
                CanFrame f;
                packDataFrame(cur, f);
                f.id = data_id;
                can.send(f);
                if (logging) {
                    double t = std::chrono::duration<double>(now - t0).count();
                    logger.log(t, cur);
                }
                if (cur.error) {
                    static auto lastErr = t0;
                    if (now - lastErr > std::chrono::seconds(2)) {
                        std::printf("\n⚠ 错误位置位（数据不可信，检查安装/磁环）\n");
                        lastErr = now;
                    }
                }
            } else {
                ++read_fail;
            }
        }

        // —— 每秒统计 ——
        if (now - last_stat >= std::chrono::seconds(1)) {
            stat_pub = published;  stat_fail = read_fail;
            published = 0;  read_fail = 0;
            last_stat = now;
            if (logging) logger.flush();
        }

        // —— 打印节流 200ms ——
        if (now - last_print >= kPrintPeriod) {
            last_print = now;
            if (have_sample) {
                std::printf("\r角度: %8.3f°  原始: %8u  %s%s 发布 %d Hz 读失败 %d  %s   ",
                            last.angle_deg, last.raw,
                            last.error ? "[错]" : "    ",
                            last.warning ? "[警]" : "    ",
                            stat_pub, stat_fail, logging ? "[日志]" : "      ");
            } else {
                std::printf("\r等待编码器回复...（读失败 %d）   ", stat_fail);
            }
            std::fflush(stdout);
        }

        // —— 按键 ——
        if (int c = getKey(); c != EOF && c != 0) {
            switch (c) {
            case 'q': case 'Q':
                running = false;
                break;
            case 'z': case 'Z':
                std::printf("\n[命令] 设零位...\n");
                if (enc.setZero()) std::printf("[信息] 设零位成功\n");
                else std::printf("[警告] 设零位未确认\n");
                next_sample = std::chrono::steady_clock::now();
                break;
            case 'w': case 'W':
                logging = !logging;
                std::printf("\n[命令] 日志 %s（record/encoder_can_bridge.csv）\n",
                            logging ? "开启" : "关闭");
                break;
            default:
                break;
            }
        }

        // —— 小睡避免 CPU 空转（远小于采样周期）——
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    std::printf("\n[退出] 停止接收线程...\n");
    rx_running = false;
    if (rx_thread.joinable()) rx_thread.join();

    return 0;
}
