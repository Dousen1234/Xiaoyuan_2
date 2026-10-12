/**
 * @file taihu_mpt_encoder.cpp
 * @brief 金钢 MPT 磁编码器独立查看工具（RS485 /dev/ttyUSB*，与 CAN 电机总线完全无关）。
 *
 * 功能：
 *   - 自动探测协议（BUS 款带地址 / RS485 款单机）；
 *   - 连续读取并显示角度（度）、原始值、状态位、读取频率；
 *   - `z` 设置零位（交替发送序列，自动等待生效）；
 *   - `d` 改设备地址（仅 BUS 款，改完自动用新地址继续轮询）；
 *   - `w` 开关 CSV 日志（record/encoder_log.csv）；
 *   - `q` 退出。
 *
 * 接线（手册 v0.6 线序）：
 *   红  = +5V   黑 = 0V(GND)   蓝 = A(+)   绿 = B(-)
 *   编码器端终端电阻已内置；USB 转 485 适配器若不带端接，需在控制器侧 A/B 间并 120Ω。
 *   两台共总线：仅 BUS(D) 款支持（先各自单独接、用本工具 `d` 改成不同地址）。
 *
 * 用法：
 *   ./build/taihu_mpt_encoder /dev/ttyUSB0 2500000        # 设备 + 波特率（默认 2.5M）
 *   ./build/taihu_mpt_encoder /dev/ttyUSB0 115200 0x1F    # 波特率 + BUS 地址（可选）
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <termios.h>
#include <unistd.h>

#include "taihu/mpt_encoder.h"

namespace {

using taihu::MptEncoder;
using taihu::Rs485Port;

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
// CSV 日志
// =====================================================================
class EncoderLogger {
public:
    explicit EncoderLogger(const std::string& path) {
        file_ = std::fopen(path.c_str(), "w");
        if (file_) std::fprintf(file_, "timestamp_s,angle_deg,raw,status,error,warning\n");
    }
    ~EncoderLogger() { if (file_) std::fclose(file_); }
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

/// 单位圈角度差（处理 360° 环绕）
double wrapDeg(double a) {
    while (a >= 360.0) a -= 360.0;
    while (a < 0.0) a += 360.0;
    return a;
}

} // namespace

int main(int argc, char** argv) {
    // —— 参数解析 ——
    const char* device = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    int baud = (argc > 2) ? std::atoi(argv[2]) : 2500000;
    uint8_t force_addr = 0xFF;  // 0xFF = 不强制（自动探测）
    if (argc > 3) force_addr = static_cast<uint8_t>(std::strtoul(argv[3], nullptr, 0));

    std::printf("==============================================\n");
    std::printf("  金钢 MPT 磁编码器独立查看工具 (手册 v0.6)\n");
    std::printf("==============================================\n");
    std::printf("  串口: %s   波特率: %d\n", device, baud);
    std::printf("  线序: 红=+5V 黑=GND 蓝=A 绿=B（编码器端已内置端接）\n");
    std::printf("  命令: z=设零位  d=改地址(BUS款)  w=日志开关  q=退出\n");
    std::printf("----------------------------------------------\n");

    // —— 打开串口 ——
    Rs485Port port;
    std::string err;
    if (!port.open(device, baud, &err)) {
        std::printf("✗ 打开串口失败: %s\n", err.c_str());
        std::printf("  插上 USB 转 485 后用 `ls /dev/ttyUSB*` 确认设备名\n");
        return 1;
    }

    // —— 探测协议 / 地址 ——
    uint8_t found_addr = MptEncoder::kFactoryAddr;
    std::printf("正在探测编码器（BUS 0x1F / RS485）...\n");
    MptEncoder::Protocol proto = MptEncoder::probe(port, &found_addr,
                                                    MptEncoder::Output::Single24);
    if (force_addr != 0xFF) found_addr = force_addr;

    // 用探测到的参数试读一次确认
    MptEncoder enc(port, proto, found_addr);
    MptEncoder::Sample s;
    bool ok = enc.readPosition(s);
    const char* proto_name = (proto == MptEncoder::Protocol::BUS) ? "BUS(D款)" : "RS485(R款)";
    if (!ok && force_addr == 0xFF) {
        std::printf("⚠ 探测未确认（%s）——可能波特率不符或未接线。按探测结果继续轮询...\n", proto_name);
    } else {
        std::printf("✓ 编码器在线: %s", proto_name);
        if (proto == MptEncoder::Protocol::BUS)
            std::printf("  地址=0x%02X", found_addr);
        std::printf("\n");
    }

    // —— 终端 raw 模式 ——
    RawTerminal term;
    if (!term.enter()) std::printf("⚠ 终端非交互，跳过按键功能\n");

    EncoderLogger logger("record/encoder_log.csv");
    bool logging = false;

    auto t0 = std::chrono::steady_clock::now();
    auto lastPrint = t0;
    auto lastSample = t0;
    int samples = 0;
    double hz = 0.0;
    MptEncoder::Sample last{};
    bool have_sample = false;

    bool running = true;
    while (running) {
        auto now = std::chrono::steady_clock::now();

        // —— 读取 ——
        MptEncoder::Sample cur;
        if (enc.readPosition(cur)) {
            ++samples;
            last = cur;
            have_sample = true;
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
        }

        // —— 频率统计（每秒）——
        if (now - lastSample >= std::chrono::seconds(1)) {
            hz = samples;
            samples = 0;
            lastSample = now;
            if (logging) logger.flush();
        }

        // —— 打印节流 200ms ——
        if (now - lastPrint >= kPrintPeriod) {
            lastPrint = now;
            if (have_sample) {
                std::printf("\r角度: %8.3f°  原始: %8u  状态: 0x%02X  %s%s %.1f Hz  %s   ",
                            last.angle_deg, last.raw, last.status,
                            last.error ? "[错]" : "    ",
                            last.warning ? "[警]" : "    ",
                            hz, logging ? "[日志]" : "      ");
            } else {
                std::printf("\r等待编码器回复...（%.1f Hz 尝试）   ", hz);
            }
            std::fflush(stdout);
        }

        // —— 按键（非阻塞） ——
        int key = getKey();
        if (key == 'q' || key == 3 /* Ctrl+C */) break;
        switch (key) {
            case 'z': {
                std::printf("\n设零位中（交替序列 10 次，约 0.5s）...\n");
                if (enc.setZero())
                    std::printf("✓ 零位已设置\n");
                else
                    std::printf("✗ 零位设置失败（检查通信）\n");
                break;
            }
            case 'd': {
                if (enc.protocol() != MptEncoder::Protocol::BUS) {
                    std::printf("\nRS485(R款)无地址概念，跳过\n");
                    break;
                }
                std::printf("\n输入新地址 (0~31，十六进制如 0x05): ");
                term.leave();
                char line[32] = {0};
                if (std::fgets(line, sizeof(line), stdin)) {
                    uint8_t na = static_cast<uint8_t>(std::strtoul(line, nullptr, 0));
                    if (enc.setBusAddress(na))
                        std::printf("✓ 地址已改为 0x%02X\n", na);
                    else
                        std::printf("✗ 改地址失败\n");
                }
                term.enter();
                break;
            }
            case 'w': {
                logging = !logging;
                std::printf("\n%s record/encoder_log.csv\n", logging ? "✓ 开始写" : "✗ 停止写");
                break;
            }
            default: break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::printf("\n退出。\n");
    return 0;
}
