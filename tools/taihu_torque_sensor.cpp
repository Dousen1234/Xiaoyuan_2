/**
 * @file taihu_torque_sensor.cpp
 * @brief 坤维扭矩传感器独立查看工具（与任意主控制程序共用 CAN 总线、互不干扰）。
 *
 * 用途：
 *   - 与电机主控制程序（demo / ROS2 节点）**同时**运行在同一条 CAN 总线上；
 *   - 随时开始 / 停止使用传感器（`s` 启动 100Hz 连续输出，`t` 停止传感器输出）；
 *   - 实时显示扭矩（已减零偏）/ 原始值 / 桥压 / 采样率 / 零偏；
 *   - `z` 重新做零偏校准（默认采 100 个样本取平均）；
 *   - `w` 开始/停止写 CSV 日志（record/torque_log.csv，覆盖模式）；
 *   - `q` 退出（自动停止传感器输出）。
 *
 * 互不干扰的原理（双向帧甄别）：
 *   - 本工具只处理 Tx_ID（默认 0x15）的 7 字节数据帧，电机帧一律忽略；
 *   - JointModule::receiveMotorFrame 只认电机 rx_id 的响应帧，传感器帧被忽略；
 *   - 两个进程各开各的 CAN raw socket，Linux 内核会给每个 socket 各投一份帧。
 *
 * 用法：
 *   ./build/taihu_torque_sensor [接口名]        # 默认 can0
 *   ./build/taihu_torque_sensor can0 0x10 0x15  # 显式指定 Rx/Tx ID
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <termios.h>
#include <unistd.h>

#include "taihu/socket_can.h"
#include "taihu/torque_sensor.h"

namespace {

using taihu::CanFrame;
using taihu::SocketCanInterface;
using taihu::TorqueSensor;

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

/// 非阻塞读一个键（raw 模式下）；无按键返回 0
int getKey() {
    int c = ::getchar();
    return c;
}

// =====================================================================
// CSV 日志（简单追加行，覆盖模式打开）
// =====================================================================
class TorqueLogger {
public:
    explicit TorqueLogger(const std::string& path) {
        file_ = std::fopen(path.c_str(), "w");
        if (file_) std::fprintf(file_, "timestamp_s,torque_nm,torque_raw_nm,bridge_mv_per_v,sync\n");
    }
    ~TorqueLogger() { if (file_) std::fclose(file_); }
    bool ok() const { return file_ != nullptr; }
    void log(double t, const TorqueSensor::Sample& s) {
        if (!file_) return;
        std::fprintf(file_, "%.6f,%.6f,%.6f,%.6f,%u\n",
                     t, s.torque_nm, s.torque_raw_nm, s.bridge_mv_per_v,
                     static_cast<unsigned>(s.sync));
    }
    void flush() { if (file_) std::fflush(file_); }
private:
    std::FILE* file_ = nullptr;
};

/// 主线程固定周期的打印节流
constexpr auto kPrintPeriod = std::chrono::milliseconds(200);

} // namespace

int main(int argc, char** argv) {
    // —— 参数解析：接口名（默认 can0）、可选 Rx/Tx ID ——
    const char* ifname = (argc > 1) ? argv[1] : "can0";
    uint32_t rx_id = 0x10, tx_id = 0x15;
    if (argc > 3) {
        rx_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 0));
        tx_id = static_cast<uint32_t>(std::strtoul(argv[3], nullptr, 0));
    }

    std::printf("==============================================\n");
    std::printf("  坤维扭矩传感器独立查看工具 (HIT-V2.0)\n");
    std::printf("==============================================\n");
    std::printf("  接口: %s   Rx_ID=0x%02X  Tx_ID=0x%02X\n", ifname, rx_id, tx_id);
    std::printf("  可与任意主控制程序同时运行（双向帧甄别，互不干扰）\n");
    std::printf("  命令: s=开始采集(100Hz)  t=停止采集  z=零偏校准\n");
    std::printf("        w=写日志开关  q=退出\n");
    std::printf("----------------------------------------------\n");

    // —— 打开 CAN（普通用户即可，接口需已 up）——
    SocketCanInterface can;
    if (!can.open(ifname, 1000000)) {
        std::fprintf(stderr, "[错误] 打开 %s 失败（先在外部终端 sudo ./build/kcanctl up %s 1000000）\n",
                     ifname, ifname);
        return 1;
    }

    TorqueSensor sensor(can, rx_id, tx_id);
    sensor.startRxThread();

    RawTerminal term;
    const bool raw_ok = term.enter();
    if (!raw_ok) std::printf("[提示] stdin 非 tty，改为逐行命令模式\n");

    // —— 先做一次单次查询确认传感器在线 ——
    if (!sensor.requestSingle()) {
        std::fprintf(stderr, "[错误] 发送单次查询命令失败\n");
        return 1;
    }
    TorqueSensor::Sample s;
    bool online = false;
    for (int i = 0; i < 20 && !online; ++i) {   // 等最多 200ms
        if (sensor.latest(s)) online = true;
        else std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (online) {
        std::printf("[信息] 传感器在线：原始扭矩 %.4f Nm，桥压 %.6f mV/V\n",
                    s.torque_raw_nm, s.bridge_mv_per_v);
        std::printf("[信息] 提示：上电有初值漂移，建议空载时按 z 做零偏校准\n");
    } else {
        std::printf("[警告] 200ms 内未收到回复——检查传感器供电/接线/ID/波特率\n");
        std::printf("[提示]   （本工具不退出，可继续尝试 s 开始采集）\n");
    }

    // —— 主循环：按键命令 + 定期打印 ——
    std::unique_ptr<TorqueLogger> logger;
    bool logging = false;
    bool running = true;
    bool collecting = false;
    const auto t0 = std::chrono::steady_clock::now();
    auto next_print = std::chrono::steady_clock::now() + kPrintPeriod;

    while (running) {
        // 1) 处理按键
        if (int c = getKey(); c != EOF && c != 0) {
            switch (c) {
            case 'q': case 'Q':
                running = false;
                break;
            case 's': case 'S':
                if (!sensor.startContinuous(TorqueSensor::OutputRate::Hz100)) {
                    std::printf("[命令] 开始采集：发送失败\n");
                } else {
                    collecting = true;
                    std::printf("[命令] 已发送 100Hz 连续输出命令 (48 AA 00 0D 0A)\n");
                }
                break;
            case 't': case 'T':
                if (sensor.stopOutput()) {
                    collecting = false;
                    std::printf("[命令] 已发送停止输出命令 (43 AA 0D 0A)\n");
                } else {
                    std::printf("[命令] 停止采集：发送失败\n");
                }
                break;
            case 'z': case 'Z':
                sensor.startZeroCalib(100);
                std::printf("[命令] 零偏校准开始：采 100 个样本取平均（保持空载静止）\n");
                break;
            case 'w': case 'W':
                if (!logging) {
                    logger = std::make_unique<TorqueLogger>("record/torque_log.csv");
                    if (logger->ok()) {
                        logging = true;
                        std::printf("[命令] 开始写日志 record/torque_log.csv（覆盖模式）\n");
                    } else {
                        std::printf("[命令] 打开日志文件失败（record/ 目录存在？）\n");
                        logger.reset();
                    }
                } else {
                    logger->flush();
                    logger.reset();
                    logging = false;
                    std::printf("[命令] 停止写日志\n");
                }
                break;
            case '\x03': // Ctrl+C
                running = false;
                break;
            default:
                break; // 忽略其它键
            }
        }

        // 2) 定期打印当前状态
        auto now = std::chrono::steady_clock::now();
        if (now >= next_print) {
            next_print = now + kPrintPeriod;
            TorqueSensor::Sample s2;
            const bool fresh = sensor.latest(s2, 250);
            const int sps = sensor.samplesPerSecond();
            if (fresh) {
                std::printf("\r%s  扭矩 %+.4f Nm  原始 %+.4f Nm  桥压 %.6f mV/V  采样 %d/s  零偏 %.4f",
                            collecting ? "[采集中]" : "[已停止]",
                            s2.torque_nm, s2.torque_raw_nm, s2.bridge_mv_per_v,
                            sps, sensor.zeroOffsetNm());
            } else {
                std::printf("\r%s  扭矩    --    Nm  采样 %d/s  零偏 %.4f   ",
                            collecting ? "[采集中]" : "[已停止]",
                            sps, sensor.zeroOffsetNm());
            }
            std::fflush(stdout);

            // 3) 日志（逐新鲜样本写入）
            if (logging && logger && fresh) {
                const double t = std::chrono::duration<double>(now - t0).count();
                logger->log(t, s2);
                logger->flush();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // —— 清理：停传感器输出、停接收线程、恢复终端 ——
    sensor.stopOutput();
    sensor.stopRxThread();
    term.leave();
    if (logger) logger.reset();
    std::printf("\n[信息] 已退出（传感器已停止输出）\n");
    return 0;
}
