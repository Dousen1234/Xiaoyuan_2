/**
 * @file taihu_keyboardctl.cpp
 * @brief 键盘交互式单腿四电机速度控制 demo。
 *
 * 操作方式：
 *   - 启动时先输入基础转速（rpm，**输出端**转速），运行中可随时用 Ctrl+V 重设；
 *   - Ctrl+1 / Ctrl+2 / Ctrl+3 / Ctrl+4 切换当前控制的电机（ID 1~4），
 *     命令行状态行常显当前控制的电机 ID；
 *   - 方向键 ↑（按住）：当前电机以设定转速**逆时针**转动；
 *     方向键 ↓（按住）：当前电机以设定转速**顺时针**转动；
 *     **只有按住时才转，松手即停**；
 *   - Ctrl+P：打印当前电机的 电压(V) / 电流(A) / 位置(相对零位, 度)；
 *   - h：打印帮助；Ctrl+C 或 q：退出（退出时停止全部电机并抱闸）。
 *
 * 键盘读取有两种后端，程序启动时自动选择：
 *   1) **evdev**（直接读 /dev/input/event*）：能拿到真实的「按下 / 松开」事件，
 *      松手立即停止，体验最好；需要当前用户属于 `input` 组（见 README）。
 *   2) **终端 raw 模式**（回退方案，普通用户即可运行）：终端只能收到按键字符，
 *      收不到「松开」事件，因此借助终端的**按键自动重复**（typematic repeat）
 *      推断是否仍按住：按下后先等待较长超时（覆盖默认约 500ms 的首次重复延迟），
 *      一旦收到重复字符就改用较短超时，超时未再收到即判定为松手并停机。
 *
 * 电机配置：ID 1~4，减速比 101 / 81 / 81 / 101（与其他单腿 demo 一致）。
 * CAN 总线：单条（运行时输入接口名，默认 can0）。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <linux/input.h>

#include "taihu/config.h"
#include "taihu/joint_module.h"
#include "taihu/socket_can.h"

namespace {

using taihu::JointModule;
using taihu::SocketCanInterface;

// =====================================================================
// 电机与总线配置
// =====================================================================

/// 一条腿的四个电机 CAN ID
const std::vector<uint32_t> kMotorIds = {1, 2, 3, 4};
/// 对应减速比（ID1/4 = 101，ID2/3 = 81），与 taihu_singleleg_step 一致
const std::vector<double> kMotorGearRatios = {101.0, 81.0, 81.0, 101.0};

constexpr int kMaxBusChannel = 6;                    ///< 鲲弘设备最多 6 路（can0~can5）
constexpr const char* kDefaultCanDevice = "can0";    ///< 默认 CAN 接口

// =====================================================================
// 速度参数
// =====================================================================

constexpr double kPi        = 3.14159265358979323846;
constexpr double kDefaultRpm = 10.0;    ///< 启动时的默认转速（输出端 rpm）
constexpr double kMinRpm     = 0.1;     ///< 允许的最小转速
constexpr double kMaxRpm     = 200.0;   ///< 允许的最大转速（安全上限）

/// rpm（输出端）-> rad/s（输出端）。
/// JointModule::setTargetVelocity() 内部会再按减速比换算成电机端 0.01Hz 编码。
double rpmToRadPerS(double rpm) { return rpm * 2.0 * kPi / 60.0; }

/// 弧度 -> 度，并归一化到 (-180, 180]，便于读「相对零位」的角度。
double radToDeg180(double rad) {
    double deg = std::fmod(rad * 180.0 / kPi, 360.0);
    if (deg > 180.0)  deg -= 360.0;
    if (deg <= -180.0) deg += 360.0;
    return deg;
}

// =====================================================================
// 速度环增益（默认不下发，用电机出厂默认值）
// =====================================================================

/// 是否在启动时下发速度环 PID 增益。
/// 默认 **false**：速度模式直接使用电机出厂增益，避免因增益不当导致飞车/抖动。
/// 若发现速度模式不转、抖动或超调明显，把它改为 true 并调整下面三个值后重新编译。
constexpr bool kApplyVelocityGains = false;
constexpr int32_t kVelocityKp = 1000;
constexpr int32_t kVelocityKi = 10;
constexpr int32_t kVelocityKd = 0;

/// 松手（或切换电机）后是否额外抱闸。
/// false（默认）：只把目标速度置 0，电机保持使能并锁在当前位置 —— 松手即停且无机械冲击；
/// true：额外发送停止命令（0x02）抱死刹车。
constexpr bool kBrakeOnRelease = false;

/// 控制线程下发速度指令的周期 (ms)：既作为心跳，也决定松手后的停机响应延迟。
constexpr int kVelSendPeriodMs = 10;

// =====================================================================
// 终端回退后端的「是否仍按住」判定超时 (ms)
// =====================================================================
// 终端按键自动重复（typematic）的典型默认值：首次重复延迟约 500ms，之后每约 30ms 重复一次。
// 所以「刚按下、还没收到重复」时必须用长超时（否则会在首次重复到来前误判松手），
// 收到重复后改用短超时，使松手能快速被检测到。

constexpr int kIdleTimeoutMs       = 200;  ///< 空闲（电机未转）时的轮询超时
constexpr int kHoldFirstTimeoutMs  = 700;  ///< 刚按下、尚未收到重复字符时的超时
constexpr int kHoldRepeatTimeoutMs = 150;  ///< 已收到重复字符后的超时（松手判定）
constexpr int kEscSeqTimeoutMs     = 50;   ///< 解析 ESC 转义序列时等待后续字节的超时

// =====================================================================
// 运行时共享状态（键盘线程写，控制线程读）
// =====================================================================

std::atomic<double> g_rpm{kDefaultRpm};   ///< 当前设定转速（输出端 rpm）
std::atomic<int>    g_direction{0};       ///< +1 逆时针 / -1 顺时针 / 0 停止
std::atomic<int>    g_motor_index{0};     ///< 当前控制的电机下标（0~3 -> ID 1~4）
std::atomic<bool>   g_print_request{false}; ///< Ctrl+P 请求标志（由控制线程执行读取与打印）
std::atomic<bool>   g_running{true};      ///< 程序运行标志

std::mutex g_out_mutex;                   ///< 保护 stdout（状态行与信息打印不交错）

// =====================================================================
// 按键抽象
// =====================================================================

/// 逻辑按键（屏蔽 evdev keycode 与终端字节流的差异）
enum class Key {
    None = 0,
    Up,        ///< 方向键上：逆时针
    Down,      ///< 方向键下：顺时针
    Motor1, Motor2, Motor3, Motor4,  ///< 切换控制电机 ID 1~4
    SetSpeed,  ///< Ctrl+V：重设转速
    PrintInfo, ///< Ctrl+P：打印电机信息
    Help,      ///< h：帮助
    Quit,      ///< Ctrl+C / q：退出
};

struct KeyEvent {
    Key  key     = Key::None;
    bool pressed = true;   ///< true = 按下（含自动重复），false = 松开
};

/// 键盘输入源抽象：evdev 与终端 raw 两种实现共用同一套主循环。
class KeyboardSource {
public:
    virtual ~KeyboardSource() = default;

    /// 是否能提供真实的「松开」事件（evdev 可以，终端回退不行）
    virtual bool hasReleaseEvents() const = 0;

    /// 等待一个按键事件，最多 timeout_ms；返回 false 表示超时无事件
    virtual bool poll(KeyEvent& ev, int timeout_ms) = 0;

    /// 临时把终端交还给标准输入（用于 scanf 读取转速）
    virtual void suspend() {}
    /// 恢复键盘捕获（并丢弃 suspend 期间堆积的按键）
    virtual void resume() {}
};

// =====================================================================
// 终端 raw 模式管理（RAII）
// =====================================================================

/// 关闭回显与行缓冲，避免方向键等按键在屏幕上留下 `^[[A` 乱码；
/// 析构或 leave() 时恢复原始终端属性（防止程序异常退出后终端不可用）。
class RawTerminal {
public:
    RawTerminal() = default;
    ~RawTerminal() { leave(); }

    RawTerminal(const RawTerminal&) = delete;
    RawTerminal& operator=(const RawTerminal&) = delete;

    /// 进入 raw 模式；stdin 不是 tty 时返回 false（调用方可选择忽略）
    bool enter() {
        if (active_) return true;
        if (!::isatty(STDIN_FILENO)) return false;
        if (::tcgetattr(STDIN_FILENO, &saved_) != 0) return false;

        struct termios raw = saved_;
        ::cfmakeraw(&raw);
        raw.c_cc[VMIN]  = 0;   // 非阻塞
        raw.c_cc[VTIME] = 0;
        if (::tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) return false;

        active_ = true;
        return true;
    }

    /// 恢复原始终端属性（可在信号处理中安全调用：tcsetattr 是 async-signal-safe）
    void leave() {
        if (!active_) return;
        ::tcsetattr(STDIN_FILENO, TCSAFLUSH, &saved_);
        active_ = false;
    }

    bool active() const { return active_; }

private:
    struct termios saved_{};
    bool active_ = false;
};

// =====================================================================
// 后端 1：evdev（真实按下 / 松开事件）
// =====================================================================

class EvdevKeyboard : public KeyboardSource {
public:
    /// 打开所有「具备方向键 + Ctrl」的 evdev 键盘设备（一台机器可能接多个键盘）；
    /// 一个都打不开（如当前用户不在 input 组）时返回 nullptr。
    static std::unique_ptr<EvdevKeyboard> create(std::vector<std::string>& out_devices) {
        std::unique_ptr<EvdevKeyboard> kb(new EvdevKeyboard());
        if (!kb->openKeyboards(out_devices)) return nullptr;
        kb->term_.enter();   // 尽力而为：关闭回显，避免按键在屏幕上留乱码
        return kb;
    }

    ~EvdevKeyboard() override {
        term_.leave();
        for (const Device& d : devices_)
            if (d.fd >= 0) ::close(d.fd);
    }

    bool hasReleaseEvents() const override { return true; }

    bool poll(KeyEvent& ev, int timeout_ms) override {
        if (paused_ || devices_.empty()) return false;

        std::vector<struct pollfd> pfds(devices_.size());
        for (size_t i = 0; i < devices_.size(); ++i) {
            pfds[i].fd      = devices_[i].fd;
            pfds[i].events  = POLLIN;
            pfds[i].revents = 0;
        }

        const int ret = ::poll(pfds.data(), pfds.size(), timeout_ms);
        if (ret <= 0) return false;

        // 逐个设备把待处理事件读空，返回第一个映射成功的按键
        for (size_t i = 0; i < devices_.size(); ++i) {
            if ((pfds[i].revents & POLLIN) == 0) continue;

            struct input_event ie;
            while (true) {
                const ssize_t n = ::read(devices_[i].fd, &ie, sizeof ie);
                if (n != static_cast<ssize_t>(sizeof ie)) break;   // EAGAIN 或出错
                if (ie.type != EV_KEY) continue;
                if (mapKey(devices_[i], ie.code, ie.value, ev)) return true;
            }
        }
        return false;
    }

    void suspend() override {
        paused_ = true;
        term_.leave();          // 交还终端，让用户能正常输入数字
    }

    void resume() override {
        drainEvents();          // 丢弃输入转速期间被 evdev 捕获的按键
        term_.enter();
        paused_ = false;
    }

private:
    EvdevKeyboard() = default;

    /// 一个 evdev 键盘设备。Ctrl 状态按设备分别记录，避免多键盘互相干扰。
    struct Device {
        int  fd    = -1;
        bool ctrl  = false;
        std::string name;
    };

    std::vector<Device> devices_;
    bool paused_ = false;
    RawTerminal term_;

    static bool testBit(unsigned long bit, const unsigned long* array) {
        return (array[bit / (8 * sizeof(unsigned long))]
                >> (bit % (8 * sizeof(unsigned long)))) & 1UL;
    }

    /// 扫描 /dev/input/event*，打开所有满足「有方向键上下 + Ctrl」的设备。
    /// 只打开键盘类设备，忽略鼠标 / 触摸板 / 手柄等，避免无关事件干扰。
    bool openKeyboards(std::vector<std::string>& out_devices) {
        constexpr size_t kBitLongs = (KEY_MAX / (8 * sizeof(unsigned long))) + 2;

        DIR* dir = ::opendir("/dev/input");
        if (!dir) return false;

        struct dirent* ent = nullptr;
        while ((ent = ::readdir(dir)) != nullptr) {
            if (std::strncmp(ent->d_name, "event", 5) != 0) continue;

            const std::string path = std::string("/dev/input/") + ent->d_name;
            const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd < 0) continue;   // 无权限（不在 input 组）时会失败

            unsigned long bits[kBitLongs];
            std::memset(bits, 0, sizeof bits);
            if (::ioctl(fd, EVIOCGBIT(EV_KEY, sizeof bits), bits) < 0) {
                ::close(fd);
                continue;
            }

            // 必备：方向键上下 + Ctrl（本 demo 的全部操作都依赖这些键）
            if (!testBit(KEY_UP, bits) || !testBit(KEY_DOWN, bits) ||
                !testBit(KEY_LEFTCTRL, bits)) {
                ::close(fd);
                continue;
            }

            char devname[256] = {0};
            if (::ioctl(fd, EVIOCGNAME(sizeof devname), devname) < 0) devname[0] = '\0';

            Device d;
            d.fd   = fd;
            d.name = devname[0] ? devname : path;
            devices_.push_back(d);
            out_devices.push_back(d.name);
        }
        ::closedir(dir);

        return !devices_.empty();
    }

    void drainEvents() {
        struct input_event ie;
        for (const Device& d : devices_) {
            if (d.fd < 0) continue;
            while (::read(d.fd, &ie, sizeof ie) == static_cast<ssize_t>(sizeof ie)) {}
        }
    }

    /// evdev keycode -> 逻辑按键；返回 false 表示该事件无需上报
    bool mapKey(Device& dev, uint16_t code, int32_t value, KeyEvent& ev) {
        // Ctrl 自身只更新该设备的状态，不上报
        if (code == KEY_LEFTCTRL || code == KEY_RIGHTCTRL) {
            dev.ctrl = (value != 0);   // 1 按下 / 2 重复 都算按住
            return false;
        }

        const bool is_repeat = (value == 2);
        const bool pressed   = (value != 0);   // 0 = 松开

        Key k = Key::None;
        switch (code) {
            case KEY_UP:   k = Key::Up;   break;
            case KEY_DOWN: k = Key::Down; break;
            case KEY_1: case KEY_KP1: k = Key::Motor1; break;
            case KEY_2: case KEY_KP2: k = Key::Motor2; break;
            case KEY_3: case KEY_KP3: k = Key::Motor3; break;
            case KEY_4: case KEY_KP4: k = Key::Motor4; break;
            case KEY_V: if (dev.ctrl) k = Key::SetSpeed;  break;
            case KEY_P: if (dev.ctrl) k = Key::PrintInfo; break;
            case KEY_C: if (dev.ctrl) k = Key::Quit;      break;
            case KEY_Q: k = Key::Quit; break;
            case KEY_H: k = Key::Help; break;
            default: return false;
        }
        if (k == Key::None) return false;

        // 方向键接受自动重复（用于维持「按住」状态）；
        // 动作类按键忽略重复，避免长按 Ctrl+P 时疯狂刷屏。
        if (is_repeat && k != Key::Up && k != Key::Down) return false;

        ev.key     = k;
        ev.pressed = pressed;
        return true;
    }
};

// =====================================================================
// 后端 2：终端 raw 模式（回退方案，普通用户可用）
// =====================================================================

class TermiosKeyboard : public KeyboardSource {
public:
    static std::unique_ptr<TermiosKeyboard> create() {
        std::unique_ptr<TermiosKeyboard> kb(new TermiosKeyboard());
        if (!kb->term_.enter()) return nullptr;   // stdin 不是 tty
        return kb;
    }

    ~TermiosKeyboard() override { term_.leave(); }

    bool hasReleaseEvents() const override { return false; }

    bool poll(KeyEvent& ev, int timeout_ms) override {
        uint8_t b = 0;
        if (!readByte(b, timeout_ms)) return false;

        ev.pressed = true;   // 终端拿不到松开事件，一律按「按下」上报

        if (b == 0x1B) {   // ESC：方向键转义序列，或 Ctrl+3（部分终端把 Ctrl+3 编码为 ESC）
            uint8_t b2 = 0;
            if (!readByte(b2, kEscSeqTimeoutMs)) { ev.key = Key::Motor3; return true; }
            if (b2 != '[') return false;   // 其他转义序列，忽略
            uint8_t b3 = 0;
            if (!readByte(b3, kEscSeqTimeoutMs)) return false;
            switch (b3) {
                case 'A': ev.key = Key::Up;   return true;
                case 'B': ev.key = Key::Down; return true;
                default:  return false;   // 左/右等本 demo 不用
            }
        }

        switch (b) {
            case 0x00: ev.key = Key::Motor2;   return true;  // Ctrl+2（xterm 系发 NUL）
            case 0x1C: ev.key = Key::Motor4;   return true;  // Ctrl+4
            case 0x16: ev.key = Key::SetSpeed; return true;  // Ctrl+V
            case 0x10: ev.key = Key::PrintInfo;return true;  // Ctrl+P
            case 0x03: ev.key = Key::Quit;     return true;  // Ctrl+C
            case '1':  ev.key = Key::Motor1;   return true;
            case '2':  ev.key = Key::Motor2;   return true;
            case '3':  ev.key = Key::Motor3;   return true;
            case '4':  ev.key = Key::Motor4;   return true;
            case 'q': case 'Q': ev.key = Key::Quit; return true;
            case 'h': case 'H': case '?': ev.key = Key::Help; return true;
            default: return false;
        }
    }

    void suspend() override { term_.leave(); }
    void resume() override  { ::tcflush(STDIN_FILENO, TCIFLUSH); term_.enter(); }

    /// 供信号处理恢复终端（tcsetattr 是 async-signal-safe）
    void restoreTerminal() { term_.leave(); }

private:
    TermiosKeyboard() = default;
    RawTerminal term_;

    /// 带超时读一个字节；返回 false 表示超时无输入
    bool readByte(uint8_t& out, int timeout_ms) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);

        struct timeval tv;
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        const int ret = ::select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0) return false;

        return ::read(STDIN_FILENO, &out, 1) == 1;
    }
};

/// 信号处理用：异常退出时也能把终端恢复成正常模式，避免用户终端卡在 raw 状态
TermiosKeyboard* g_termios_kb = nullptr;

void onSignal(int) {
    g_running.store(false);
    if (g_termios_kb) g_termios_kb->restoreTerminal();
}

// =====================================================================
// 屏幕输出
// =====================================================================

/// 绘制单行状态（\r 回到行首 + ANSI 清除到行尾），常显当前控制的电机 ID
void drawStatusLine() {
    const int idx = g_motor_index.load();
    const int dir = g_direction.load();
    const char* dir_str = (dir > 0) ? "↑ 逆时针转动中"
                        : (dir < 0) ? "↓ 顺时针转动中"
                                    : "停止（按住 ↑/↓ 转动）";

    std::lock_guard<std::mutex> lk(g_out_mutex);
    std::printf("\r\x1b[K[当前电机 ID=%u | 减速比 %.0f | 转速 %.1f rpm] %s",
                kMotorIds[idx], kMotorGearRatios[idx], g_rpm.load(), dir_str);
    std::fflush(stdout);
}

void printHelp() {
    std::lock_guard<std::mutex> lk(g_out_mutex);
    std::printf("\r\x1b[K\n");
    std::printf("----------------------------------------\n");
    std::printf("  快捷键一览\n");
    std::printf("----------------------------------------\n");
    std::printf("  Ctrl+V          重新设定转速（rpm，输出端）\n");
    std::printf("  Ctrl+1 / 1      切换控制电机 ID=1\n");
    std::printf("  Ctrl+2 / 2      切换控制电机 ID=2\n");
    std::printf("  Ctrl+3 / 3      切换控制电机 ID=3\n");
    std::printf("  Ctrl+4 / 4      切换控制电机 ID=4\n");
    std::printf("  ↑（按住）       当前电机逆时针转动，松手即停\n");
    std::printf("  ↓（按住）       当前电机顺时针转动，松手即停\n");
    std::printf("  Ctrl+P          打印当前电机 电压/电流/位置\n");
    std::printf("  h               显示本帮助\n");
    std::printf("  Ctrl+C / q      退出（停止全部电机并抱闸）\n");
    std::printf("----------------------------------------\n");
    std::fflush(stdout);
}

/// 打印当前电机信息（在控制线程中调用，独占 CAN 总线访问）
void printMotorInfo(std::vector<std::unique_ptr<JointModule>>& motors, int idx) {
    int32_t voltage_v  = 0;
    int32_t current_ma = 0;
    double  pos_rad    = 0.0;
    uint32_t err_bits  = 0;

    const bool ok_v = motors[idx]->readBusVoltage(voltage_v);
    const bool ok_c = motors[idx]->readCurrent(current_ma);
    const bool ok_p = motors[idx]->readPosition(pos_rad);
    const bool ok_e = motors[idx]->readErrorState(err_bits);

    {
        std::lock_guard<std::mutex> lk(g_out_mutex);
        std::printf("\r\x1b[K\n");
        std::printf("--------- 电机 ID=%u 当前信息 ---------\n", kMotorIds[idx]);
        if (ok_v) std::printf("  电压：%d V\n", voltage_v);
        else      std::printf("  电压：读取失败\n");
        if (ok_c) std::printf("  电流：%.3f A\n", current_ma / 1000.0);
        else      std::printf("  电流：读取失败\n");
        if (ok_p) std::printf("  位置（相对零位）：%.2f °\n", radToDeg180(pos_rad));
        else      std::printf("  位置（相对零位）：读取失败\n");
        if (ok_e) std::printf("  错误状态：0x%X%s\n", err_bits,
                              err_bits == 0 ? "（无错误）" : "");
        std::printf("---------------------------------------\n");
        std::fflush(stdout);
    }
    drawStatusLine();   // 信息块之后把状态行补回来
}

// =====================================================================
// 控制线程：独占 CAN 总线，按当前状态周期性下发速度指令
// =====================================================================

void controlThread(std::vector<std::unique_ptr<JointModule>>& motors) {
    int last_idx = -1;
    int last_dir = 0;

    while (g_running.load()) {
        const int    idx = g_motor_index.load();
        const int    dir = g_direction.load();
        const double rpm = g_rpm.load();

        // 需要让上一个电机停下来的两种情况：切换了电机，或刚从转动变为松手
        const bool need_stop_last =
            (last_idx >= 0) && ((last_idx != idx) || (last_dir != 0 && dir == 0));

        if (need_stop_last) {
            motors[last_idx]->setTargetVelocity(0.0);
            if (kBrakeOnRelease) motors[last_idx]->stop();
        }

        if (dir != 0) {
            // 周期性重发速度指令（心跳），保证驱动器不会因通信超时自行停机
            motors[idx]->setTargetVelocity(dir * rpmToRadPerS(rpm));
        }

        last_idx = idx;
        last_dir = dir;

        // Ctrl+P 的读取与打印放在本线程执行，避免与速度指令争抢 CAN 总线
        if (g_print_request.exchange(false)) {
            printMotorInfo(motors, idx);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kVelSendPeriodMs));
    }

    // 退出：停止全部电机并抱闸
    for (auto& m : motors) {
        m->setTargetVelocity(0.0);
        m->stop();
    }
}

// =====================================================================
// 交互动作
// =====================================================================

/// 切换当前控制的电机（切换瞬间强制停止，避免带着转速换电机）
void switchMotor(int new_idx) {
    if (new_idx == g_motor_index.load()) return;
    g_direction.store(0);
    g_motor_index.store(new_idx);

    {
        std::lock_guard<std::mutex> lk(g_out_mutex);
        std::printf("\r\x1b[K[信息] 已切换控制电机 -> ID=%u（减速比 %.0f）\n",
                    kMotorIds[new_idx], kMotorGearRatios[new_idx]);
        std::fflush(stdout);
    }
    drawStatusLine();
}

/// Ctrl+V：重新设定转速。期间暂停键盘捕获并强制电机停止。
void doSetSpeed(KeyboardSource& kb) {
    g_direction.store(0);   // 输入期间必须停机
    std::this_thread::sleep_for(std::chrono::milliseconds(kVelSendPeriodMs * 3));

    kb.suspend();   // 交还终端，恢复正常行输入

    {
        std::lock_guard<std::mutex> lk(g_out_mutex);
        std::printf("\r\x1b[K[重新定义速度] 请输入转速（rpm，输出端，范围 %.1f~%.1f，当前 %.1f）：",
                    kMinRpm, kMaxRpm, g_rpm.load());
        std::fflush(stdout);
    }

    double v = 0.0;
    const int scanned = std::scanf("%lf", &v);
    // 清掉本行残留字符，避免影响后续按键解析
    int c = 0;
    while ((c = std::getchar()) != '\n' && c != EOF) {}

    {
        std::lock_guard<std::mutex> lk(g_out_mutex);
        if (scanned == 1 && v >= kMinRpm && v <= kMaxRpm) {
            g_rpm.store(v);
            std::printf("[信息] 转速已更新为 %.1f rpm\n", v);
        } else {
            std::printf("[提示] 输入无效或超出范围（%.1f~%.1f），保持 %.1f rpm\n",
                        kMinRpm, kMaxRpm, g_rpm.load());
        }
        std::fflush(stdout);
    }

    kb.resume();
    drawStatusLine();
}

// =====================================================================
// 主循环
// =====================================================================

/// 通用按键动作处理（方向键除外）；返回 false 表示请求退出
bool handleAction(Key key, KeyboardSource& kb) {
    switch (key) {
        case Key::Motor1: switchMotor(0); return true;
        case Key::Motor2: switchMotor(1); return true;
        case Key::Motor3: switchMotor(2); return true;
        case Key::Motor4: switchMotor(3); return true;
        case Key::SetSpeed:  doSetSpeed(kb);        return true;
        case Key::PrintInfo: g_print_request.store(true); return true;
        case Key::Help:      printHelp(); drawStatusLine(); return true;
        case Key::Quit:      return false;
        default:             return true;
    }
}

/// evdev 后端：有真实按下/松开事件，直接按事件驱动
void runLoopWithRelease(KeyboardSource& kb) {
    while (g_running.load()) {
        KeyEvent ev;
        if (!kb.poll(ev, kIdleTimeoutMs)) {
            drawStatusLine();   // 定时刷新状态行（转速可能已改变）
            continue;
        }

        switch (ev.key) {
            case Key::Up:
                if (ev.pressed) g_direction.store(+1);
                else if (g_direction.load() == +1) g_direction.store(0);
                break;
            case Key::Down:
                if (ev.pressed) g_direction.store(-1);
                else if (g_direction.load() == -1) g_direction.store(0);
                break;
            default:
                if (ev.pressed && !handleAction(ev.key, kb)) {
                    g_running.store(false);
                    return;
                }
                break;
        }
        drawStatusLine();
    }
}

/// 终端回退后端：靠自动重复推断「是否仍按住」
/// - 刚按下、还没收到重复字符时用长超时（覆盖终端首次重复延迟，默认约 500ms）；
/// - 收到重复字符后改用短超时，松手即可快速判定并停机。
void runLoopWithoutRelease(KeyboardSource& kb) {
    bool got_repeat = false;

    while (g_running.load()) {
        const int dir = g_direction.load();
        int timeout = kIdleTimeoutMs;
        if (dir != 0) timeout = got_repeat ? kHoldRepeatTimeoutMs : kHoldFirstTimeoutMs;

        KeyEvent ev;
        if (!kb.poll(ev, timeout)) {
            // 超时未收到重复字符 => 判定为松手，停机
            if (dir != 0) {
                g_direction.store(0);
                got_repeat = false;
            }
            drawStatusLine();
            continue;
        }

        switch (ev.key) {
            case Key::Up: {
                if (g_direction.load() == +1) {
                    got_repeat = true;              // 自动重复：仍按住
                } else {
                    g_direction.store(+1);
                    got_repeat = false;             // 新按下：等待首次重复
                }
                break;
            }
            case Key::Down: {
                if (g_direction.load() == -1) {
                    got_repeat = true;
                } else {
                    g_direction.store(-1);
                    got_repeat = false;
                }
                break;
            }
            default:
                got_repeat = false;
                if (!handleAction(ev.key, kb)) {
                    g_running.store(false);
                    return;
                }
                break;
        }
        drawStatusLine();
    }
}

/// 输入一个正整数，带默认值与范围检查
int inputInt(const char* prompt, int def, int lo, int hi) {
    std::printf("%s", prompt);
    int v = def;
    if (std::scanf("%d", &v) != 1 || v < lo || v > hi) {
        std::printf("[提示] 输入无效，使用默认值 %d\n", def);
        return def;
    }
    return v;
}

/// 归一化接口名：纯数字 0~5 自动补全为 canN
std::string normalizeIfname(const char* raw) {
    char* end = nullptr;
    const long v = std::strtol(raw, &end, 10);
    if (end != raw && *end == '\0' && v >= 0 && v < kMaxBusChannel) {
        char buf[16];
        std::snprintf(buf, sizeof buf, "can%ld", v);
        return buf;
    }
    return raw;
}

} // namespace

int main() {
    std::printf("========================================\n");
    std::printf("  键盘控制单腿四电机 demo (taihu_keyboardctl)\n");
    std::printf("========================================\n");
    std::printf("  一条腿：电机 ID 1~4（减速比 101/81/81/101）\n");
    std::printf("  ↑ 按住逆时针转，↓ 按住顺时针转，松手即停\n");
    std::printf("  Ctrl+V 改转速 | Ctrl+1~4 换电机 | Ctrl+P 看信息 | q 退出\n");
    std::printf("----------------------------------------\n");

    // 1. 选择 CAN 接口
    char ifbuf[32] = {0};
    std::printf("请输入 CAN 接口名或通道号（默认 %s）：", kDefaultCanDevice);
    if (std::scanf("%31s", ifbuf) != 1 || ifbuf[0] == '\0')
        std::snprintf(ifbuf, sizeof ifbuf, "%s", kDefaultCanDevice);
    const std::string ifname = normalizeIfname(ifbuf);

    // 2. 打开 CAN 设备
    SocketCanInterface can;
    if (!can.open(ifname.c_str(), 0)) {
        std::printf("[错误] 打开 %s 失败，请确认已接上鲲弘 CANFD 模块且接口已 up"
                    "（sudo ./build/kcanctl up %s 1000000）\n",
                    ifname.c_str(), ifname.c_str());
        return -1;
    }
    std::printf("[信息] 已打开 %s\n", ifname.c_str());

    // 3. 初始化四个电机
    std::vector<std::unique_ptr<JointModule>> motors;
    motors.reserve(kMotorIds.size());
    for (size_t i = 0; i < kMotorIds.size(); ++i) {
        auto m = std::make_unique<JointModule>(can, kMotorIds[i], kMotorIds[i]);
        m->setGearRatio(kMotorGearRatios[i]);
        m->clearError();
        if (kApplyVelocityGains) {
            m->setVelocityKp(kVelocityKp);
            m->setVelocityKi(kVelocityKi);
            m->setVelocityKd(kVelocityKd);
        }
        motors.push_back(std::move(m));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::printf("[信息] 四个电机初始化完成（ID 1~4）\n");

    // 4. 输入初始转速
    std::printf("请输入初始转速（rpm，输出端，范围 %.1f~%.1f，默认 %.1f）：",
                kMinRpm, kMaxRpm, kDefaultRpm);
    double rpm = kDefaultRpm;
    if (std::scanf("%lf", &rpm) != 1 || rpm < kMinRpm || rpm > kMaxRpm) {
        std::printf("[提示] 输入无效或超出范围，使用默认值 %.1f rpm\n", kDefaultRpm);
        rpm = kDefaultRpm;
    }
    g_rpm.store(rpm);
    // 清空 scanf 残留的换行，避免影响后续键盘读取
    int c = 0;
    while ((c = std::getchar()) != '\n' && c != EOF) {}

    // 5. 创建键盘后端：evdev 优先（可精确检测松手），失败则回退终端 raw 模式
    std::vector<std::string> evdev_devices;
    std::unique_ptr<KeyboardSource> kb = EvdevKeyboard::create(evdev_devices);
    if (kb) {
        std::printf("[信息] 键盘后端：evdev（/dev/input）—— 支持真实按下/松开，松手立即停止\n");
        for (const std::string& dev : evdev_devices)
            std::printf("       已监听键盘设备：%s\n", dev.c_str());
    } else {
        auto tkb = TermiosKeyboard::create();
        if (!tkb) {
            std::printf("[错误] 无法初始化键盘：stdin 不是终端，且 /dev/input 不可读\n");
            can.close();
            return -1;
        }
        g_termios_kb = tkb.get();
        kb = std::move(tkb);

        std::printf("[提示] 键盘后端：终端 raw 模式（未能读取 /dev/input，普通用户默认如此）\n");
        std::printf("       该模式收不到真实「松开」事件，改用按键自动重复判定松手，\n");
        std::printf("       松手后停机延迟约 %d ms。\n", kHoldRepeatTimeoutMs);
        std::printf("       如需松手立即停止，可把自己加入 input 组后注销重登：\n");
        std::printf("         sudo usermod -aG input $USER\n");
    }

    // 6. 注册信号处理，保证异常退出时终端能恢复正常
    struct sigaction sa{};
    sa.sa_handler = onSignal;
    ::sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    ::sigaction(SIGINT, &sa, nullptr);
    ::sigaction(SIGTERM, &sa, nullptr);

    printHelp();
    drawStatusLine();

    // 7. 启动控制线程，进入键盘主循环
    std::thread ctrl_thread(controlThread, std::ref(motors));

    if (kb->hasReleaseEvents()) runLoopWithRelease(*kb);
    else                        runLoopWithoutRelease(*kb);

    // 8. 退出清理
    g_running.store(false);
    g_direction.store(0);
    ctrl_thread.join();

    kb.reset();               // 恢复终端属性
    g_termios_kb = nullptr;
    can.close();

    std::printf("\n[信息] 已退出，全部电机停止并抱闸\n");
    return 0;
}
