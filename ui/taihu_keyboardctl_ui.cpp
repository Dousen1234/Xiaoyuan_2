/**
 * @file taihu_keyboardctl_ui.cpp
 * @brief taihu_keyboardctl 的 **Qt5 图形界面版本**：用窗口控件控制一条腿的四个电机。
 *
 * 与命令行版 `taihu_keyboardctl` 功能对应关系：
 *
 * | 命令行版             | 图形界面版                                    |
 * |----------------------|-----------------------------------------------|
 * | 启动输入 CAN 接口     | 顶部「连接」区下拉框（自动枚举 can*）+ 连接按钮 |
 * | 启动输入 / Ctrl+V 改转速 | 「转速设定」数字框 + 滑块 + 快捷按钮         |
 * | Ctrl+1~4 切电机       | 「电机选择」四个单选按钮（显示 ID 与减速比）     |
 * | ↑/↓ 按住转动松手即停   | 「运动控制」两个大按钮（按住转动）+ 键盘 ↑/↓     |
 * | Ctrl+P 打印信息       | 右侧「电机状态」面板自动周期刷新 + 手动刷新按钮  |
 * | q / Ctrl+C 退出       | 关闭窗口 / 急停按钮 / Esc                       |
 *
 * 线程模型（关键设计）：
 *   - **CAN 总线半双工且必须串行访问**，因此所有 CAN 收发都集中在一个 `MotorWorker`
 *     线程里（`QThread` + 事件循环 + 10ms `QTimer`），UI 线程绝不直接碰总线；
 *   - UI → worker 的「运动命令」（转速/电机/方向）通过 `std::atomic` 传递，
 *     **不走事件队列**，保证按下/松手能在一个 tick（≤10ms）内生效；
 *   - worker → UI 的状态快照通过 Qt 信号（跨线程自动 QueuedConnection）回传；
 *   - 转动期间默认**不读状态**（可在界面勾选打开），避免阻塞式读取拖慢速度心跳，
 *     从而保证「松手即停」和急停的响应时间稳定在 10ms 量级。
 *
 * 安全设计（多重兜底，任何一条都能让电机停下）：
 *   1. 按钮 released() / 键盘 keyRelease 事件 → 方向置 0；
 *   2. 看门狗定时器：若方向非 0 但鼠标已松开且无键盘按住 → 强制置 0；
 *   3. 窗口失焦（WindowDeactivate）→ 立即停止运动；
 *   4. 急停按钮 → 方向置 0 + 置急停闩锁，worker 对全部电机发 0x02（抱闸）；
 *   5. 关闭窗口 / 程序退出 → 全部电机置速度 0 并抱闸；
 *   6. 转速上限 kMaxRpm，且弹出任何对话框前先停止运动。
 *
 * 电机配置：ID 1~4，减速比 101 / 81 / 81 / 101（与命令行版完全一致）。
 * 构建：需要 Qt5 Widgets（CMake 里自动检测，未装 Qt5 时跳过该目标）。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <dirent.h>

#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QRadioButton>
#include <QShortcut>
#include <QSlider>
#include <QSpinBox>
#include <QTextEdit>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "taihu/config.h"
#include "taihu/joint_module.h"
#include "taihu/socket_can.h"
#include "taihu/taihu_tools.h"

// =====================================================================
// 公共常量与换算（与 examples/taihu_keyboardctl.cpp 保持一致）
// =====================================================================

namespace taihu_ui {

using taihu::JointModule;
using taihu::SocketCanInterface;

/// 一条腿的四个电机 CAN ID
static const std::vector<uint32_t> kMotorIds = {1, 2, 3, 4};
/// 对应减速比（ID1/4 = 101，ID2/3 = 81），与 taihu_singleleg_step 一致
static const std::vector<double> kMotorGearRatios = {101.0, 81.0, 81.0, 101.0};

static constexpr int kMaxBusChannel = 6;                 ///< 鲲弘设备最多 6 路（can0~can5）
static constexpr const char* kDefaultCanDevice = "can0"; ///< 默认 CAN 接口

static constexpr double kPi         = 3.14159265358979323846;
static constexpr double kDefaultRpm = 10.0;   ///< 启动默认转速（输出端 rpm）
static constexpr double kMinRpm     = 0.1;    ///< 允许最小转速
static constexpr double kMaxRpm     = 200.0;  ///< 允许最大转速（安全上限）

/// worker 线程 tick 周期 (ms)：既是速度指令心跳周期，也决定松手/急停的最大响应延迟
static constexpr int kVelSendPeriodMs = 10;

/// 默认状态刷新周期 (ms)。CAN 半双工，读取会占用总线，不宜过快。
static constexpr int kDefaultStatusPeriodMs = 200;

/// 看门狗周期 (ms)：定期检查「是否还真的按住」，兜住丢失 released 事件的情况
static constexpr int kWatchdogPeriodMs = 80;

/// rpm（输出端）-> rad/s（输出端）。
/// JointModule::setTargetVelocity() 内部再按减速比换算成电机端 0.01Hz 编码。
inline double rpmToRadPerS(double rpm) { return rpm * 2.0 * kPi / 60.0; }

/// 弧度 -> 度，并归一化到 (-180, 180]，便于读「相对零位」的角度。
inline double radToDeg180(double rad) {
    double deg = std::fmod(rad * 180.0 / kPi, 360.0);
    if (deg > 180.0)  deg -= 360.0;
    if (deg <= -180.0) deg += 360.0;
    return deg;
}

/// 日志级别（作为 int 跨线程传递）
enum LogLevel {
    LogInfo  = 0,   ///< 普通信息
    LogWarn  = 1,   ///< 提示 / 警告
    LogError = 2,   ///< 错误
};

// =====================================================================
// CAN 接口枚举
// =====================================================================

/// 一个 CAN 网络接口的描述
struct CanIfInfo {
    std::string name;     ///< 接口名，如 "can0"
    bool is_up = false;   ///< 是否已 up（IFF_UP）
};

/// 读取 /sys/class/net/<if>/type，CAN 接口的 ARP 硬件类型为 ARPHRD_CAN(280)
bool isCanInterface(const std::string& ifname) {
    const std::string path = "/sys/class/net/" + ifname + "/type";
    std::FILE* fp = std::fopen(path.c_str(), "r");
    if (!fp) return false;
    int type = 0;
    const int n = std::fscanf(fp, "%d", &type);
    std::fclose(fp);
    return n == 1 && type == 280;   // ARPHRD_CAN
}

/// 读取 /sys/class/net/<if>/flags，最低位为 IFF_UP
bool isInterfaceUp(const std::string& ifname) {
    const std::string path = "/sys/class/net/" + ifname + "/flags";
    std::FILE* fp = std::fopen(path.c_str(), "r");
    if (!fp) return false;
    unsigned flags = 0;
    const int n = std::fscanf(fp, "%x", &flags);
    std::fclose(fp);
    return n == 1 && (flags & 0x1) != 0;
}

/// 扫描系统中所有 CAN 接口（/sys/class/net/*），按名字排序返回。
/// 鲲弘 KH-UCANFDX6-Mini 加载 kcan 驱动后会注册 can0~can5。
std::vector<CanIfInfo> scanCanInterfaces() {
    std::vector<CanIfInfo> result;

    DIR* dir = ::opendir("/sys/class/net");
    if (!dir) {
        // sysfs 不可用时退化为固定列出 can0~can5
        for (int i = 0; i < kMaxBusChannel; ++i) {
            char buf[16];
            std::snprintf(buf, sizeof buf, "can%d", i);
            result.push_back({buf, false});
        }
        return result;
    }

    struct dirent* ent = nullptr;
    while ((ent = ::readdir(dir)) != nullptr) {
        const std::string name = ent->d_name;
        if (name == "." || name == "..") continue;
        if (!isCanInterface(name)) continue;
        result.push_back({name, isInterfaceUp(name)});
    }
    ::closedir(dir);

    std::sort(result.begin(), result.end(),
              [](const CanIfInfo& a, const CanIfInfo& b) { return a.name < b.name; });
    return result;
}

// =====================================================================
// 状态快照（worker 线程 -> UI 线程）
// =====================================================================

/// 一次电机状态采样结果。每一项单独带 ok 标志，因为 CAN 读取可能部分失败。
struct MotorSnapshot {
    uint32_t can_id       = 0;      ///< 采样的电机 CAN ID
    bool     any_ok       = false;  ///< 是否至少有一项读取成功（用于判断电机在线）

    bool     has_voltage  = false;  ///< 母线电压是否读到
    bool     has_current  = false;  ///< 电流是否读到
    bool     has_position = false;  ///< 位置是否读到
    bool     has_error    = false;  ///< 错误状态是否读到

    int32_t  voltage_v      = 0;    ///< 母线电压 (V)
    int32_t  current_ma     = 0;    ///< 电流 (mA)
    double   position_rad   = 0.0;  ///< 位置 (rad，输出端，相对零位)
    double   velocity_rad_s = 0.0;  ///< 速度 (rad/s，输出端)
    uint32_t error_bits     = 0;    ///< 错误状态位

    int      direction      = 0;    ///< 采样瞬间的方向（+1 逆 / -1 顺 / 0 停）
    double   rpm            = 0.0;  ///< 采样瞬间的设定转速
};

// =====================================================================
// MotorWorker：独占 CAN 总线的工作线程
// =====================================================================

/**
 * @brief CAN 通信与电机控制的工作对象（运行在独立 QThread 的事件循环里）。
 *
 * 所有 CAN 收发都发生在本对象所属线程，UI 线程只通过原子变量下发运动命令、
 * 通过 Qt 信号接收状态快照，从根本上避免多线程争抢半双工总线。
 */
class MotorWorker : public QObject {
    Q_OBJECT

public:
    MotorWorker() = default;
    ~MotorWorker() override = default;

    // —— 供 UI 线程直接调用的原子命令（不经事件队列，立即生效）——

    /// 设定转速（输出端 rpm）
    void setRpmAtomic(double rpm) { rpm_.store(rpm); }

    /// 设定当前控制的电机下标（0~3 -> ID 1~4）。越界值会被忽略。
    void setMotorIndexAtomic(int idx) {
        if (idx >= 0 && idx < static_cast<int>(kMotorIds.size())) motor_index_.store(idx);
    }

    /// 设定方向：+1 逆时针 / -1 顺时针 / 0 停止
    void setDirectionAtomic(int dir) {
        direction_.store(dir > 0 ? 1 : (dir < 0 ? -1 : 0));
    }

    /// 请求立即刷新一次状态（即使转动中、即使自动刷新已关闭）
    void requestStatusNow() { status_request_.store(true); }

    /// 设置急停闩锁：worker 在下一个 tick 会对全部电机发停止命令（抱闸）
    void latchEmergencyStop() { estop_latch_.store(true); }

    /// 设置「松手是否抱闸」
    void setBrakeOnReleaseAtomic(bool brake) { brake_on_release_.store(brake); }

    /// 设置状态自动刷新周期（ms，0 = 关闭自动刷新）
    void setStatusPeriodAtomic(int ms) { status_period_ms_.store(ms); }

    /// 设置「转动时是否也刷新状态」
    void setRefreshWhileMovingAtomic(bool on) { refresh_while_moving_.store(on); }

    /// 总线是否已打开
    bool isBusOpen() const { return can_open_.load(); }

public slots:
    /// 打开 CAN 接口并初始化四个电机（清错误）。结果通过 busOpened / logLine 上报。
    void openBus(const QString& ifname) {
        if (can_open_.load()) {
            emit busOpened(false, "总线已打开，请先断开");
            return;
        }

        // 复位全部运行时状态，避免上次会话残留
        direction_.store(0);
        estop_latch_.store(false);
        status_request_.store(false);
        last_index_ = -1;
        last_dir_   = 0;

        const std::string dev = ifname.toStdString();
        if (!can_.open(dev.c_str(), 0)) {
            emit busOpened(false,
                QString("打开 %1 失败：请确认已接上鲲弘 CANFD 模块、驱动已加载，"
                        "且接口已 up（sudo ./build/kcanctl up %2 1000000）")
                    .arg(ifname).arg(ifname));
            return;
        }
        can_open_.store(true);

        // 创建四个电机对象并清一次错误（欠压等锁存错误会阻止使能）
        motors_.clear();
        motors_.reserve(kMotorIds.size());
        for (size_t i = 0; i < kMotorIds.size(); ++i) {
            auto m = std::make_unique<JointModule>(can_, kMotorIds[i], kMotorIds[i]);
            m->setGearRatio(kMotorGearRatios[i]);
            motors_.push_back(std::move(m));
        }

        emit logLine(QString("已打开 %1，正在初始化四个电机…").arg(ifname), LogInfo);

        // 逐个清错误并探测在线状态（读位置成功即认为在线）
        std::vector<uint32_t> online;
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->clearError();
            double pos = 0.0;
            if (motors_[i]->readPosition(pos)) {
                online.push_back(kMotorIds[i]);
                emit logLine(QString("  电机 ID=%1 在线，当前位置 %2°")
                                 .arg(kMotorIds[i])
                                 .arg(radToDeg180(pos), 0, 'f', 2),
                             LogInfo);
            } else {
                emit logLine(QString("  电机 ID=%1 无应答（未连接或 ID 不符）")
                                 .arg(kMotorIds[i]),
                             LogWarn);
            }
        }

        // 初始化完成后启动心跳/刷新定时器
        if (!tick_timer_) {
            tick_timer_ = new QTimer(this);
            tick_timer_->setTimerType(Qt::PreciseTimer);
            tick_timer_->setInterval(kVelSendPeriodMs);
            connect(tick_timer_, &QTimer::timeout, this, &MotorWorker::onTick);
        }
        tick_timer_->start();

        if (online.empty()) {
            emit busOpened(true, QString("%1 已打开，但四个电机均无应答").arg(ifname));
        } else {
            QString ids;
            for (size_t i = 0; i < online.size(); ++i) {
                if (i) ids += ", ";
                ids += QString::number(online[i]);
            }
            emit busOpened(true, QString("%1 已打开，在线电机 ID：%2").arg(ifname).arg(ids));
        }
    }

    /// 关闭总线：先停全部电机（抱闸），再停定时器、释放电机对象、关闭 socket。
    void closeBus() {
        stopAllMotors(true);
        if (tick_timer_) tick_timer_->stop();
        motors_.clear();
        if (can_open_.exchange(false)) can_.close();
        last_index_ = -1;
        last_dir_   = 0;
        emit busClosed();
        emit logLine("总线已关闭，全部电机停止并抱闸", LogInfo);
    }

    /// 下发速度环 PID 增益（默认不调用；电机抖动/无力时才需要）
    void applyVelocityGains(int kp, int ki, int kd) {
        if (!can_open_.load()) {
            emit logLine("未连接总线，无法下发增益", LogError);
            return;
        }
        // 下发参数前先停机，避免增益突变引起飞车
        const int saved_dir = direction_.load();
        direction_.store(0);
        for (auto& m : motors_) m->setTargetVelocity(0.0);

        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->setVelocityKp(kp);
            motors_[i]->setVelocityKi(ki);
            motors_[i]->setVelocityKd(kd);
        }
        emit logLine(QString("已向 ID1~4 下发速度环增益 Kp=%1 Ki=%2 Kd=%3")
                         .arg(kp).arg(ki).arg(kd),
                     LogInfo);

        // 恢复原方向（一般此时为 0）
        direction_.store(saved_dir);
    }

    /// 对当前电机标零（写 Flash，掉电保留）
    void zeroCurrentMotor() {
        if (!can_open_.load()) {
            emit logLine("未连接总线，无法标零", LogError);
            return;
        }
        const int idx = motor_index_.load();
        direction_.store(0);
        for (auto& m : motors_) m->setTargetVelocity(0.0);

        const uint32_t id = kMotorIds[idx];
        const bool ok = taihu::setZeroPosition(can_, id);
        emit logLine(ok ? QString("电机 ID=%1 标零成功（当前位置已设为零位）").arg(id)
                        : QString("电机 ID=%1 标零失败").arg(id),
                     ok ? LogInfo : LogError);
        status_request_.store(true);
    }

    /// 清除四个电机的锁存错误（欠压/过流等）
    void clearAllErrors() {
        if (!can_open_.load()) {
            emit logLine("未连接总线，无法清错误", LogError);
            return;
        }
        for (auto& m : motors_) m->clearError();
        emit logLine("已向全部电机发送清除错误命令（0x0B）", LogInfo);
        status_request_.store(true);
    }

    /// 程序退出前的收尾：停机 + 停定时器 + 退出线程事件循环
    void shutdown() {
        stopAllMotors(true);
        if (tick_timer_) tick_timer_->stop();
        motors_.clear();
        if (can_open_.exchange(false)) can_.close();
        emit logLine("已退出，全部电机停止并抱闸", LogInfo);
        if (thread()) thread()->quit();
    }

signals:
    /// 总线打开结果（ok=false 时 message 为失败原因）
    void busOpened(bool ok, const QString& message);
    /// 总线已关闭
    void busClosed();
    /// 一次状态采样结果
    void snapshot(const MotorSnapshot& snap);
    /// 日志输出（level 取 LogLevel）
    void logLine(const QString& text, int level);

private slots:
    /// 周期心跳：下发速度指令 + 按节流策略读取状态
    void onTick() {
        if (!can_open_.load() || motors_.empty()) return;

        // 1) 急停闩锁优先处理
        if (estop_latch_.exchange(false)) {
            direction_.store(0);
            stopAllMotors(true);
            emit logLine("急停已执行：全部电机速度置 0 并抱闸", LogWarn);
        }

        const int    idx = motor_index_.load();
        const int    dir = direction_.load();
        const double rpm = rpm_.load();

        // 2) 让上一个电机停下来：切换了电机，或刚从转动变为松手
        const bool need_stop_last =
            (last_index_ >= 0) &&
            ((last_index_ != idx) || (last_dir_ != 0 && dir == 0));
        if (need_stop_last && last_index_ < static_cast<int>(motors_.size())) {
            motors_[last_index_]->setTargetVelocity(0.0);
            if (brake_on_release_.load()) motors_[last_index_]->stop();
        }

        // 3) 周期重发速度指令（心跳），防止驱动器因通信超时自行停机
        if (dir != 0 && idx < static_cast<int>(motors_.size())) {
            motors_[idx]->setTargetVelocity(dir * rpmToRadPerS(rpm));
        }

        last_index_ = idx;
        last_dir_   = dir;

        // 4) 状态刷新（节流）。转动期间默认跳过，保证心跳不被阻塞式读取拖慢。
        const int  period = status_period_ms_.load();
        const bool force  = status_request_.exchange(false);
        const auto now    = std::chrono::steady_clock::now();
        const long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 now - last_status_time_).count();

        const bool due = force || (period > 0 && elapsed >= period);
        if (!due) return;
        if (dir != 0 && !force && !refresh_while_moving_.load()) return;

        last_status_time_ = now;
        readAndEmitSnapshot(idx, dir, rpm);
    }

private:
    /// 对全部电机置速度 0，brake=true 时额外发停止命令（0x02，抱闸）
    void stopAllMotors(bool brake) {
        for (auto& m : motors_) {
            if (!m) continue;
            m->setTargetVelocity(0.0);
            if (brake) m->stop();
        }
    }

    /// 读取指定电机的电压/电流/位置/错误并发出快照信号
    void readAndEmitSnapshot(int idx, int dir, double rpm) {
        if (idx < 0 || idx >= static_cast<int>(motors_.size())) return;

        MotorSnapshot snap;
        snap.can_id    = kMotorIds[idx];
        snap.direction = dir;
        snap.rpm       = rpm;

        JointModule* m = motors_[idx].get();

        // 优先用 0x41 三合一读（一次拿到电流+速度+位置，减少总线占用）
        int32_t cur_ma = 0;
        double  vel    = 0.0;
        double  pos    = 0.0;
        if (m->readCurrentVelocityPosition(cur_ma, vel, pos)) {
            snap.current_ma     = cur_ma;
            snap.velocity_rad_s = vel;
            snap.position_rad   = pos;
            snap.has_current    = true;
            snap.has_position   = true;
            snap.any_ok         = true;
        }

        int32_t volt = 0;
        if (m->readBusVoltage(volt)) {
            snap.voltage_v  = volt;
            snap.has_voltage = true;
            snap.any_ok      = true;
        }

        uint32_t err = 0;
        if (m->readErrorState(err)) {
            snap.error_bits = err;
            snap.has_error  = true;
            snap.any_ok     = true;
        }

        emit snapshot(snap);
    }

    SocketCanInterface can_;                              ///< CAN 接口（本线程独占）
    std::vector<std::unique_ptr<JointModule>> motors_;    ///< 四个电机
    QTimer* tick_timer_ = nullptr;                        ///< 心跳/刷新定时器（本线程创建）

    // UI 线程写、本线程读的原子命令
    std::atomic<double> rpm_{kDefaultRpm};
    std::atomic<int>    direction_{0};
    std::atomic<int>    motor_index_{0};
    std::atomic<bool>   status_request_{false};
    std::atomic<bool>   estop_latch_{false};
    std::atomic<bool>   brake_on_release_{false};
    std::atomic<int>    status_period_ms_{kDefaultStatusPeriodMs};
    std::atomic<bool>   refresh_while_moving_{false};
    std::atomic<bool>   can_open_{false};

    // 仅本线程访问
    int  last_index_ = -1;
    int  last_dir_   = 0;
    std::chrono::steady_clock::time_point last_status_time_{};
};

} // namespace taihu_ui

Q_DECLARE_METATYPE(taihu_ui::MotorSnapshot)

namespace taihu_ui {

// =====================================================================
// 主窗口
// =====================================================================

/**
 * @brief 电机控制面板主窗口。
 *
 * 持有 QThread + MotorWorker，负责搭建界面、把用户操作翻译成原子命令、
 * 以及用多重兜底机制保证「松手即停」。
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr) : QMainWindow(parent) {
        setWindowTitle("钛虎单腿四电机控制面板 (taihu_keyboardctl_ui)");
        resize(980, 720);

        buildUi();
        buildWorker();
        buildShortcuts();

        // 全局方向键拦截：除输入框外，任意焦点位置都能用 ↑/↓ 控制电机
        qApp->installEventFilter(this);

        watchdog_ = new QTimer(this);
        watchdog_->setInterval(kWatchdogPeriodMs);
        connect(watchdog_, &QTimer::timeout, this, &MainWindow::onWatchdog);
        watchdog_->start();

        appendLog("程序已启动。请选择 CAN 接口并点击「连接」。", LogInfo);
        appendLog("提示：若下拉框为空或接口未 up，请先在外部终端执行 "
                  "sudo ./build/kcanctl up can0 1000000", LogWarn);
        updateConnectionUi(false);
    }

    ~MainWindow() override {
        shutdownWorker();
    }

protected:
    /// 关闭窗口前确保电机停止、线程安全退出
    void closeEvent(QCloseEvent* event) override {
        // 弹窗前先停机（本函数不弹窗，但保守起见先置 0）
        setDirection(0);
        shutdownWorker();
        event->accept();
    }

    /// 窗口失焦是最容易被忽略的危险场景（用户切走窗口但鼠标还按着），必须立即停机
    bool event(QEvent* e) override {
        if (e->type() == QEvent::WindowDeactivate) {
            if (direction_ != 0) stopMotion("窗口失去焦点，自动停止");
        }
        return QMainWindow::event(e);
    }

    /// 全局事件过滤：拦截 ↑/↓ 实现「任意焦点都能按住转动」，
    /// 但焦点在输入类控件时放行（否则用户没法用方向键调数值）。
    bool eventFilter(QObject* watched, QEvent* event) override {
        if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
            auto* ke = static_cast<QKeyEvent*>(event);
            if (ke->key() == Qt::Key_Up || ke->key() == Qt::Key_Down) {
                QWidget* fw = qApp->focusWidget();
                const bool editing =
                    fw && (fw->inherits("QLineEdit") ||
                           fw->inherits("QAbstractSpinBox") ||
                           fw->inherits("QTextEdit") ||
                           fw->inherits("QAbstractSlider"));
                if (!editing) {
                    handleArrowKey(ke->key(),
                                   event->type() == QEvent::KeyPress,
                                   ke->isAutoRepeat());
                    return true;   // 已消费，不再传递（避免焦点导航抢走方向键）
                }
            }
        }
        return QMainWindow::eventFilter(watched, event);
    }

private slots:
    // —— 连接区 ——
    void onRefreshInterfaces() {
        fillInterfaceCombo();
        appendLog("已重新扫描 CAN 接口", LogInfo);
    }

    void onConnectClicked() {
        if (worker_ && worker_->isBusOpen()) {
            // 已连接则执行断开
            emit requestCloseBus();
            return;
        }
        const QString ifname = combo_if_->currentText().trimmed().split(' ').first();
        if (ifname.isEmpty()) {
            appendLog("请先选择或输入 CAN 接口名（如 can0）", LogError);
            return;
        }
        setBusy(true, QString("正在连接 %1 …").arg(ifname));
        emit requestOpenBus(ifname);
    }

    // —— 转速区 ——
    void onRpmSpinChanged(double v) {
        if (slider_rpm_->value() != static_cast<int>(std::lround(v * 10.0)))
            slider_rpm_->setValue(static_cast<int>(std::lround(v * 10.0)));
        applyRpm(v);
    }

    void onRpmSliderChanged(int v) {
        const double rpm = v / 10.0;
        if (std::fabs(spin_rpm_->value() - rpm) > 1e-6) spin_rpm_->setValue(rpm);
        applyRpm(rpm);
    }

    void onRpmPresetClicked() {
        auto* btn = qobject_cast<QPushButton*>(sender());
        if (!btn) return;
        spin_rpm_->setValue(btn->property("rpm").toDouble());
    }

    // —— 电机选择 ——
    void onMotorRadioToggled(int id, bool checked) {
        if (!checked) return;
        const int idx = id - 1;
        if (idx < 0 || idx >= static_cast<int>(kMotorIds.size())) return;

        // 切换电机前先停机，避免带着转速换目标
        if (direction_ != 0) stopMotion("切换电机，先停止");
        motor_index_ = idx;
        if (worker_) worker_->setMotorIndexAtomic(idx);

        label_gear_->setText(QString("减速比 %1").arg(kMotorGearRatios[idx], 0, 'f', 0));
        appendLog(QString("已切换控制电机 -> ID=%1（减速比 %2）")
                      .arg(kMotorIds[idx]).arg(kMotorGearRatios[idx], 0, 'f', 0),
                  LogInfo);
        if (worker_ && worker_->isBusOpen()) worker_->requestStatusNow();
        updateMotionUi();
    }

    // —— 运动控制（按钮）——
    void onCcwPressed() { startMotion(+1, "按钮「▲ 逆时针」"); }
    void onCcwReleased() {
        if (direction_ == +1) stopMotion("松开「▲ 逆时针」");
        hold_source_ = HoldNone;
    }
    void onCwPressed()  { startMotion(-1, "按钮「▼ 顺时针」"); }
    void onCwReleased() {
        if (direction_ == -1) stopMotion("松开「▼ 顺时针」");
        hold_source_ = HoldNone;
    }

    void onEstopClicked() {
        setDirection(0);
        hold_source_ = HoldNone;
        if (worker_) {
            worker_->setDirectionAtomic(0);
            worker_->latchEmergencyStop();
        }
        estop_active_ = true;
        appendLog("【急停】已触发：全部电机速度置 0 并抱闸", LogError);
        label_motion_state_->setText("已急停");
        label_motion_state_->setStyleSheet("color:#b00020;font-weight:bold;font-size:14px;");
        if (worker_ && worker_->isBusOpen()) worker_->requestStatusNow();
    }

    void onBrakeToggled(bool on) {
        if (worker_) worker_->setBrakeOnReleaseAtomic(on);
        appendLog(on ? "松手后将额外抱闸（0x02）" : "松手后只置速度 0，保持使能不抱闸",
                  LogInfo);
    }

    void onClearErrorClicked() { emit requestClearErrors(); }

    void onApplyGainsClicked() {
        emit requestApplyGains(spin_kp_->value(), spin_ki_->value(), spin_kd_->value());
    }

    void onZeroClicked() {
        // 标零会写 Flash，先停机再二次确认
        if (direction_ != 0) stopMotion("标零前先停止");
        const uint32_t id = kMotorIds[motor_index_];
        const auto ret = QMessageBox::question(
            this, "确认标零",
            QString("将把电机 ID=%1 的**当前物理位置**设为零位并写入 Flash（掉电保留）。\n"
                    "确定继续吗？").arg(id),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (ret != QMessageBox::Yes) return;
        emit requestZero();
    }

    // —— 状态区 ——
    void onRefreshStatusClicked() {
        if (!worker_ || !worker_->isBusOpen()) {
            appendLog("未连接总线，无法读取状态", LogError);
            return;
        }
        worker_->requestStatusNow();
    }

    void onAutoRefreshToggled(bool on) {
        const int period = on ? spin_period_->value() : 0;
        if (worker_) worker_->setStatusPeriodAtomic(period);
        appendLog(on ? QString("自动刷新已开启，周期 %1 ms").arg(period)
                     : "自动刷新已关闭", LogInfo);
    }

    void onPeriodChanged(int ms) {
        if (check_autorefresh_->isChecked() && worker_) worker_->setStatusPeriodAtomic(ms);
    }

    void onRefreshWhileMovingToggled(bool on) {
        if (worker_) worker_->setRefreshWhileMovingAtomic(on);
        appendLog(on ? "转动时也会刷新状态（可能增大心跳抖动）"
                     : "转动时暂停状态刷新（优先保证心跳与松手即停）", LogInfo);
    }

    void onLogClearClicked() { text_log_->clear(); }

    // —— worker 信号 ——
    void onBusOpened(bool ok, const QString& message) {
        setBusy(false, QString());
        updateConnectionUi(ok);
        appendLog(message, ok ? LogInfo : LogError);
        if (!ok) {
            QMessageBox::warning(this, "连接失败", message);
        } else {
            if (worker_) {
                worker_->setMotorIndexAtomic(motor_index_);
                worker_->setBrakeOnReleaseAtomic(check_brake_->isChecked());
                worker_->setStatusPeriodAtomic(
                    check_autorefresh_->isChecked() ? spin_period_->value() : 0);
                worker_->setRefreshWhileMovingAtomic(check_refresh_moving_->isChecked());
                worker_->requestStatusNow();
            }
        }
        updateMotionUi();
    }

    void onBusClosed() {
        setDirection(0);
        hold_source_ = HoldNone;
        updateConnectionUi(false);
        setBusy(false, QString());
        updateMotionUi();
    }

    void onSnapshot(const MotorSnapshot& s) {
        estop_active_ = false;   // 收到新快照说明通信正常，解除急停显示锁定

        if (!s.any_ok) {
            label_voltage_->setText("—");
            label_current_->setText("—");
            label_position_->setText("—");
            label_velocity_->setText("—");
            label_error_->setText("无应答");
            label_error_->setStyleSheet("color:#b00020;font-weight:bold;");
            label_online_->setText(QString("ID=%1：无应答").arg(s.can_id));
            label_online_->setStyleSheet("color:#b00020;font-weight:bold;");
            return;
        }

        label_online_->setText(QString("ID=%1：在线").arg(s.can_id));
        label_online_->setStyleSheet("color:#0a7d32;font-weight:bold;");

        label_voltage_->setText(s.has_voltage ? QString("%1 V").arg(s.voltage_v)
                                              : QString("读取失败"));
        label_current_->setText(s.has_current ? QString("%1 A").arg(s.current_ma / 1000.0, 0, 'f', 3)
                                              : QString("读取失败"));
        label_position_->setText(s.has_position
                                     ? QString("%1 °").arg(radToDeg180(s.position_rad), 0, 'f', 2)
                                     : QString("读取失败"));
        label_velocity_->setText(s.has_position
                                     ? QString("%1 rad/s　(%2 rpm)")
                                           .arg(s.velocity_rad_s, 0, 'f', 3)
                                           .arg(s.velocity_rad_s * 60.0 / (2.0 * kPi), 0, 'f', 2)
                                     : QString("读取失败"));

        if (s.has_error) {
            if (s.error_bits == 0) {
                label_error_->setText("0x0（无错误）");
                label_error_->setStyleSheet("color:#0a7d32;");
            } else {
                label_error_->setText(QString("0x%1").arg(s.error_bits, 0, 16).toUpper());
                label_error_->setStyleSheet("color:#b00020;font-weight:bold;");
            }
        } else {
            label_error_->setText("读取失败");
            label_error_->setStyleSheet("color:#808080;");
        }

        label_sample_time_->setText(
            QDateTime::currentDateTime().toString("HH:mm:ss.zzz") + " 刷新");
    }

    void onWorkerLog(const QString& text, int level) {
        appendLog(text, level);
    }

private:
    // =====================================================================
    // 界面构建
    // =====================================================================

    void buildUi() {
        auto* central = new QWidget(this);
        setCentralWidget(central);
        auto* root = new QVBoxLayout(central);
        root->setContentsMargins(12, 12, 12, 12);
        root->setSpacing(10);

        root->addWidget(buildConnectGroup());
        root->addWidget(buildSpeedGroup());
        root->addWidget(buildMotorGroup());
        root->addWidget(buildMotionGroup());

        // 下半部分：左状态 / 右日志
        auto* bottom = new QHBoxLayout();
        bottom->setSpacing(10);
        bottom->addWidget(buildStatusGroup(), 0);
        bottom->addWidget(buildLogGroup(), 1);
        root->addLayout(bottom, 1);

        root->addWidget(buildFooterBar());
    }

    /// 连接区：接口选择 + 刷新 + 连接/断开 + 状态指示
    QGroupBox* buildConnectGroup() {
        auto* box = new QGroupBox("① 连接", this);
        auto* lay = new QHBoxLayout(box);

        lay->addWidget(new QLabel("CAN 接口：", box));
        combo_if_ = new QComboBox(box);
        combo_if_->setEditable(true);          // 允许手动输入（设备未枚举到时也能试）
        combo_if_->setMinimumWidth(260);
        lay->addWidget(combo_if_);

        btn_refresh_if_ = new QPushButton("刷新", box);
        connect(btn_refresh_if_, &QPushButton::clicked, this, &MainWindow::onRefreshInterfaces);
        lay->addWidget(btn_refresh_if_);

        btn_connect_ = new QPushButton("连接", box);
        btn_connect_->setMinimumWidth(110);
        connect(btn_connect_, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
        lay->addWidget(btn_connect_);

        label_conn_ = new QLabel("● 未连接", box);
        label_conn_->setStyleSheet("color:#808080;font-weight:bold;");
        lay->addWidget(label_conn_);
        lay->addStretch(1);

        fillInterfaceCombo();
        return box;
    }

    /// 转速区：数字框 + 滑块 + 快捷预设
    QGroupBox* buildSpeedGroup() {
        auto* box = new QGroupBox("② 转速设定（输出端 rpm）", this);
        auto* lay = new QVBoxLayout(box);

        auto* row1 = new QHBoxLayout();
        spin_rpm_ = new QDoubleSpinBox(box);
        spin_rpm_->setRange(kMinRpm, kMaxRpm);
        spin_rpm_->setDecimals(1);
        spin_rpm_->setSingleStep(1.0);
        spin_rpm_->setValue(kDefaultRpm);
        spin_rpm_->setSuffix(" rpm");
        spin_rpm_->setMinimumWidth(140);
        QFont f = spin_rpm_->font();
        f.setPointSize(13);
        f.setBold(true);
        spin_rpm_->setFont(f);
        connect(spin_rpm_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &MainWindow::onRpmSpinChanged);
        row1->addWidget(spin_rpm_);

        // 滑块以 0.1 rpm 为步进（整数 1~2000 映射 0.1~200.0）
        slider_rpm_ = new QSlider(Qt::Horizontal, box);
        slider_rpm_->setRange(static_cast<int>(kMinRpm * 10), static_cast<int>(kMaxRpm * 10));
        slider_rpm_->setValue(static_cast<int>(kDefaultRpm * 10));
        slider_rpm_->setPageStep(10);
        connect(slider_rpm_, &QSlider::valueChanged, this, &MainWindow::onRpmSliderChanged);
        row1->addWidget(slider_rpm_, 1);

        row1->addWidget(new QLabel(QString("%1 ~ %2").arg(kMinRpm).arg(kMaxRpm, 0, 'f', 0), box));
        lay->addLayout(row1);

        auto* row2 = new QHBoxLayout();
        row2->addWidget(new QLabel("快捷：", box));
        const double presets[] = {1.0, 5.0, 10.0, 30.0, 50.0, 100.0, 200.0};
        for (double p : presets) {
            auto* b = new QPushButton(QString::number(p, 'g', 3), box);
            b->setProperty("rpm", p);
            b->setMaximumWidth(56);
            connect(b, &QPushButton::clicked, this, &MainWindow::onRpmPresetClicked);
            row2->addWidget(b);
        }
        row2->addStretch(1);
        row2->addWidget(new QLabel("运行中可随时修改（等同命令行版 Ctrl+V）", box));
        lay->addLayout(row2);

        return box;
    }

    /// 电机选择区：ID1~4 单选 + 当前减速比
    QGroupBox* buildMotorGroup() {
        auto* box = new QGroupBox("③ 选择控制的电机", this);
        auto* lay = new QHBoxLayout(box);

        motor_group_ = new QButtonGroup(this);
        motor_group_->setExclusive(true);
        for (size_t i = 0; i < kMotorIds.size(); ++i) {
            auto* rb = new RadioButtonNoFocus(
                QString("ID %1（减速比 %2）")
                    .arg(kMotorIds[i]).arg(kMotorGearRatios[i], 0, 'f', 0), box);
            rb->setStyleSheet("font-size:14px;padding:4px 8px;");
            motor_group_->addButton(rb, static_cast<int>(i) + 1);
            lay->addWidget(rb);
        }
        motor_group_->button(1)->setChecked(true);
        connect(motor_group_, &QButtonGroup::idToggled,
                this, &MainWindow::onMotorRadioToggled);

        lay->addStretch(1);
        label_gear_ = new QLabel("减速比 101", box);
        label_gear_->setStyleSheet("color:#0050a0;font-weight:bold;font-size:14px;");
        lay->addWidget(label_gear_);
        return box;
    }

    /// 运动控制区：两个大按钮（按住转动）+ 急停 + 选项
    QGroupBox* buildMotionGroup() {
        auto* box = new QGroupBox("④ 运动控制（按住按钮或方向键转动，松手立即停止）", this);
        auto* lay = new QVBoxLayout(box);

        auto* row = new QHBoxLayout();
        row->setSpacing(12);

        btn_ccw_ = new HoldButton("▲\n逆时针转动\n（按住）", box);
        btn_cw_  = new HoldButton("▼\n顺时针转动\n（按住）", box);
        for (auto* b : {btn_ccw_, btn_cw_}) {
            b->setMinimumHeight(120);
            QFont f = b->font();
            f.setPointSize(13);
            f.setBold(true);
            b->setFont(f);
            b->setStyleSheet(
                "QPushButton{background:#eef3fa;border:2px solid #6c8ebf;border-radius:8px;}"
                "QPushButton:pressed{background:#cfe0f5;border-color:#0050a0;}");
        }
        connect(btn_ccw_, &QPushButton::pressed,  this, &MainWindow::onCcwPressed);
        connect(btn_ccw_, &QPushButton::released, this, &MainWindow::onCcwReleased);
        connect(btn_cw_,  &QPushButton::pressed,  this, &MainWindow::onCwPressed);
        connect(btn_cw_,  &QPushButton::released, this, &MainWindow::onCwReleased);

        row->addWidget(btn_ccw_, 1);
        row->addWidget(btn_cw_, 1);

        // 右侧竖排：急停 + 选项
        auto* side = new QVBoxLayout();
        side->setSpacing(8);

        btn_estop_ = new QPushButton("■ 急停\n(全部停止并抱闸)", box);
        btn_estop_->setMinimumHeight(96);
        btn_estop_->setMinimumWidth(170);
        QFont ef = btn_estop_->font();
        ef.setPointSize(12);
        ef.setBold(true);
        btn_estop_->setFont(ef);
        btn_estop_->setStyleSheet(
            "QPushButton{background:#ffe5e5;border:2px solid #d33;border-radius:8px;color:#a00;}"
            "QPushButton:pressed{background:#ffc9c9;}");
        connect(btn_estop_, &QPushButton::clicked, this, &MainWindow::onEstopClicked);
        side->addWidget(btn_estop_);

        check_brake_ = new QCheckBox("松手时抱闸（0x02）", box);
        check_brake_->setChecked(false);
        check_brake_->setToolTip("不勾选：松手只把目标速度置 0，电机保持使能（无机械冲击，默认）\n"
                                 "勾选：松手额外发停止命令抱死刹车");
        connect(check_brake_, &QCheckBox::toggled, this, &MainWindow::onBrakeToggled);
        side->addWidget(check_brake_);

        btn_clear_err_ = new QPushButton("清除电机错误", box);
        connect(btn_clear_err_, &QPushButton::clicked, this, &MainWindow::onClearErrorClicked);
        side->addWidget(btn_clear_err_);

        btn_zero_ = new QPushButton("当前位置标零…", box);
        connect(btn_zero_, &QPushButton::clicked, this, &MainWindow::onZeroClicked);
        side->addWidget(btn_zero_);

        row->addLayout(side, 0);
        lay->addLayout(row);

        // 状态提示行
        label_motion_state_ = new QLabel("停止（按住 ▲/▼ 或方向键转动）", box);
        label_motion_state_->setAlignment(Qt::AlignCenter);
        label_motion_state_->setStyleSheet("font-size:15px;font-weight:bold;color:#333;padding:4px;");
        lay->addWidget(label_motion_state_);

        // 高级：速度环增益
        auto* adv = new QHBoxLayout();
        adv->addWidget(new QLabel("速度环增益（可选，默认用出厂值）：Kp", box));
        spin_kp_ = new QSpinBox(box);
        spin_kp_->setRange(0, 100000);
        spin_kp_->setValue(1000);
        adv->addWidget(spin_kp_);
        adv->addWidget(new QLabel("Ki", box));
        spin_ki_ = new QSpinBox(box);
        spin_ki_->setRange(0, 100000);
        spin_ki_->setValue(10);
        adv->addWidget(spin_ki_);
        adv->addWidget(new QLabel("Kd", box));
        spin_kd_ = new QSpinBox(box);
        spin_kd_->setRange(0, 100000);
        spin_kd_->setValue(0);
        adv->addWidget(spin_kd_);
        btn_gains_ = new QPushButton("下发增益", box);
        connect(btn_gains_, &QPushButton::clicked, this, &MainWindow::onApplyGainsClicked);
        adv->addWidget(btn_gains_);
        adv->addStretch(1);
        lay->addLayout(adv);

        return box;
    }

    /// 状态区：电压/电流/位置/速度/错误 + 自动刷新设置
    QGroupBox* buildStatusGroup() {
        auto* box = new QGroupBox("⑤ 当前电机状态", this);
        box->setMinimumWidth(330);
        auto* lay = new QVBoxLayout(box);

        label_online_ = new QLabel("ID=1：未连接", box);
        label_online_->setStyleSheet("color:#808080;font-weight:bold;font-size:14px;");
        lay->addWidget(label_online_);

        auto* form = new QFormLayout();
        form->setSpacing(8);
        const QFont vf = [] { QFont f; f.setPointSize(13); f.setBold(true); return f; }();

        label_voltage_  = new QLabel("—", box);
        label_current_  = new QLabel("—", box);
        label_position_ = new QLabel("—", box);
        label_velocity_ = new QLabel("—", box);
        label_error_    = new QLabel("—", box);
        for (auto* l : {label_voltage_, label_current_, label_position_, label_velocity_})
            l->setFont(vf);
        label_error_->setFont(vf);

        form->addRow("电压：",              label_voltage_);
        form->addRow("电流：",              label_current_);
        form->addRow("位置（相对零位）：",  label_position_);
        form->addRow("速度：",              label_velocity_);
        form->addRow("错误状态：",          label_error_);
        lay->addLayout(form);

        label_sample_time_ = new QLabel("尚未采样", box);
        label_sample_time_->setStyleSheet("color:#808080;");
        lay->addWidget(label_sample_time_);

        auto* opt = new QHBoxLayout();
        check_autorefresh_ = new QCheckBox("自动刷新，周期", box);
        check_autorefresh_->setChecked(true);
        connect(check_autorefresh_, &QCheckBox::toggled, this, &MainWindow::onAutoRefreshToggled);
        opt->addWidget(check_autorefresh_);

        spin_period_ = new QSpinBox(box);
        spin_period_->setRange(50, 5000);
        spin_period_->setSingleStep(50);
        spin_period_->setValue(kDefaultStatusPeriodMs);
        spin_period_->setSuffix(" ms");
        connect(spin_period_, QOverload<int>::of(&QSpinBox::valueChanged),
                this, &MainWindow::onPeriodChanged);
        opt->addWidget(spin_period_);
        opt->addStretch(1);
        lay->addLayout(opt);

        check_refresh_moving_ = new QCheckBox("转动时也刷新状态（会占用总线，可能影响心跳）", box);
        check_refresh_moving_->setChecked(false);
        connect(check_refresh_moving_, &QCheckBox::toggled,
                this, &MainWindow::onRefreshWhileMovingToggled);
        lay->addWidget(check_refresh_moving_);

        auto* row = new QHBoxLayout();
        btn_refresh_status_ = new QPushButton("立即刷新（等同 Ctrl+P）", box);
        connect(btn_refresh_status_, &QPushButton::clicked, this, &MainWindow::onRefreshStatusClicked);
        row->addWidget(btn_refresh_status_);
        row->addStretch(1);
        lay->addLayout(row);

        lay->addStretch(1);
        return box;
    }

    /// 日志区
    QGroupBox* buildLogGroup() {
        auto* box = new QGroupBox("⑥ 操作日志", this);
        auto* lay = new QVBoxLayout(box);

        text_log_ = new QTextEdit(box);
        text_log_->setReadOnly(true);
        QFont f("Monospace");
        f.setStyleHint(QFont::TypeWriter);
        f.setPointSize(10);
        text_log_->setFont(f);
        // 限制历史长度，防止长时间运行内存膨胀
        text_log_->document()->setMaximumBlockCount(2000);
        lay->addWidget(text_log_);

        auto* row = new QHBoxLayout();
        row->addStretch(1);
        btn_clear_log_ = new QPushButton("清空日志", box);
        connect(btn_clear_log_, &QPushButton::clicked, this, &MainWindow::onLogClearClicked);
        row->addWidget(btn_clear_log_);
        lay->addLayout(row);
        return box;
    }

    /// 底部状态栏
    QWidget* buildFooterBar() {
        auto* bar = new QWidget(this);
        auto* lay = new QHBoxLayout(bar);
        lay->setContentsMargins(0, 0, 0, 0);

        label_hint_ = new QLabel(
            "快捷键：↑ 逆时针 / ↓ 顺时针（按住转动，松手即停）　"
            "Ctrl+1~4 切电机　Ctrl+P 刷新状态　Ctrl+V 聚焦转速框　Esc 急停", bar);
        label_hint_->setStyleSheet("color:#555;");
        lay->addWidget(label_hint_);
        lay->addStretch(1);

        label_busy_ = new QLabel("", bar);
        label_busy_->setStyleSheet("color:#0050a0;font-weight:bold;");
        lay->addWidget(label_busy_);
        return bar;
    }

    /// 填充 CAN 接口下拉框（带 up/down 提示）
    void fillInterfaceCombo() {
        const QString prev = combo_if_->currentText();
        combo_if_->clear();

        const auto list = scanCanInterfaces();
        for (const auto& info : list) {
            const QString name = QString::fromStdString(info.name);
            const QString text = info.is_up
                ? QString("%1  （已就绪）").arg(name)
                : QString("%1  （未 up：sudo ./build/kcanctl up %2 1000000）").arg(name, name);
            combo_if_->addItem(text, name);
        }

        if (combo_if_->count() == 0) {
            // 一个 CAN 接口都没有：给出可手动输入的默认项
            combo_if_->addItem(QString("%1  （未检测到 CAN 接口，可手动输入）")
                                   .arg(kDefaultCanDevice),
                               QString(kDefaultCanDevice));
        }

        // 尽量恢复之前的选择
        const int found = combo_if_->findText(prev);
        combo_if_->setCurrentIndex(found >= 0 ? found : 0);
    }

    // =====================================================================
    // 线程与 worker
    // =====================================================================

    void buildWorker() {
        qRegisterMetaType<MotorSnapshot>("MotorSnapshot");

        worker_ = new MotorWorker();          // 不能有 parent（要 moveToThread）
        worker_thread_ = new QThread(this);
        worker_->moveToThread(worker_thread_);
        connect(worker_thread_, &QThread::finished, worker_, &QObject::deleteLater);

        // worker -> UI（跨线程自动 QueuedConnection）
        connect(worker_, &MotorWorker::busOpened, this, &MainWindow::onBusOpened);
        connect(worker_, &MotorWorker::busClosed, this, &MainWindow::onBusClosed);
        connect(worker_, &MotorWorker::snapshot,  this, &MainWindow::onSnapshot);
        connect(worker_, &MotorWorker::logLine,   this, &MainWindow::onWorkerLog);

        // UI -> worker（跨线程自动 QueuedConnection，在 worker 线程执行 CAN 操作）
        connect(this, &MainWindow::requestOpenBus,       worker_, &MotorWorker::openBus);
        connect(this, &MainWindow::requestCloseBus,      worker_, &MotorWorker::closeBus);
        connect(this, &MainWindow::requestClearErrors,   worker_, &MotorWorker::clearAllErrors);
        connect(this, &MainWindow::requestApplyGains,    worker_, &MotorWorker::applyVelocityGains);
        connect(this, &MainWindow::requestZero,          worker_, &MotorWorker::zeroCurrentMotor);
        connect(this, &MainWindow::requestShutdown,      worker_, &MotorWorker::shutdown);

        worker_thread_->start();
    }

    /// 安全关闭 worker：先原子置 0 停机，再排队 shutdown，最后等待线程退出
    void shutdownWorker() {
        if (!worker_thread_) return;

        // 立即（原子）停止运动，不等事件队列
        direction_ = 0;
        hold_source_ = HoldNone;
        if (worker_) {
            worker_->setDirectionAtomic(0);
            worker_->latchEmergencyStop();
            worker_->setStatusPeriodAtomic(0);
        }
        if (watchdog_) watchdog_->stop();

        // 排队执行完整收尾（停电机 + 关 socket + 退出事件循环）
        emit requestShutdown();

        if (!worker_thread_->wait(3000)) {
            // 极端情况（worker 阻塞在 CAN 读取）下兜底强制结束
            worker_thread_->terminate();
            worker_thread_->wait(1000);
        }
        worker_ = nullptr;
        worker_thread_ = nullptr;   // QThread 以 this 为 parent，由 Qt 释放
    }

signals:
    void requestOpenBus(const QString& ifname);
    void requestCloseBus();
    void requestClearErrors();
    void requestApplyGains(int kp, int ki, int kd);
    void requestZero();
    void requestShutdown();

private:
    // =====================================================================
    // 运动控制的统一入口（所有兜底机制都收敛到这两个函数）
    // =====================================================================

    /// 按住来源，用于看门狗判断「是否还按着」
    enum HoldSource { HoldNone = 0, HoldMouse, HoldKeyboard };

    /// 开始转动。dir = +1 逆时针 / -1 顺时针。
    void startMotion(int dir, const char* reason) {
        if (!worker_ || !worker_->isBusOpen()) {
            appendLog("尚未连接 CAN 总线，无法转动", LogError);
            return;
        }
        if (estop_active_) {
            appendLog("处于急停状态，请先再次点击「急停」解除或重新连接", LogWarn);
            return;
        }
        if (dir == 0) { stopMotion(reason); return; }

        direction_ = dir;
        worker_->setMotorIndexAtomic(motor_index_);
        worker_->setRpmAtomic(currentRpm());
        worker_->setDirectionAtomic(dir);

        appendLog(QString("%1 → ID=%2 %3，%4 rpm")
                      .arg(reason)
                      .arg(kMotorIds[motor_index_])
                      .arg(dir > 0 ? "逆时针转动" : "顺时针转动")
                      .arg(currentRpm(), 0, 'f', 1),
                  LogInfo);
        updateMotionUi();
    }

    /// 停止转动（松手 / 切换 / 失焦 / 看门狗 等所有场景共用）
    void stopMotion(const char* reason) {
        if (direction_ == 0) { updateMotionUi(); return; }
        direction_ = 0;
        if (worker_) worker_->setDirectionAtomic(0);
        appendLog(QString("%1 → 停止").arg(reason), LogInfo);
        updateMotionUi();
        if (worker_ && worker_->isBusOpen()) worker_->requestStatusNow();
    }

    /// 直接设定方向（用于急停等不需要日志文案的场景）
    void setDirection(int dir) {
        direction_ = dir;
        if (worker_) worker_->setDirectionAtomic(dir);
    }

    double currentRpm() const { return spin_rpm_->value(); }

    void applyRpm(double rpm) {
        if (worker_) worker_->setRpmAtomic(rpm);
        // 正在转动时立即用新转速重发一次（worker 心跳也会跟上）
        label_motion_state_->setStyleSheet(motionStateStyle());
        if (direction_ != 0) {
            appendLog(QString("转速已更新为 %1 rpm（转动中即时生效）").arg(rpm, 0, 'f', 1), LogInfo);
        }
    }

    /// 方向键处理：按下设方向，松开清零（忽略自动重复）
    void handleArrowKey(int key, bool is_press, bool is_repeat) {
        const int dir = (key == Qt::Key_Up) ? +1 : -1;

        if (is_press) {
            if (is_repeat) {
                // 自动重复：仅确认「仍然按住」，不重复打日志
                if (direction_ == dir) hold_source_ = HoldKeyboard;
                return;
            }
            hold_source_ = HoldKeyboard;
            keyboard_hold_ = true;
            startMotion(dir, key == Qt::Key_Up ? "键盘 ↑ 按下" : "键盘 ↓ 按下");
        } else {
            if (is_repeat) return;   // 自动重复产生的假 release 必须忽略
            keyboard_hold_ = false;
            if (direction_ == dir && hold_source_ == HoldKeyboard)
                stopMotion(key == Qt::Key_Up ? "键盘 ↑ 松开" : "键盘 ↓ 松开");
            if (hold_source_ == HoldKeyboard) hold_source_ = HoldNone;
        }
    }

    /// 看门狗：兜住「released / keyRelease 事件丢失」的危险情况
    void onWatchdog() {
        if (direction_ == 0) return;
        if (!worker_ || !worker_->isBusOpen()) { stopMotion("总线已断开"); return; }

        if (hold_source_ == HoldMouse) {
            // 鼠标路径：Qt 能实时查询按键状态，松开却漏掉 released 时在此兜底
            if (qApp->mouseButtons() == Qt::NoButton) {
                stopMotion("看门狗：鼠标已松开");
                hold_source_ = HoldNone;
            }
        } else if (hold_source_ == HoldKeyboard) {
            if (!keyboard_hold_) {
                stopMotion("看门狗：方向键已松开");
                hold_source_ = HoldNone;
            }
        } else {
            // 方向非 0 却没有按住来源，属于异常状态，立即停止
            stopMotion("看门狗：无按住来源，异常保护停机");
        }
    }

    // =====================================================================
    // 界面状态更新
    // =====================================================================

    void updateConnectionUi(bool connected) {
        connected_ = connected;
        if (connected) {
            label_conn_->setText("● 已连接");
            label_conn_->setStyleSheet("color:#0a7d32;font-weight:bold;");
            btn_connect_->setText("断开");
            combo_if_->setEnabled(false);
            btn_refresh_if_->setEnabled(false);
        } else {
            label_conn_->setText("● 未连接");
            label_conn_->setStyleSheet("color:#808080;font-weight:bold;");
            btn_connect_->setText("连接");
            combo_if_->setEnabled(true);
            btn_refresh_if_->setEnabled(true);
        }
        updateMotionUi();
    }

    /// 未连接时禁用所有运动/参数控件，防止误操作
    void updateMotionUi() {
        const bool on = connected_;
        btn_ccw_->setEnabled(on);
        btn_cw_->setEnabled(on);
        btn_estop_->setEnabled(on);
        btn_clear_err_->setEnabled(on);
        btn_zero_->setEnabled(on);
        btn_gains_->setEnabled(on);
        btn_refresh_status_->setEnabled(on);
        check_brake_->setEnabled(on);
        check_autorefresh_->setEnabled(on);
        check_refresh_moving_->setEnabled(on);

        if (!on) {
            label_motion_state_->setText("未连接（请先选择 CAN 接口并连接）");
            label_motion_state_->setStyleSheet("font-size:15px;font-weight:bold;color:#808080;padding:4px;");
            return;
        }

        if (estop_active_) {
            label_motion_state_->setText("已急停");
            label_motion_state_->setStyleSheet(
                "color:#b00020;font-weight:bold;font-size:15px;padding:4px;");
            return;
        }

        if (direction_ > 0) {
            label_motion_state_->setText(
                QString("▲ ID=%1 逆时针转动中　%2 rpm").arg(kMotorIds[motor_index_])
                    .arg(currentRpm(), 0, 'f', 1));
        } else if (direction_ < 0) {
            label_motion_state_->setText(
                QString("▼ ID=%1 顺时针转动中　%2 rpm").arg(kMotorIds[motor_index_])
                    .arg(currentRpm(), 0, 'f', 1));
        } else {
            label_motion_state_->setText(
                QString("停止（当前控制 ID=%1，按住 ▲/▼ 或方向键转动）")
                    .arg(kMotorIds[motor_index_]));
        }
        label_motion_state_->setStyleSheet(motionStateStyle());
    }

    const char* motionStateStyle() const {
        return direction_ != 0
            ? "color:#0050a0;font-weight:bold;font-size:15px;padding:4px;"
            : "color:#333;font-weight:bold;font-size:15px;padding:4px;";
    }

    /// 忙碌状态：禁用交互，显示提示文字
    void setBusy(bool busy, const QString& text) {
        busy_ = busy;
        label_busy_->setText(text);
        btn_connect_->setEnabled(!busy);
        combo_if_->setEnabled(!busy && !connected_);
        btn_refresh_if_->setEnabled(!busy && !connected_);
        if (busy) {
            btn_ccw_->setEnabled(false);
            btn_cw_->setEnabled(false);
        } else {
            updateMotionUi();
        }
    }

    /// 追加一条带时间戳的日志（自动按级别着色）
    void appendLog(const QString& text, int level) {
        const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
        const char* tag = (level == LogError) ? "错误" : (level == LogWarn) ? "提示" : "信息";
        const char* color = (level == LogError) ? "#b00020"
                          : (level == LogWarn)  ? "#b26a00" : "#222222";
        // 用 toHtmlEscaped 防止日志文本里的特殊字符破坏富文本
        text_log_->append(QString("<span style='color:#808080'>%1</span> "
                                  "<span style='color:%2'>[%3] %4</span>")
                              .arg(ts, color, QString(tag), text.toHtmlEscaped()));
    }

    // =====================================================================
    // 快捷键
    // =====================================================================

    void buildShortcuts() {
        // Ctrl+1~4 切换电机（等同命令行版）
        for (size_t i = 0; i < kMotorIds.size(); ++i) {
            auto* sc = new QShortcut(QKeySequence(QString("Ctrl+%1").arg(i + 1)), this);
            sc->setContext(Qt::WindowShortcut);
            const int idx = static_cast<int>(i);
            connect(sc, &QShortcut::activated, this, [this, idx]() {
                auto* btn = motor_group_->button(idx + 1);
                if (btn) btn->setChecked(true);
            });
        }

        // Ctrl+P 立即刷新状态
        auto* sc_p = new QShortcut(QKeySequence("Ctrl+P"), this);
        sc_p->setContext(Qt::WindowShortcut);
        connect(sc_p, &QShortcut::activated, this, &MainWindow::onRefreshStatusClicked);

        // Ctrl+V 把焦点移到转速输入框（等同命令行版重设转速）
        auto* sc_v = new QShortcut(QKeySequence("Ctrl+V"), this);
        sc_v->setContext(Qt::WindowShortcut);
        connect(sc_v, &QShortcut::activated, this, [this]() {
            spin_rpm_->setFocus();
            spin_rpm_->selectAll();
        });

        // Esc / Ctrl+C 急停
        for (const char* seq : {"Esc", "Ctrl+C"}) {
            auto* sc = new QShortcut(QKeySequence(QString(seq)), this);
            sc->setContext(Qt::WindowShortcut);
            connect(sc, &QShortcut::activated, this, &MainWindow::onEstopClicked);
        }
    }

    /// 无焦点按钮：防止方向键被按钮的焦点导航吃掉，也避免空格误触发
    class RadioButtonNoFocus : public QRadioButton {
    public:
        explicit RadioButtonNoFocus(const QString& text, QWidget* parent = nullptr)
            : QRadioButton(text, parent) { setFocusPolicy(Qt::NoFocus); }
    };

    /// 按住式按钮：不可聚焦（方向键交给主窗口统一处理）
    class HoldButton : public QPushButton {
    public:
        explicit HoldButton(const QString& text, QWidget* parent = nullptr)
            : QPushButton(text, parent) { setFocusPolicy(Qt::NoFocus); }
    };

    // —— 连接区 ——
    QComboBox*   combo_if_       = nullptr;
    QPushButton* btn_refresh_if_ = nullptr;
    QPushButton* btn_connect_    = nullptr;
    QLabel*      label_conn_     = nullptr;

    // —— 转速区 ——
    QDoubleSpinBox* spin_rpm_  = nullptr;
    QSlider*        slider_rpm_= nullptr;

    // —— 电机选择 ——
    QButtonGroup* motor_group_ = nullptr;
    QLabel*       label_gear_  = nullptr;

    // —— 运动控制 ——
    HoldButton*  btn_ccw_   = nullptr;
    HoldButton*  btn_cw_    = nullptr;
    QPushButton* btn_estop_ = nullptr;
    QCheckBox*   check_brake_     = nullptr;
    QPushButton* btn_clear_err_   = nullptr;
    QPushButton* btn_zero_        = nullptr;
    QSpinBox*    spin_kp_ = nullptr;
    QSpinBox*    spin_ki_ = nullptr;
    QSpinBox*    spin_kd_ = nullptr;
    QPushButton* btn_gains_ = nullptr;
    QLabel*      label_motion_state_ = nullptr;

    // —— 状态区 ——
    QLabel*    label_online_    = nullptr;
    QLabel*    label_voltage_   = nullptr;
    QLabel*    label_current_   = nullptr;
    QLabel*    label_position_  = nullptr;
    QLabel*    label_velocity_  = nullptr;
    QLabel*    label_error_     = nullptr;
    QLabel*    label_sample_time_ = nullptr;
    QCheckBox* check_autorefresh_     = nullptr;
    QSpinBox*  spin_period_           = nullptr;
    QCheckBox* check_refresh_moving_  = nullptr;
    QPushButton* btn_refresh_status_  = nullptr;

    // —— 日志 / 底部 ——
    QTextEdit*   text_log_    = nullptr;
    QPushButton* btn_clear_log_ = nullptr;
    QLabel*      label_hint_  = nullptr;
    QLabel*      label_busy_  = nullptr;

    // —— 线程 ——
    QThread*     worker_thread_ = nullptr;
    MotorWorker* worker_        = nullptr;
    QTimer*      watchdog_      = nullptr;

    // —— 运行时状态（仅 UI 线程访问）——
    int  motor_index_  = 0;       ///< 当前控制电机下标
    int  direction_    = 0;       ///< +1 / -1 / 0
    bool connected_    = false;
    bool busy_         = false;
    bool estop_active_ = false;
    bool keyboard_hold_= false;   ///< 方向键是否处于按住状态
    HoldSource hold_source_ = HoldNone;
};

} // namespace taihu_ui

// =====================================================================
// 程序入口
// =====================================================================

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("taihu_keyboardctl_ui");
    QApplication::setOrganizationName("TaiHu");

    // 状态快照跨线程传递，必须先注册 metatype
    qRegisterMetaType<taihu_ui::MotorSnapshot>("taihu_ui::MotorSnapshot");
    qRegisterMetaType<taihu_ui::MotorSnapshot>("MotorSnapshot");

    taihu_ui::MainWindow win;
    win.show();
    return app.exec();
}

// Q_OBJECT 类定义在 .cpp 中，必须包含 AUTOMOC 生成的 moc 文件
#include "taihu_keyboardctl_ui.moc"
