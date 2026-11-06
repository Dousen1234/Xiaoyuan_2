/**
 * @file x3588_ui.cpp
 * @brief 腿臂上下位机系统 · 3588 图形控制界面（Qt5 Widgets）
 *
 * ============================================================================
 * 这个程序是干什么的
 * ============================================================================
 * 把原本要在好几个终端里手敲的命令，集成到一个窗口里：
 *
 *   ① CAN 接口配置   can0~can5 + can_mcu 的 up/down 与波特率
 *   ② 轨迹文件选择   手动输入或浏览选择 .xlsx 路径
 *   ③ 进程启停       下位机 lower_control / 上位机 upper_control
 *   ④ 回零工具       调 leg_goto_zero，指定腿臂与时长
 *   ⑤ 电机状态反馈   16 个关节的实际角 / 目标角 / 误差 / 在线性
 *   ⑥ 急停 + 抱闸    estop / resume / arm / disarm
 *   ⑦ CAN 总线状态   UP/DOWN、波特率、ERROR-ACTIVE、错误计数
 *
 * ============================================================================
 * 线程模型（关键设计）
 * ============================================================================
 *   - **UI 线程**只做界面，绝不阻塞等 CAN；所有耗时操作都是异步的。
 *   - **CanMonitor 线程**独占一个 CAN_RAW socket（默认 can_mcu），
 *     周期性发 0x41 请求、被动收 0x41 反馈 / 0x161 心跳 / 0x162 报警。
 *     全局数据用 std::atomic 存放，UI 用 QTimer 定时取快照，**不加锁**。
 *   - **QProcess** 管理三个子进程（lower_control / upper_control /
 *     leg_goto_zero），stdout+stderr 合并后实时追加到对应日志页。
 *     往子进程 stdin 写命令来实现 arm / disarm / estop。
 *
 * ============================================================================
 * can_mcu 的占用约定（重要）
 * ============================================================================
 *   upper_control 运行时也要打开 can_mcu。两个进程同时开同一个 SocketCAN
 *   接口是**合法的**（内核会把每帧复制给所有 socket），但两端都会发 0x41
 *   请求，导致下位机回帧量翻倍。
 *
 *   因此本程序做【自动退让】：
 *     - 检测到 upper_control 在运行  → CAN 监控进入【被动模式】：只收不发
 *     - upper_control 未运行         → CAN 监控进入【主动模式】：200ms 轮询
 *
 *   leg_goto_zero 自己会独占 can_mcu 并发大量帧，所以**启动它之前本程序会
 *   主动关闭 CAN 监控**，等它退出后自动重新打开。
 *
 * ============================================================================
 * 权限模型
 * ============================================================================
 *   `ip link set ... up` 和修改波特率都需要 root。本程序**不自带提权**，
 *   而是要求你用 sudo 启动：
 *
 *       sudo ./run_ui_sudo.sh        ← 推荐，脚本会处理好 X 授权
 *
 *   一旦以 root 启动，界面里所有操作都不再需要密码。
 *   注意：root 下 Qt 需要能读到原用户的 Xauthority，脚本已自动设置
 *   XAUTHORITY / DISPLAY / HOME，详见 run_ui_sudo.sh。
 *
 *   （CAN_RAW socket 本身不需要 root，所以监控功能在非 root 下也能用，
 *     只是改不了接口状态。）
 *
 * ============================================================================
 * 编译
 * ============================================================================
 *   cd 3588_control && mkdir -p build && cd build && cmake .. && make -j4
 *   产物：build/x3588_ui
 *
 * ============================================================================
 * 命令行参数
 * ============================================================================
 *   --can <接口>        上位机侧 CAN 接口（默认 can_mcu）
 *   --hz <值>           上位机发送频率（透传给 upper_control，默认 10）
 *   --loop <次>         轨迹循环次数（透传给 upper_control，默认 1）
 *   --bin-dir <目录>    三个可执行文件所在目录（默认与本程序同目录）
 *   --no-monitor        启动时不自动打开 CAN 监控
 *   -h, --help          显示帮助
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h> // geteuid()

#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QFontDatabase>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollBar>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextCursor>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <taihu/socket_can.h>

#include "proto.h"

// =============================================================================
// 与上下位机协议保持一致的常量（全部来自 proto.h，此处只做别名，避免手抄出错）
// =============================================================================
using namespace x3588;

namespace
{

    // =========================================================================
    // 颜色 / 字号（集中定义，方便统一改风格）
    // =========================================================================
    const char *kCssGreen = "#1a7f37";
    const char *kCssRed = "#c62828";
    const char *kCssOrange = "#c77700";
    const char *kCssGray = "#666666";
    const char *kCssBlue = "#1565c0";

    int64_t nowMs()
    {
        using namespace std::chrono;
        return duration_cast<milliseconds>(system_clock::now().time_since_epoch())
            .count();
    }

    /// 当前进程是否以 root 运行（决定界面上哪些按钮可用）
    bool isRoot()
    {
        return ::geteuid() == 0;
    }

    // =========================================================================
    // 关节命名（与 lower_control 的 jointName 一致）
    // =========================================================================
    QString jointName(int global_idx)
    {
        const int leg = global_idx / kJointCount;
        const int j = global_idx % kJointCount;
        return QString("%1%2").arg(QLatin1Char(legLetter(leg))).arg(j + 1);
    }

    QString legName(int leg_idx)
    {
        return QString(QLatin1Char(legLetter(leg_idx)));
    }

    // =========================================================================
    // std::string / QString / 数值 互转的小工具
    // =========================================================================
    QString fmtDeg(double rad)
    {
        return QString::number(rad * kRadToDeg, 'f', 2);
    }

    QString fmtSignedDeg(double rad)
    {
        const double d = rad * kRadToDeg;
        return QString("%1%2").arg(d >= 0 ? "+" : "").arg(d, 0, 'f', 2);
    }

} // namespace

// =============================================================================
// CAN 监控线程
//
// 用一个独立线程独占一个 CAN_RAW socket：
//   - 主动模式：每 poll_ms 向 ID 1~16 各发一帧 0x41 请求
//   - 被动模式：只收不发（upper_control 自己在轮询数据已经在总线上了）
//   - 始终解析下位机回帧：0x41 反馈、0x161 心跳、0x162 软限位报警
//
// 所有对外数据都是 std::atomic，UI 线程直接读，不需要加锁。
// =============================================================================
class CanMonitor
{
public:
    enum class Mode
    {
        Idle,    ///< 未打开
        Passive, ///< 已打开，只收不发
        Active,  ///< 已打开，主动轮询
    };

    CanMonitor() = default;
    ~CanMonitor() { stop(); }

    CanMonitor(const CanMonitor &) = delete;
    CanMonitor &operator=(const CanMonitor &) = delete;

    /// 打开指定接口并启动线程；返回失败原因（成功返回空字符串）
    QString start(const std::string &ifname, int poll_ms = 200)
    {
        stop();
        ifname_ = ifname;
        poll_ms_ = poll_ms > 0 ? poll_ms : 200;

        if (!can_.open(ifname_.c_str(), 0))
        {
            return QString("打开 %1 失败（接口可能未 up，或没有权限）")
                .arg(QString::fromStdString(ifname_));
        }

        // 清掉可能残留的旧帧，避免界面上出现过期数据
        can_.drain();

        // 复位所有反馈状态
        for (int i = 0; i < kTotalJoints; ++i)
        {
            fb_online_[i].store(false, std::memory_order_relaxed);
            fb_ms_[i].store(0, std::memory_order_relaxed);
            fb_proto_[i].store(0, std::memory_order_relaxed);
            fb_frames_[i].store(0, std::memory_order_relaxed);
        }
        hb_bits_.store(0, std::memory_order_relaxed);
        hb_ms_.store(0, std::memory_order_relaxed);
        alarm_active_.store(false, std::memory_order_relaxed);
        alarm_id_.store(0, std::memory_order_relaxed);
        alarm_side_.store(0, std::memory_order_relaxed);
        alarm_proto_.store(0, std::memory_order_relaxed);
        tx_frames_.store(0, std::memory_order_relaxed);
        rx_frames_.store(0, std::memory_order_relaxed);
        last_error_.clear();

        run_.store(true, std::memory_order_release);
        th_ = std::thread(&CanMonitor::runLoop, this);
        opened_.store(true, std::memory_order_release);
        return QString();
    }

    void stop()
    {
        if (!opened_.load(std::memory_order_acquire))
            return;
        run_.store(false, std::memory_order_release);
        if (th_.joinable())
            th_.join();
        can_.close();
        opened_.store(false, std::memory_order_release);
    }

    bool isOpen() const { return opened_.load(std::memory_order_acquire); }

    /// 切换主动 / 被动模式（运行中可随时切）
    void setActivePolling(bool on)
    {
        active_poll_.store(on, std::memory_order_relaxed);
    }

    bool activePolling() const
    {
        return active_poll_.load(std::memory_order_relaxed);
    }

    Mode mode() const
    {
        if (!opened_.load(std::memory_order_acquire))
            return Mode::Idle;
        return active_poll_.load(std::memory_order_relaxed) ? Mode::Active
                                                            : Mode::Passive;
    }

    std::string ifname() const { return ifname_; }

    /// 紧急停止：向全部 16 个 ID 发一帧 0x02（退出流模式，电机保持）
    void requestStopAll()
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        pending_cmds_.push_back(Cmd{Cmd::StopAll, 0, {}});
    }

    /// 向全部 16 个 ID 发一帧 0x03（清除软限位报警锁定）
    void requestClearLimit()
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        pending_cmds_.push_back(Cmd{Cmd::ClearLimit, 0, {}});
    }

    // ---- 快照读取（UI 线程调用，全部是 atomic，无锁） ----

    bool jointOnline(int idx) const
    {
        return fb_online_[idx].load(std::memory_order_relaxed) &&
               nowMs() - fb_ms_[idx].load(std::memory_order_relaxed) < 1500;
    }

    double jointRad(int idx) const
    {
        return protoToRad(fb_proto_[idx].load(std::memory_order_relaxed));
    }

    int64_t jointFrames(int idx) const
    {
        return fb_frames_[idx].load(std::memory_order_relaxed);
    }

    bool heartbeatFresh() const
    {
        const int64_t t = hb_ms_.load(std::memory_order_relaxed);
        return t != 0 && nowMs() - t < 1500;
    }

    uint8_t heartbeatBits() const
    {
        return hb_bits_.load(std::memory_order_relaxed);
    }

    bool limitLocked() const
    {
        return alarm_active_.load(std::memory_order_relaxed);
    }

    int alarmJointId() const { return alarm_id_.load(std::memory_order_relaxed); }
    int alarmSide() const { return alarm_side_.load(std::memory_order_relaxed); }
    double alarmRad() const
    {
        return protoToRad(alarm_proto_.load(std::memory_order_relaxed));
    }

    int64_t txFrames() const { return tx_frames_.load(std::memory_order_relaxed); }
    int64_t rxFrames() const { return rx_frames_.load(std::memory_order_relaxed); }

    QString lastError() const
    {
        std::lock_guard<std::mutex> lk(err_mutex_);
        return last_error_;
    }

private:
    struct Cmd
    {
        enum Kind
        {
            StopAll,
            ClearLimit
        } kind;
        uint16_t id;
        uint8_t data[8];
    };

    void setError(const QString &e)
    {
        std::lock_guard<std::mutex> lk(err_mutex_);
        last_error_ = e;
    }

    /// 构造并发送一帧
    bool sendRaw(uint16_t id, const uint8_t *data, uint8_t dlc)
    {
        taihu::CanFrame f;
        f.id = id;
        f.dlc = dlc;
        f.is_extended = false;
        std::memcpy(f.data, data, dlc);
        if (can_.send(f))
        {
            tx_frames_.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
        return false;
    }

    /// 向 ID 1~16 各发一帧单字节功能码
    void broadcastCommand(uint8_t code)
    {
        const uint8_t d[1] = {code};
        for (int i = 0; i < kTotalJoints; ++i)
            sendRaw(jointToId(i / kJointCount, i % kJointCount), d, 1);
    }

    /// 处理一帧收到的数据
    void handleFrame(const taihu::CanFrame &f)
    {
        rx_frames_.fetch_add(1, std::memory_order_relaxed);
        if (f.is_extended)
            return;

        const uint16_t id = static_cast<uint16_t>(f.id & 0x7FF);

        // ---- 0x161 心跳 ----
        if (id == kHeartbeatId && f.dlc >= 1)
        {
            hb_bits_.store(f.data[0], std::memory_order_relaxed);
            hb_ms_.store(nowMs(), std::memory_order_relaxed);
            // 心跳的 bit2 是软限位报警；若已清除，本地警报状态跟着复位
            if ((f.data[0] & kHbBitLimitLock) == 0)
                alarm_active_.store(false, std::memory_order_relaxed);
            return;
        }

        // ---- 0x162 软限位报警（DLC=8） ----
        if (id == kLimitAlarmId && f.dlc >= 8 && f.data[0] == kLimitAlarmCode)
        {
            alarm_id_.store(f.data[1], std::memory_order_relaxed);
            alarm_side_.store(f.data[2], std::memory_order_relaxed);
            alarm_proto_.store(getI32LE(&f.data[4]), std::memory_order_relaxed);
            alarm_active_.store(true, std::memory_order_relaxed);
            return;
        }

        // ---- 0x41 关节反馈（DLC=5） ----
        if (isJointId(id) && f.dlc >= 5 && f.data[0] == kCmdGetState)
        {
            const int idx = idToLeg(id) * kJointCount + idToJoint(id);
            fb_proto_[idx].store(getI32LE(&f.data[1]), std::memory_order_relaxed);
            fb_ms_[idx].store(nowMs(), std::memory_order_relaxed);
            fb_online_[idx].store(true, std::memory_order_relaxed);
            fb_frames_[idx].fetch_add(1, std::memory_order_relaxed);
            return;
        }
    }

    void drainPendingCommands()
    {
        std::deque<Cmd> local;
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            if (pending_cmds_.empty())
                return;
            local.swap(pending_cmds_);
        }
        for (const Cmd &c : local)
        {
            switch (c.kind)
            {
            case Cmd::StopAll:
                broadcastCommand(kCmdStop);
                break;
            case Cmd::ClearLimit:
                broadcastCommand(kCmdClearLimit);
                break;
            }
        }
    }

    void runLoop()
    {
        int64_t last_poll = 0;
        while (run_.load(std::memory_order_acquire))
        {
            // 1) 先执行外部塞进来的命令（急停优先级最高）
            drainPendingCommands();

            // 2) 主动模式下周期性轮询
            if (active_poll_.load(std::memory_order_relaxed))
            {
                const int64_t t = nowMs();
                if (t - last_poll >= poll_ms_)
                {
                    last_poll = t;
                    broadcastCommand(kCmdGetState);
                }
            }

            // 3) 收帧（阻塞最多 50ms，保证 run_ 变化能及时响应）
            taihu::CanFrame f;
            if (can_.receive(f, 50))
            {
                handleFrame(f);
                // 连续把缓冲区里的帧都取出来，避免积压
                while (can_.receive(f, 0))
                    handleFrame(f);
            }
        }
    }

    taihu::SocketCanInterface can_;
    std::thread th_;
    std::atomic<bool> run_{false};
    std::atomic<bool> opened_{false};
    std::atomic<bool> active_poll_{true};

    std::string ifname_{"can_mcu"};
    int poll_ms_{200};

    std::atomic<bool> fb_online_[kTotalJoints];
    std::atomic<int64_t> fb_ms_[kTotalJoints];
    std::atomic<int32_t> fb_proto_[kTotalJoints];
    std::atomic<int64_t> fb_frames_[kTotalJoints];

    std::atomic<uint8_t> hb_bits_{0};
    std::atomic<int64_t> hb_ms_{0};

    std::atomic<bool> alarm_active_{false};
    std::atomic<int> alarm_id_{0};
    std::atomic<int> alarm_side_{0};
    std::atomic<int32_t> alarm_proto_{0};

    std::atomic<int64_t> tx_frames_{0};
    std::atomic<int64_t> rx_frames_{0};

    std::mutex cmd_mutex_;
    std::deque<Cmd> pending_cmds_;

    mutable std::mutex err_mutex_;
    QString last_error_;
};

// =============================================================================
// 主窗口
// =============================================================================
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

    /// 由 main() 在 show() 之前调用，覆盖可执行文件搜索目录
    void setBinDir(const QString &d)
    {
        if (d.isEmpty())
            return;
        // 轨迹路径默认留空，不再随 bin 目录迁移
        m_binDir = d;
    }

protected:
    void closeEvent(QCloseEvent *e) override;

private:
    // ---- 界面搭建（按功能分组，便于对照需求） ----
    void buildUi();
    QWidget *buildTopBar();        ///< 标题 + 权限提示 + 急停
    QWidget *buildCanConfigBox();  ///< ① CAN 接口配置
    QWidget *buildTrajectoryBox(); ///< ② 轨迹文件
    QWidget *buildProcessBox();    ///< ③ 进程启停 + 抱闸
    QWidget *buildGotoZeroBox();   ///< ④ 回零工具
    QWidget *buildMotorTableBox(); ///< ⑤ 电机状态反馈
    QWidget *buildBusStatusBox();  ///< ⑥ CAN 总线状态
    QWidget *buildLogBox();        ///< ⑦ 日志

    // ---- 业务动作 ----
    void onApplyCanConfig();   ///< 配置勾选的 CAN 通道
    void onDownAllCan();       ///< 关闭全部通道
    void onRefreshBusStatus(); ///< 刷新总线状态面板
    void onBrowseXlsx();       ///< 选择轨迹文件
    void onStartLower();       ///< 启动下位机
    void onStopLower();        ///< 停止下位机
    void onStartUpper();       ///< 启动上位机
    void onStopUpper();        ///< 停止上位机
    void onStartGotoZero();    ///< 开始回零
    void onStopGotoZero();     ///< 中止回零
    void onEstop();            ///< 急停
    void onSendLowerCmd(const QString &cmd, const QString &label);
    void onMonitorToggle(bool on); ///< 打开/关闭 CAN 监控
    void onClearLimitAlarm();      ///< 清除软限位报警

    // ---- 周期刷新 ----
    void tickProcesses();  ///< 500ms：进程状态 + 监控模式仲裁
    void tickMotorTable(); ///< 150ms：关节表 + 报警摘要
    void tickBusStatus();  ///< 1500ms：总线状态

    // ---- 工具 ----
    void appendLog(QPlainTextEdit *dst, const QString &text);
    void logLower(const QString &s) { appendLog(m_logLower, s); }
    void logUpper(const QString &s) { appendLog(m_logUpper, s); }
    void logGoto(const QString &s) { appendLog(m_logGoto, s); }
    void logSys(const QString &s);
    bool runSync(const QString &prog, const QStringList &args, QString *out);
    QString findKcanctl() const;
    QString binPath(const QString &name) const;
    void setLabelState(QLabel *l, const QString &text, const char *color);
    void updateProcessButtons();

    // ---- CAN 接口配置行 ----
    struct IfRow
    {
        QString name;
        QCheckBox *en = nullptr;
        QSpinBox *rate = nullptr;
        QLabel *state = nullptr;
    };
    std::vector<IfRow> m_ifRows;

    // ---- 监控 / 进程 ----
    CanMonitor *m_mon = nullptr;
    QProcess *m_lower = nullptr;
    QProcess *m_upper = nullptr;
    QProcess *m_goto = nullptr;

    QString m_binDir;
    QString m_canIf{"can_mcu"};
    QString m_lowerHostIf{"can5"};
    bool m_autoMonitor = true;
    bool m_monitorPausedForGoto = false;

    // ---- 控件 ----
    QLabel *m_permLabel = nullptr;
    QLabel *m_monStateLabel = nullptr;
    QLabel *m_heartbeatLabel = nullptr;
    QLabel *m_alarmLabel = nullptr;
    QLabel *m_frameLabel = nullptr;

    QLineEdit *m_xlsxEdit = nullptr;
    QSpinBox *m_hzSpin = nullptr;
    QSpinBox *m_loopSpin = nullptr;

    QLabel *m_lowerState = nullptr;
    QLabel *m_upperState = nullptr;
    QLabel *m_gotoState = nullptr;

    QPushButton *m_btnLowerStart = nullptr;
    QPushButton *m_btnLowerStop = nullptr;
    QPushButton *m_btnLowerArm = nullptr;
    QPushButton *m_btnLowerDisarm = nullptr;
    QPushButton *m_btnUpperStart = nullptr;
    QPushButton *m_btnUpperStop = nullptr;

    QComboBox *m_legCombo = nullptr;
    QDoubleSpinBox *m_gotoTimeSpin = nullptr;
    QPushButton *m_btnGotoStart = nullptr;
    QPushButton *m_btnGotoStop = nullptr;

    QTableWidget *m_motorTable = nullptr;
    QTableWidget *m_busTable = nullptr;

    QPlainTextEdit *m_logLower = nullptr;
    QPlainTextEdit *m_logUpper = nullptr;
    QPlainTextEdit *m_logGoto = nullptr;
    QPlainTextEdit *m_logSys = nullptr;

    QPushButton *m_btnMonitor = nullptr;
    QPushButton *m_btnEstop = nullptr;

    // 关节限位（关节端，度）——与 lower_control 默认值严格一致
    static const double kLimDeg[kJointCount][2];
};

const double MainWindow::kLimDeg[kJointCount][2] = {
    {-120.0, 120.0}, // J1 Y  直驱
    {-120.0, 120.0}, // J2 P1 锥齿轮 1:2
    {-30.0, 180.0},  // J3 P2 锥齿轮 1:2（不对称）
    {-90.0, 90.0},   // J4 Y  直驱
};

// =============================================================================
// 构造 / 析构
// =============================================================================
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    m_binDir = QCoreApplication::applicationDirPath();
    m_mon = new CanMonitor();

    // 子进程对象
    m_lower = new QProcess(this);
    m_upper = new QProcess(this);
    m_goto = new QProcess(this);

    for (QProcess *p : {m_lower, m_upper, m_goto})
    {
        // 合并 stderr 到 stdout，保证日志顺序正确
        p->setProcessChannelMode(QProcess::MergedChannels);
    }

    buildUi();

    // ---- 子进程输出 → 日志 ----
    connect(m_lower, &QProcess::readyReadStandardOutput, this, [this]
            { logLower(QString::fromLocal8Bit(m_lower->readAllStandardOutput())); });
    connect(m_upper, &QProcess::readyReadStandardOutput, this, [this]
            { logUpper(QString::fromLocal8Bit(m_upper->readAllStandardOutput())); });
    connect(m_goto, &QProcess::readyReadStandardOutput, this, [this]
            { logGoto(QString::fromLocal8Bit(m_goto->readAllStandardOutput())); });

    connect(m_lower, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int code, QProcess::ExitStatus st)
            {
                logLower(QString("\n=== [下位机] 进程结束，退出码 %1%2 ===\n")
                             .arg(code)
                             .arg(st == QProcess::CrashExit ? "（异常崩溃）" : ""));
                updateProcessButtons(); });
    connect(m_upper, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int code, QProcess::ExitStatus st)
            {
                logUpper(QString("\n=== [上位机] 进程结束，退出码 %1%2 ===\n")
                             .arg(code)
                             .arg(st == QProcess::CrashExit ? "（异常崩溃）" : ""));
                updateProcessButtons(); });
    connect(m_goto, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int code, QProcess::ExitStatus st)
            {
                logGoto(QString("\n=== [回零] 进程结束，退出码 %1%2 ===\n")
                            .arg(code)
                            .arg(st == QProcess::CrashExit ? "（异常崩溃）" : ""));
                updateProcessButtons(); });

    // ---- 周期定时器 ----
    auto *t1 = new QTimer(this);
    connect(t1, &QTimer::timeout, this, &MainWindow::tickProcesses);
    t1->start(500);

    auto *t2 = new QTimer(this);
    connect(t2, &QTimer::timeout, this, &MainWindow::tickMotorTable);
    t2->start(150);

    auto *t3 = new QTimer(this);
    connect(t3, &QTimer::timeout, this, &MainWindow::tickBusStatus);
    t3->start(1500);

    updateProcessButtons();
    onRefreshBusStatus();

    // 启动时自动打开监控（若接口可用）
    if (m_autoMonitor)
        onMonitorToggle(true);
}

MainWindow::~MainWindow()
{
    if (m_mon)
    {
        m_mon->stop();
        delete m_mon;
        m_mon = nullptr;
    }
}

void MainWindow::closeEvent(QCloseEvent *e)
{
    const bool anyRunning =
        (m_lower->state() != QProcess::NotRunning) ||
        (m_upper->state() != QProcess::NotRunning) ||
        (m_goto->state() != QProcess::NotRunning);

    if (anyRunning)
    {
        const auto r = QMessageBox::question(
            this, "确认退出",
            "还有程序在运行（下位机 / 上位机 / 回零）。\n\n"
            "退出会先向所有关节发 0x02（退出流模式，电机保持），\n"
            "然后结束这些进程。是否继续？",
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (r != QMessageBox::Yes)
        {
            e->ignore();
            return;
        }
    }

    // 尽量安全地收尾：先发 0x02，再终止进程
    if (m_mon && m_mon->isOpen())
    {
        m_mon->requestStopAll();
        QThread::msleep(120);
        m_mon->stop();
    }

    for (QProcess *p : {m_goto, m_upper, m_lower})
    {
        if (p->state() != QProcess::NotRunning)
        {
            p->terminate();
            if (!p->waitForFinished(2500))
                p->kill();
        }
    }

    e->accept();
}

// =============================================================================
// 界面搭建
// =============================================================================

void MainWindow::buildUi()
{
    setWindowTitle("3588 腿臂控制系统 · 上下位机控制台");
    // 3588 屏幕是 1920×1080，窗口尽量用满，字号才能放得开
    resize(1700, 1000);
    setMaximumHeight(1040);

    // 统一加大一点控件字号，避免在 1920 屏上看着吃力
    setStyleSheet(
        "QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;"
        "font-weight:bold;font-size:14px;color:#1b3a5c;}"
        "QPushButton{min-height:28px;padding:2px 12px;font-size:13px;}"
        "QHeaderView::section{padding:5px 4px;font-weight:bold;"
        "background:#eef2f6;border:0px;border-right:1px solid #d0d6dd;"
        "border-bottom:1px solid #d0d6dd;}"
        "QCheckBox::indicator{width:16px;height:16px;}");

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    root->addWidget(buildTopBar());

    // 上部：三列 —— CAN 配置 / 上下位机程序 / 轨迹 + 回零
    // 这样做的好处：每列都只占自己的高度，不会被别的框挤扁。
    auto *topSplit = new QSplitter(Qt::Horizontal);

    auto *colCan = new QWidget();
    auto *layCan = new QVBoxLayout(colCan);
    layCan->setContentsMargins(0, 0, 0, 0);
    layCan->setSpacing(8);
    layCan->addWidget(buildCanConfigBox());
    layCan->addStretch(1);

    auto *colProc = new QWidget();
    auto *layProc = new QVBoxLayout(colProc);
    layProc->setContentsMargins(0, 0, 0, 0);
    layProc->setSpacing(8);
    layProc->addWidget(buildProcessBox());
    layProc->addStretch(1);

    auto *colTraj = new QWidget();
    auto *layTraj = new QVBoxLayout(colTraj);
    layTraj->setContentsMargins(0, 0, 0, 0);
    layTraj->setSpacing(8);
    layTraj->addWidget(buildTrajectoryBox());
    layTraj->addWidget(buildGotoZeroBox());
    layTraj->addStretch(1);

    topSplit->addWidget(colCan);
    topSplit->addWidget(colProc);
    topSplit->addWidget(colTraj);
    topSplit->setStretchFactor(0, 1);
    topSplit->setStretchFactor(1, 1);
    topSplit->setStretchFactor(2, 1);
    // CAN 列元素最多（7 行 × 4 列），给它多一点，其余两列各让一点
    topSplit->setSizes({580, 550, 550});

    // 下部：垂直分割（电机表 + 总线状态） / 日志
    auto *botSplit = new QSplitter(Qt::Horizontal);
    botSplit->addWidget(buildMotorTableBox());
    botSplit->addWidget(buildBusStatusBox());

    auto *mainSplit = new QSplitter(Qt::Vertical);
    mainSplit->addWidget(topSplit);
    mainSplit->addWidget(botSplit);
    mainSplit->addWidget(buildLogBox());
    mainSplit->setStretchFactor(0, 0);
    mainSplit->setStretchFactor(1, 2);
    mainSplit->setStretchFactor(2, 2);
    // 初始高度分配：上部功能区 380，中部表格 330，下部日志 190
    mainSplit->setSizes({380, 330, 190});

    root->addWidget(mainSplit, 1);
    setCentralWidget(central);

    // 关键：给主分割器一个很小的最小高度，避免内部控件把窗口顶到 1080 之外。
    // 3588 屏幕只有 1920×1080，超出部分会被窗口管理器裁掉。
    mainSplit->setMinimumHeight(240);
    central->setMinimumHeight(0);
    central->setMinimumWidth(0);

    // ---- 状态栏 ----
    m_frameLabel = new QLabel("—");
    statusBar()->addPermanentWidget(m_frameLabel);
    statusBar()->showMessage("就绪");
}

// -----------------------------------------------------------------------------
// 顶部：标题 / 权限 / 监控状态 / 急停
// -----------------------------------------------------------------------------
QWidget *MainWindow::buildTopBar()
{
    auto *box = new QGroupBox("系统状态");
    auto *lay = new QHBoxLayout(box);
    lay->setContentsMargins(10, 6, 10, 6);
    lay->setSpacing(14);

    m_permLabel = new QLabel();
    setLabelState(m_permLabel,
                  isRoot() ? "权限：root ✓" : "权限：普通用户（CAN 配置不可用）",
                  isRoot() ? kCssGreen : kCssRed);

    m_monStateLabel = new QLabel("CAN 监控：未打开");
    setLabelState(m_monStateLabel, "CAN 监控：未打开", kCssGray);

    m_btnMonitor = new QPushButton("打开 CAN 监控");
    m_btnMonitor->setCheckable(false);
    connect(m_btnMonitor, &QPushButton::clicked, this, [this]
            {
                if (m_mon && m_mon->isOpen())
                    onMonitorToggle(false);
                else
                    onMonitorToggle(true); });

    m_heartbeatLabel = new QLabel("下位机心跳：—");
    setLabelState(m_heartbeatLabel, "下位机心跳：—", kCssGray);

    m_alarmLabel = new QLabel("软限位：正常");
    setLabelState(m_alarmLabel, "软限位：正常", kCssGreen);

    m_btnEstop = new QPushButton("急  停");
    m_btnEstop->setMinimumSize(120, 40);
    m_btnEstop->setStyleSheet(
        "QPushButton{background:#c62828;color:white;font-weight:bold;"
        "font-size:16px;border-radius:4px;}"
        "QPushButton:hover{background:#e53935;}"
        "QPushButton:pressed{background:#8e0000;}");
    m_btnEstop->setToolTip(
        "立即执行：\n"
        "  · 向下位机发送 estop（全部抱闸并锁定）\n"
        "  · 向 can_mcu 上全部 16 个 ID 发 0x02（双重保障）\n"
        "  · 终止 upto_control 与回零进程");
    connect(m_btnEstop, &QPushButton::clicked, this, &MainWindow::onEstop);

    lay->addWidget(m_permLabel);
    lay->addWidget(m_monStateLabel);
    lay->addWidget(m_btnMonitor);
    lay->addWidget(m_heartbeatLabel);
    lay->addWidget(m_alarmLabel);
    lay->addStretch(1);
    lay->addWidget(m_btnEstop);
    return box;
}

// -----------------------------------------------------------------------------
// ① CAN 接口配置
// -----------------------------------------------------------------------------
QWidget *MainWindow::buildCanConfigBox()
{
    auto *box = new QGroupBox("① CAN 接口配置（需要 root）");
    auto *lay = new QVBoxLayout(box);
    lay->setContentsMargins(10, 8, 10, 8);
    lay->setSpacing(6);

    auto *hint = new QLabel(
        "勾选要拉起的通道，改好波特率后点【应用】。"
        "配置动作 = ip link set &lt;通道&gt; down → type can bitrate N → up。");
    hint->setWordWrap(true);
    hint->setStyleSheet("color:#555;font-size:12px;");
    lay->addWidget(hint);

    auto *grid = new QGridLayout();
    grid->setHorizontalSpacing(12);
    grid->setVerticalSpacing(8);

    // 表头
    const char *heads[] = {"启用", "通道", "波特率 (bit/s)", "当前状态"};
    for (int c = 0; c < 4; ++c)
    {
        auto *h = new QLabel(heads[c]);
        QFont f = h->font();
        f.setBold(true);
        f.setPointSize(11);
        h->setFont(f);
        grid->addWidget(h, 0, c);
    }

    // 默认被勾选的通道集合
    const QStringList all = {"can0", "can1", "can2", "can3", "can4", "can5",
                             "can_mcu"};

    // 默认波特率：腿臂 can0~can3 用 1M；下位机通道 can5 与 can_mcu 用 990000。
    // 注意 can4 未使用，给它 1M 即可；can5 必须与 can_mcu 保持一致。
    auto defaultRate = [](const QString &n) -> int
    {
        if (n == "can5" || n == "can_mcu")
            return 990000;
        return 1000000;
    };

    // 默认勾选：can0~can3、can5（can4 与 can_mcu 默认不勾，避免误操作）
    auto defaultChecked = [](const QString &n) -> bool
    {
        return n == "can0" || n == "can1" || n == "can2" || n == "can3" ||
               n == "can5";
    };

    int row = 1;
    for (const QString &n : all)
    {
        IfRow r;
        r.name = n;
        r.en = new QCheckBox();
        r.en->setChecked(defaultChecked(n));
        r.rate = new QSpinBox();
        r.rate->setRange(10000, 2000000);
        r.rate->setSingleStep(10000);
        r.rate->setValue(defaultRate(n));
        r.rate->setGroupSeparatorShown(true);
        // 波特率是常用的手改项，字号给足、宽度给足，否则尾数看不清
        r.rate->setMinimumWidth(160);
        r.rate->setMinimumHeight(30);
        QFont rf = r.rate->font();
        rf.setPointSize(12);
        rf.setBold(true);
        r.rate->setFont(rf);
        r.state = new QLabel("—");
        setLabelState(r.state, "—", kCssGray);
        {
            QFont sf = r.state->font();
            sf.setPointSize(11);
            r.state->setFont(sf);
        }
        // 【状态】列留够宽度，防止【UP 990000】这种文本被挤断行
        r.state->setMinimumWidth(104);

        grid->addWidget(r.en, row, 0, Qt::AlignCenter);
        auto *nameLab = new QLabel(n);
        QFont nf = nameLab->font();
        nf.setFamily("monospace");
        nf.setPointSize(12);
        nf.setBold(true);
        nameLab->setFont(nf);
        nameLab->setMinimumWidth(72);
        grid->addWidget(nameLab, row, 1);
        grid->addWidget(r.rate, row, 2);
        grid->addWidget(r.state, row, 3);

        m_ifRows.push_back(r);
        ++row;
    }

    grid->setColumnStretch(3, 1);
    lay->addLayout(grid);

    auto *btnRow = new QHBoxLayout();
    auto *btnApply = new QPushButton("应用配置（up + 设置波特率）");
    btnApply->setMinimumHeight(34);
    connect(btnApply, &QPushButton::clicked, this, &MainWindow::onApplyCanConfig);

    auto *btnDown = new QPushButton("关闭全部");
    btnDown->setMinimumHeight(34);
    connect(btnDown, &QPushButton::clicked, this, &MainWindow::onDownAllCan);

    auto *btnRef = new QPushButton("刷新状态");
    btnRef->setMinimumHeight(34);
    connect(btnRef, &QPushButton::clicked, this, &MainWindow::onRefreshBusStatus);

    btnRow->addWidget(btnApply);
    btnRow->addWidget(btnDown);
    btnRow->addWidget(btnRef);
    btnRow->addStretch(1);
    lay->addLayout(btnRow);

    return box;
}

// -----------------------------------------------------------------------------
// ② 轨迹文件
// -----------------------------------------------------------------------------
QWidget *MainWindow::buildTrajectoryBox()
{
    auto *box = new QGroupBox("② 上位机轨迹文件");
    auto *lay = new QVBoxLayout(box);
    lay->setContentsMargins(10, 8, 10, 8);
    lay->setSpacing(6);

    auto *row1 = new QHBoxLayout();
    row1->addWidget(new QLabel("xlsx 路径："));
    m_xlsxEdit = new QLineEdit();
    // 默认留空，避免误发上一次的旧轨迹；点【浏览…】或手动输入后再启动上位机
    m_xlsxEdit->setPlaceholderText(
        "未选择轨迹文件，点【浏览…】或手打完整路径，例如 "
        "/home/rk/walk_systemctl/workspaces/Xiaoyuan_2/step_position/step_test_ui.xlsx");
    m_xlsxEdit->clear();
    m_xlsxEdit->setMinimumHeight(30);
    auto *btnBrowse = new QPushButton("浏览…");
    btnBrowse->setMinimumSize(88, 30);
    btnBrowse->setMaximumWidth(110);
    connect(btnBrowse, &QPushButton::clicked, this, &MainWindow::onBrowseXlsx);
    row1->addWidget(m_xlsxEdit, 1);
    row1->addWidget(btnBrowse);
    lay->addLayout(row1);

    auto *row2 = new QHBoxLayout();
    row2->addWidget(new QLabel("发送频率："));
    m_hzSpin = new QSpinBox();
    m_hzSpin->setRange(1, 100);
    m_hzSpin->setValue(10);
    m_hzSpin->setSuffix(" Hz");
    m_hzSpin->setMinimumSize(110, 30);
    m_hzSpin->setMaximumWidth(140);
    row2->addWidget(m_hzSpin);

    row2->addSpacing(16);
    row2->addWidget(new QLabel("循环次数："));
    m_loopSpin = new QSpinBox();
    m_loopSpin->setRange(0, 9999);
    m_loopSpin->setValue(1);
    m_loopSpin->setSpecialValueText("0（无限）");
    m_loopSpin->setMinimumSize(110, 30);
    m_loopSpin->setMaximumWidth(140);
    row2->addWidget(m_loopSpin);

    row2->addSpacing(16);
    auto *hint = new QLabel(
        "（轨迹路径默认留空；频率与循环次数会作为参数传给 upper_control）");
    hint->setStyleSheet("color:#666;font-size:12px;");
    row2->addWidget(hint);
    row2->addStretch(1);
    lay->addLayout(row2);

    return box;
}

// -----------------------------------------------------------------------------
// ③ 进程启停 + 抱闸
// -----------------------------------------------------------------------------
QWidget *MainWindow::buildProcessBox()
{
    auto *box = new QGroupBox("③ 上 / 下位机程序");
    auto *lay = new QVBoxLayout(box);
    lay->setContentsMargins(10, 8, 10, 8);
    lay->setSpacing(10);

    auto *advice = new QLabel(
        "<b>启动顺序：</b>先配 CAN → 启<b>下位机</b>（它会 arm 解抱闸）"
        " → 再启<b>上位机</b>或<b>回零</b>。");
    advice->setWordWrap(true);
    advice->setStyleSheet("color:#555;font-size:12px;");
    lay->addWidget(advice);

    // 运行状态标签统一用大号加粗字体，避免看不清
    auto makeStateLabel = []()
    {
        auto *l = new QLabel("● 未运行");
        QFont f = l->font();
        f.setPointSize(13);
        f.setBold(true);
        l->setFont(f);
        l->setStyleSheet(QString("color:%1;").arg(kCssGray));
        return l;
    };

    // ---- 下位机 ----
    auto *lowerBox = new QGroupBox("下位机 lower_control");
    auto *ll = new QVBoxLayout(lowerBox);
    ll->setContentsMargins(10, 8, 10, 10);
    ll->setSpacing(8);

    auto *lr1 = new QHBoxLayout();
    m_lowerState = makeStateLabel();
    lr1->addWidget(m_lowerState);
    lr1->addStretch(1);
    m_btnLowerStart = new QPushButton("启动");
    m_btnLowerStart->setMinimumSize(110, 34);
    m_btnLowerStop = new QPushButton("停止");
    m_btnLowerStop->setMinimumSize(110, 34);
    lr1->addWidget(m_btnLowerStart);
    lr1->addWidget(m_btnLowerStop);
    ll->addLayout(lr1);

    auto *capLabel = new QLabel("抱闸 / 急停 / 状态：");
    capLabel->setStyleSheet("color:#333;font-size:12px;");
    ll->addWidget(capLabel);

    // 2×2 网格：四个按钮的标都挺长，挤成一行会看不清
    auto *lr2 = new QGridLayout();
    lr2->setHorizontalSpacing(8);
    lr2->setVerticalSpacing(8);
    m_btnLowerArm = new QPushButton("arm 解抱闸");
    m_btnLowerDisarm = new QPushButton("disarm 抱闸");
    auto *btnResume = new QPushButton("resume 解除急停");
    auto *btnStat = new QPushButton("stat 打印状态");
    for (QPushButton *b : {m_btnLowerArm, m_btnLowerDisarm, btnResume, btnStat})
        b->setMinimumHeight(34);
    lr2->addWidget(m_btnLowerArm, 0, 0);
    lr2->addWidget(m_btnLowerDisarm, 0, 1);
    lr2->addWidget(btnResume, 1, 0);
    lr2->addWidget(btnStat, 1, 1);
    lr2->setColumnStretch(0, 1);
    lr2->setColumnStretch(1, 1);
    ll->addLayout(lr2);

    connect(m_btnLowerStart, &QPushButton::clicked, this, &MainWindow::onStartLower);
    connect(m_btnLowerStop, &QPushButton::clicked, this, &MainWindow::onStopLower);
    connect(m_btnLowerArm, &QPushButton::clicked, this, [this]
            { onSendLowerCmd("arm", "arm 解抱闸"); });
    connect(m_btnLowerDisarm, &QPushButton::clicked, this, [this]
            { onSendLowerCmd("disarm", "disarm 抱闸"); });
    connect(btnResume, &QPushButton::clicked, this, [this]
            { onSendLowerCmd("resume", "resume 解除急停"); });
    connect(btnStat, &QPushButton::clicked, this, [this]
            { onSendLowerCmd("stat", "stat 打印状态"); });

    lay->addWidget(lowerBox);

    // ---- 上位机 ----
    auto *upperBox = new QGroupBox("上位机 upper_control");
    auto *ul = new QVBoxLayout(upperBox);
    ul->setContentsMargins(10, 8, 10, 10);
    ul->setSpacing(8);

    auto *ur1 = new QHBoxLayout();
    m_upperState = makeStateLabel();
    ur1->addWidget(m_upperState);
    ur1->addStretch(1);
    m_btnUpperStart = new QPushButton("启动");
    m_btnUpperStart->setMinimumSize(110, 34);
    m_btnUpperStop = new QPushButton("停止");
    m_btnUpperStop->setMinimumSize(110, 34);
    ur1->addWidget(m_btnUpperStart);
    ur1->addWidget(m_btnUpperStop);
    ul->addLayout(ur1);

    auto *uhint = new QLabel(
        "启动时自动打开 <code>can_mcu</code>；若下位机未运行，轨迹会被丢弃。");
    uhint->setWordWrap(true);
    uhint->setStyleSheet("color:#666;font-size:12px;");
    ul->addWidget(uhint);

    connect(m_btnUpperStart, &QPushButton::clicked, this, &MainWindow::onStartUpper);
    connect(m_btnUpperStop, &QPushButton::clicked, this, &MainWindow::onStopUpper);

    lay->addWidget(upperBox);

    return box;
}

// -----------------------------------------------------------------------------
// ④ 回零工具
// -----------------------------------------------------------------------------
QWidget *MainWindow::buildGotoZeroBox()
{
    auto *box = new QGroupBox("④ 回 0 位（leg_goto_zero）");
    auto *lay = new QVBoxLayout(box);
    lay->setContentsMargins(10, 8, 10, 8);
    lay->setSpacing(6);

    auto *hint = new QLabel(
        "五次多项式平滑插值，速率曲线首尾平滑、全程单调，"
        "不会冲出软限位。开始时本程序会<b>自动关闭 CAN 监控</b>，"
        "结束后自动恢复。");
    hint->setWordWrap(true);
    hint->setStyleSheet("color:#555;font-size:12px;");
    lay->addWidget(hint);

    auto *row1 = new QHBoxLayout();
    row1->addWidget(new QLabel("目标腿臂："));
    m_legCombo = new QComboBox();
    for (int i = 0; i < kLegCount; ++i)
        m_legCombo->addItem(legName(i) + QString(" 腿"));
    m_legCombo->setMinimumSize(110, 30);
    m_legCombo->setMaximumWidth(130);
    row1->addWidget(m_legCombo);

    row1->addSpacing(16);
    row1->addWidget(new QLabel("运动时长："));
    m_gotoTimeSpin = new QDoubleSpinBox();
    m_gotoTimeSpin->setRange(0.5, 600.0);
    m_gotoTimeSpin->setDecimals(1);
    m_gotoTimeSpin->setSingleStep(0.5);
    m_gotoTimeSpin->setValue(8.0);
    m_gotoTimeSpin->setSuffix(" 秒");
    m_gotoTimeSpin->setMinimumSize(130, 30);
    m_gotoTimeSpin->setMaximumWidth(160);
    row1->addWidget(m_gotoTimeSpin);

    row1->addStretch(1);
    lay->addLayout(row1);

    auto *row2 = new QHBoxLayout();
    m_gotoState = new QLabel("● 未运行");
    setLabelState(m_gotoState, "● 未运行", kCssGray);
    {
        QFont gf = m_gotoState->font();
        gf.setPointSize(13);
        m_gotoState->setFont(gf);
    }
    row2->addWidget(m_gotoState);
    row2->addStretch(1);

    m_btnGotoStart = new QPushButton("开始回零");
    m_btnGotoStart->setMinimumSize(110, 34);
    m_btnGotoStop = new QPushButton("中止");
    m_btnGotoStop->setMinimumSize(90, 34);
    row2->addWidget(m_btnGotoStart);
    row2->addWidget(m_btnGotoStop);
    lay->addLayout(row2);

    connect(m_btnGotoStart, &QPushButton::clicked, this, &MainWindow::onStartGotoZero);
    connect(m_btnGotoStop, &QPushButton::clicked, this, &MainWindow::onStopGotoZero);

    auto *note = new QLabel(
        "提示：C / D 腿需要 can2 / can3 处于 UP 状态，"
        "否则回零会因「下位机链路未就绪」而失败。");
    note->setWordWrap(true);
    note->setStyleSheet("color:#c77700;font-size:12px;font-weight:bold;");
    lay->addWidget(note);

    return box;
}

// -----------------------------------------------------------------------------
// ⑤ 电机状态反馈表
// -----------------------------------------------------------------------------
QWidget *MainWindow::buildMotorTableBox()
{
    auto *box = new QGroupBox("⑤ 电机 / 关节状态反馈");
    auto *lay = new QVBoxLayout(box);
    lay->setContentsMargins(8, 8, 8, 8);

    m_motorTable = new QTableWidget(kTotalJoints, 7);
    m_motorTable->setHorizontalHeaderLabels(
        {"关节", "在线", "实际角 (°)", "目标角 (°)", "误差 (°)", "帧数", "限位"});
    m_motorTable->verticalHeader()->setVisible(false);
    m_motorTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_motorTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_motorTable->setAlternatingRowColors(true);
    m_motorTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_motorTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    // 行高加大，字号也加大，16 行才能一眼看清
    m_motorTable->verticalHeader()->setDefaultSectionSize(27);
    m_motorTable->setMinimumHeight(180);

    QFont mono("monospace");
    mono.setStyleHint(QFont::Monospace);
    mono.setPointSize(11);
    m_motorTable->setFont(mono);

    for (int i = 0; i < kTotalJoints; ++i)
    {
        const int j = i % kJointCount;
        for (int c = 0; c < 7; ++c)
        {
            auto *it = new QTableWidgetItem("");
            it->setTextAlignment(Qt::AlignCenter);
            m_motorTable->setItem(i, c, it);
        }
        m_motorTable->item(i, 0)->setText(jointName(i));

        // 限位列常驻显示允许范围，方便人一眼判断
        const QString lim = QString("%1 ~ %2")
                                .arg(kLimDeg[j][0], 0, 'f', 0)
                                .arg(kLimDeg[j][1], 0, 'f', 0);
        m_motorTable->item(i, 6)->setText(lim);
        m_motorTable->item(i, 6)->setForeground(QColor(kCssGray));
    }

    lay->addWidget(m_motorTable);
    return box;
}

// -----------------------------------------------------------------------------
// ⑥ CAN 总线状态面板
// -----------------------------------------------------------------------------
QWidget *MainWindow::buildBusStatusBox()
{
    auto *box = new QGroupBox("⑥ CAN 总线状态");
    auto *lay = new QVBoxLayout(box);
    lay->setContentsMargins(8, 8, 8, 8);
    lay->setSpacing(6);

    m_busTable = new QTableWidget(0, 4);
    m_busTable->setHorizontalHeaderLabels({"通道", "状态", "波特率", "错误计数"});
    m_busTable->verticalHeader()->setVisible(false);
    m_busTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_busTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_busTable->setAlternatingRowColors(true);
    m_busTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_busTable->verticalHeader()->setDefaultSectionSize(27);

    QFont mono("monospace");
    mono.setStyleHint(QFont::Monospace);
    mono.setPointSize(11);
    m_busTable->setFont(mono);

    lay->addWidget(m_busTable);

    auto *btnRow = new QHBoxLayout();
    auto *btnRef = new QPushButton("立即刷新");
    btnRef->setMinimumHeight(32);
    connect(btnRef, &QPushButton::clicked, this, &MainWindow::onRefreshBusStatus);
    btnRow->addWidget(btnRef);
    btnRow->addStretch(1);

    m_frameLabel = new QLabel("发送 0 帧 / 接收 0 帧");
    m_frameLabel->setStyleSheet("color:#555;font-size:12px;");
    btnRow->addWidget(m_frameLabel);
    lay->addLayout(btnRow);

    auto *note = new QLabel("数据来源：<code>ip -details -json link show</code>，每 1.5 秒刷新一次。");
    note->setWordWrap(true);
    note->setStyleSheet("color:#666;font-size:12px;");
    lay->addWidget(note);

    return box;
}

// -----------------------------------------------------------------------------
// ⑦ 日志
// -----------------------------------------------------------------------------
QWidget *MainWindow::buildLogBox()
{
    auto *box = new QGroupBox("⑦ 运行日志");
    auto *lay = new QVBoxLayout(box);
    lay->setContentsMargins(8, 8, 8, 8);

    auto makeLog = []()
    {
        auto *e = new QPlainTextEdit();
        e->setReadOnly(true);
        e->setMaximumBlockCount(5000); // 环形缓冲，防止内存无限增长
        QFont f("monospace");
        f.setStyleHint(QFont::Monospace);
        f.setPointSize(11);
        e->setFont(f);
        e->setLineWrapMode(QPlainTextEdit::NoWrap);
        e->setMinimumHeight(90);
        return e;
    };

    m_logLower = makeLog();
    m_logUpper = makeLog();
    m_logGoto = makeLog();
    m_logSys = makeLog();

    auto *tabs = new QTabWidget();
    tabs->addTab(m_logSys, "系统消息");
    tabs->addTab(m_logLower, "下位机");
    tabs->addTab(m_logUpper, "上位机");
    tabs->addTab(m_logGoto, "回零");
    tabs->setMinimumHeight(110);

    lay->addWidget(tabs);
    return box;
}

// =============================================================================
// 工具函数
// =============================================================================
void MainWindow::appendLog(QPlainTextEdit *dst, const QString &text)
{
    if (!dst || text.isEmpty())
        return;
    // 统一行尾，避免部分程序输出 \r\n 导致出现 ^M
    QString t = text;
    t.replace("\r\n", "\n");
    t.replace('\r', '\n');

    QScrollBar *sb = dst->verticalScrollBar();
    const bool atBottom = (sb->value() >= sb->maximum() - 4);

    dst->moveCursor(QTextCursor::End);
    dst->insertPlainText(t);
    if (atBottom)
    {
        sb->setValue(sb->maximum());
    }
}

void MainWindow::logSys(const QString &s)
{
    const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss");
    appendLog(m_logSys, QString("[%1] %2\n").arg(ts).arg(s));
    statusBar()->showMessage(s, 6000);
}

void MainWindow::setLabelState(QLabel *l, const QString &text, const char *color)
{
    if (!l)
        return;
    l->setText(text);
    // 只改字重和颜色，不碰字号 —— 有些标签单独调过大字号，不能被覆盖
    QFont f = l->font();
    f.setBold(true);
    l->setFont(f);
    l->setStyleSheet(QString("color:%1;").arg(color));
}

QString MainWindow::binPath(const QString &name) const
{
    return QDir(m_binDir).absoluteFilePath(name);
}

QString MainWindow::findKcanctl() const
{
    // 依次尝试：可执行文件目录 → 上一级 build → PATH
    const QStringList cands = {
        binPath("kcanctl"),
        QDir(m_binDir).absoluteFilePath("../kcanctl"),
        QString("/usr/local/bin/kcanctl"),
        QString("/usr/bin/kcanctl"),
    };
    for (const QString &c : cands)
    {
        QFileInfo fi(c);
        if (fi.exists() && fi.isExecutable())
            return fi.absoluteFilePath();
    }
    return QString(); // 交给 QStandardPaths / PATH 查找
}

/// 同步执行一个短命令（只用于 ip / kcanctl 这类毫秒级操作）
bool MainWindow::runSync(const QString &prog, const QStringList &args, QString *out)
{
    QProcess p;
    p.setProcessChannelMode(QProcess::MergedChannels);
    p.start(prog, args);
    if (!p.waitForStarted(2000))
    {
        if (out)
            *out = QString("无法启动 %1").arg(prog);
        return false;
    }
    if (!p.waitForFinished(5000))
    {
        p.kill();
        p.waitForFinished(1000);
        if (out)
            *out = QString("%1 执行超时").arg(prog);
        return false;
    }
    if (out)
        *out = QString::fromLocal8Bit(p.readAllStandardOutput());
    return p.exitCode() == 0;
}

// =============================================================================
// CAN 接口配置
// =============================================================================
void MainWindow::onApplyCanConfig()
{
    if (!isRoot())
    {
        QMessageBox::warning(
            this, "权限不足",
            "配置 CAN 接口需要 root 权限。\n\n"
            "请退出本程序，改用：\n"
            "    sudo ./run_ui_sudo.sh\n"
            "重新启动。");
        return;
    }

    int ok = 0, fail = 0;
    QString errDetail;

    for (const IfRow &r : m_ifRows)
    {
        if (!r.en->isChecked())
            continue;

        const QString ifname = r.name;
        const int bitrate = r.rate->value();

        QString out;
        bool good = false;

        // 优先用自带的 kcanctl（行为与文档一致）
        const QString kc = findKcanctl();
        if (!kc.isEmpty() && QFileInfo(kc).isExecutable())
        {
            good = runSync(kc, {"up", ifname, QString::number(bitrate)}, &out);
        }
        if (!good)
        {
            // 退路：直接 ip link（等价的三步操作）
            runSync("/usr/sbin/ip", {"link", "set", ifname, "down"}, nullptr);
            runSync("/usr/sbin/ip",
                    {"link", "set", ifname, "type", "can",
                     "bitrate", QString::number(bitrate)},
                    nullptr);
            good = runSync("/usr/sbin/ip", {"link", "set", ifname, "up"}, &out);
        }

        if (good)
        {
            ++ok;
            logSys(QString("CAN 配置成功：%1 @ %2 bit/s")
                       .arg(ifname)
                       .arg(bitrate));
        }
        else
        {
            ++fail;
            errDetail += QString("%1 @ %2 → %3\n")
                             .arg(ifname)
                             .arg(bitrate)
                             .arg(out.trimmed().isEmpty() ? "未知错误" : out.trimmed());
            logSys(QString("CAN 配置失败：%1").arg(ifname));
        }
    }

    if (ok == 0 && fail == 0)
    {
        QMessageBox::information(this, "没有勾选", "当前没有任何通道被勾选。");
        return;
    }

    onRefreshBusStatus();

    if (fail > 0)
    {
        QMessageBox::warning(
            this, "部分失败",
            QString("成功 %1 个，失败 %2 个。\n\n%3").arg(ok).arg(fail).arg(errDetail));
    }
    else
    {
        QMessageBox::information(
            this, "配置完成",
            QString("已成功拉起 %1 个 CAN 通道。\n"
                    "若某路仍显示 DOWN，请检查该通道的物理线缆与驱动。")
                .arg(ok));
    }
}

void MainWindow::onDownAllCan()
{
    if (!isRoot())
    {
        QMessageBox::warning(this, "权限不足",
                             "关闭 CAN 接口需要 root 权限，请用 sudo 启动本程序。");
        return;
    }

    if (QMessageBox::question(this, "确认",
                              "将关闭全部 CAN 通道（包括 can_mcu）。\n"
                              "这会中断上下位机通信，是否继续？",
                              QMessageBox::Yes | QMessageBox::No,
                              QMessageBox::No) != QMessageBox::Yes)
        return;

    // 先停监控，避免边关边收
    if (m_mon && m_mon->isOpen())
        onMonitorToggle(false);

    for (const IfRow &r : m_ifRows)
    {
        runSync("/usr/sbin/ip", {"link", "set", r.name, "down"}, nullptr);
    }

    logSys("已关闭全部 CAN 通道");
    onRefreshBusStatus();
}

void MainWindow::onRefreshBusStatus()
{
    // 用 ip -details -json link show 一次性获取所有接口信息
    QString out;
    runSync("/usr/sbin/ip", {"-details", "-json", "link", "show"}, &out);

    // 目标接口顺序
    QStringList targets;
    for (const IfRow &r : m_ifRows)
        targets << r.name;

    // 解析 JSON：iproute2 的输出对我们只需要少数几个字段，
    // 因此手工扫描而不是引入 JSON 库。
    struct Info
    {
        bool up = false;
        int bitrate = 0;
        QString state;
        int txErr = -1, rxErr = -1;
        bool found = false;
    };
    std::map<QString, Info> infos;

    const QStringList blocks = out.split(QString("{\"ifindex\""));
    for (int bi = 1; bi < blocks.size(); ++bi)
    {
        const QString &b = blocks[bi];
        QRegularExpression reName("\"ifname\"\\s*:\\s*\"([^\"]+)\"");
        auto mName = reName.match(b);
        if (!mName.hasMatch())
            continue;
        const QString name = mName.captured(1);

        Info info;
        info.found = true;

        // 注意：operstate 才是链路是否真正工作的判据；
        // flags 里的 "UP" 只代表 "接口被 set up 过"（DOWN 的接口也会带 UP 标志以外的东西），
        // 所以以 operstate 为准，flags 仅作为后备。
        bool haveOper = false;
        QRegularExpression reOper("\"operstate\"\\s*:\\s*\"([^\"]+)\"");
        auto mOper = reOper.match(b);
        if (mOper.hasMatch())
        {
            haveOper = true;
            info.up = mOper.captured(1).compare("UP", Qt::CaseInsensitive) == 0 ||
                      mOper.captured(1).compare("UNKNOWN", Qt::CaseInsensitive) == 0;
        }

        if (!haveOper)
        {
            QRegularExpression reFlags("\"flags\"\\s*:\\s*\\[([^\\]]*)\\]");
            auto mFlags = reFlags.match(b);
            if (mFlags.hasMatch())
            {
                // 必须匹配带引号的 "UP"，否则 LOWER_UP 也会被当成 UP
                info.up = mFlags.captured(1).contains("\"UP\"", Qt::CaseInsensitive);
            }
        }

        // 二次校验：CAN 接口若是 down，driver 通常也不会报 state，此处不做强判

        QRegularExpression reState("\"state\"\\s*:\\s*\"([^\"]+)\"");
        auto mState = reState.match(b);
        if (mState.hasMatch())
            info.state = mState.captured(1);

        QRegularExpression reRate("\"bitrate\"\\s*:\\s*(\\d+)");
        auto mRate = reRate.match(b);
        if (mRate.hasMatch())
            info.bitrate = mRate.captured(1).toInt();

        QRegularExpression reErr("\"berr_counter\"\\s*:\\s*\\{\\s*\"tx\"\\s*:\\s*(\\d+)"
                                 "\\s*,\\s*\"rx\"\\s*:\\s*(\\d+)");
        auto mErr = reErr.match(b);
        if (mErr.hasMatch())
        {
            info.txErr = mErr.captured(1).toInt();
            info.rxErr = mErr.captured(2).toInt();
        }

        infos[name] = info;
    }

    // ---- 更新接口配置行里的状态标签 ----
    for (const IfRow &r : m_ifRows)
    {
        auto it = infos.find(r.name);
        if (it == infos.end() || !it->second.found)
        {
            setLabelState(r.state, "不存在", kCssRed);
            continue;
        }
        const Info &i = it->second;
        if (i.up)
        {
            // 波特率左边的输入框已经显示了，这里只报【UP】加异常标记，
            // 文字短一倍，11pt 也绝不会被挤断行
            QString txt = "UP";
            if (!i.state.isEmpty() && i.state != "ERROR-ACTIVE")
                txt += " · " + i.state; // 例如 BUS-OFF / ERROR-PASSIVE
            setLabelState(r.state, txt, i.state == "ERROR-ACTIVE" ? kCssGreen : kCssOrange);
        }
        else
        {
            setLabelState(r.state, "DOWN", kCssGray);
        }
    }

    // ---- 更新总线状态表 ----
    m_busTable->setRowCount(targets.size());
    for (int row = 0; row < targets.size(); ++row)
    {
        const QString &name = targets[row];
        auto *c0 = new QTableWidgetItem(name);
        c0->setTextAlignment(Qt::AlignCenter);

        auto it = infos.find(name);
        QString stateTxt = "不存在";
        QString rateTxt = "—";
        QString errTxt = "—";
        QColor color(kCssRed);

        if (it != infos.end() && it->second.found)
        {
            const Info &i = it->second;
            stateTxt = i.up ? "UP" : "DOWN";
            if (!i.state.isEmpty())
                stateTxt += " " + i.state;
            rateTxt = i.bitrate > 0 ? QString::number(i.bitrate) : QString("未配置");
            if (i.txErr >= 0 || i.rxErr >= 0)
                errTxt = QString("tx %1 / rx %2")
                             .arg(i.txErr >= 0 ? i.txErr : 0)
                             .arg(i.rxErr >= 0 ? i.rxErr : 0);
            if (!i.up)
                color = QColor(kCssGray);
            else if (i.state == "ERROR-ACTIVE")
                color = QColor(kCssGreen);
            else if (i.state.isEmpty())
                color = QColor(kCssBlue);
            else
                color = QColor(kCssOrange);
        }

        auto *c1 = new QTableWidgetItem(stateTxt);
        c1->setTextAlignment(Qt::AlignCenter);
        c1->setForeground(color);
        auto *c2 = new QTableWidgetItem(rateTxt);
        c2->setTextAlignment(Qt::AlignCenter);
        auto *c3 = new QTableWidgetItem(errTxt);
        c3->setTextAlignment(Qt::AlignCenter);

        m_busTable->setItem(row, 0, c0);
        m_busTable->setItem(row, 1, c1);
        m_busTable->setItem(row, 2, c2);
        m_busTable->setItem(row, 3, c3);
    }

    if (m_frameLabel && m_mon)
    {
        m_frameLabel->setText(QString("发送 %1 帧 / 接收 %2 帧")
                                  .arg(m_mon->txFrames())
                                  .arg(m_mon->rxFrames()));
    }
}

// =============================================================================
// 轨迹文件
// =============================================================================
void MainWindow::onBrowseXlsx()
{
    const QString startDir =
        m_xlsxEdit->text().trimmed().isEmpty()
            ? QDir::homePath()
            : QFileInfo(m_xlsxEdit->text().trimmed()).absolutePath();

    const QString f = QFileDialog::getOpenFileName(
        this, "选择轨迹 xlsx 文件", startDir,
        "Excel 工作簿 (*.xlsx);;所有文件 (*)");
    if (!f.isEmpty())
        m_xlsxEdit->setText(f);
}

// =============================================================================
// 进程启停
// =============================================================================
void MainWindow::updateProcessButtons()
{
    const bool lowerRun = m_lower->state() != QProcess::NotRunning;
    const bool upperRun = m_upper->state() != QProcess::NotRunning;
    const bool gotoRun = m_goto->state() != QProcess::NotRunning;

    m_btnLowerStart->setEnabled(!lowerRun);
    m_btnLowerStop->setEnabled(lowerRun);
    m_btnLowerArm->setEnabled(lowerRun);
    m_btnLowerDisarm->setEnabled(lowerRun);

    m_btnUpperStart->setEnabled(!upperRun && !gotoRun);
    m_btnUpperStop->setEnabled(upperRun);

    // 回零要求：下位机在跑，且上位机没在跑（避免抢 can_mcu）
    m_btnGotoStart->setEnabled(lowerRun && !upperRun && !gotoRun);
    m_btnGotoStop->setEnabled(gotoRun);

    if (lowerRun)
        setLabelState(m_lowerState, "● 运行中 (PID " + QString::number(m_lower->processId()) + ")",
                      kCssGreen);
    else
        setLabelState(m_lowerState, "● 未运行", kCssGray);

    if (upperRun)
        setLabelState(m_upperState, "● 运行中 (PID " + QString::number(m_upper->processId()) + ")",
                      kCssGreen);
    else
        setLabelState(m_upperState, "● 未运行", kCssGray);

    if (gotoRun)
        setLabelState(m_gotoState, "● 运行中 (PID " + QString::number(m_goto->processId()) + ")",
                      kCssGreen);
    else
        setLabelState(m_gotoState, "● 未运行", kCssGray);
}

void MainWindow::onStartLower()
{
    if (m_lower->state() != QProcess::NotRunning)
        return;

    const QString exe = binPath("lower_control");
    if (!QFileInfo(exe).isExecutable())
    {
        QMessageBox::critical(this, "找不到程序",
                              QString("找不到可执行文件：\n%1\n\n"
                                      "请先在本目录编译：cmake .. && make -j4")
                                  .arg(exe));
        return;
    }

    logLower(QString("\n=== [下位机] 启动 %1 ===\n").arg(exe));
    logSys(QString("启动下位机：%1").arg(exe));

    m_lower->setWorkingDirectory(m_binDir);
    m_lower->start(exe, QStringList());
    if (!m_lower->waitForStarted(3000))
    {
        logLower(QString("启动失败：%1\n").arg(m_lower->errorString()));
        QMessageBox::critical(this, "启动失败", m_lower->errorString());
    }
    updateProcessButtons();
}

void MainWindow::onStopLower()
{
    if (m_lower->state() == QProcess::NotRunning)
        return;

    // 优雅退出：先写 quit，让它自己收尾（会驱抱闸）
    if (m_lower->state() == QProcess::Running)
    {
        logLower("\n=== [下位机] 发送 quit ===\n");
        m_lower->write("quit\n");
        m_lower->closeWriteChannel();
    }

    if (!m_lower->waitForFinished(4000))
    {
        logLower("\n=== [下位机] 超时，强制结束 ===\n");
        m_lower->terminate();
        if (!m_lower->waitForFinished(2000))
            m_lower->kill();
    }
    updateProcessButtons();
}

void MainWindow::onSendLowerCmd(const QString &cmd, const QString &label)
{
    if (m_lower->state() != QProcess::Running)
    {
        QMessageBox::information(this, "下位机未运行",
                                 QString("请先启动下位机，再执行【%1】。").arg(label));
        return;
    }
    m_lower->write((cmd + "\n").toLocal8Bit());
    logSys(QString("向下位机发送：%1").arg(cmd));
}

void MainWindow::onStartUpper()
{
    if (m_upper->state() != QProcess::NotRunning)
        return;

    const QString exe = binPath("upper_control");
    if (!QFileInfo(exe).isExecutable())
    {
        QMessageBox::critical(this, "找不到程序",
                              QString("找不到可执行文件：\n%1").arg(exe));
        return;
    }

    // ---- 校验轨迹文件 ----
    const QString xlsx = m_xlsxEdit->text().trimmed();
    if (xlsx.isEmpty())
    {
        QMessageBox::warning(this, "缺少轨迹文件",
                             "请先在【② 上位机轨迹文件】里指定 .xlsx 路径。");
        return;
    }
    if (!QFileInfo(xlsx).isFile())
    {
        QMessageBox::warning(this, "文件不存在",
                             QString("轨迹文件不存在：\n%1").arg(xlsx));
        return;
    }

    // ---- 提醒下位机状态（不强制，允许单独调试）----
    if (m_lower->state() == QProcess::NotRunning)
    {
        const auto r = QMessageBox::question(
            this, "下位机未运行",
            "下位机还没有启动。\n\n"
            "上位机会通过 can_mcu 发送轨迹，但下位机没起来就没人接收，\n"
            "（电机不会动，只是白跑一遍轨迹）。\n\n"
            "仍要启动上位机吗？",
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (r != QMessageBox::Yes)
            return;
    }

    // ---- 组装参数 ----
    QStringList args;
    args << "--can" << m_canIf;
    args << "--hz" << QString::number(m_hzSpin->value());
    args << "--xlsx" << xlsx;
    args << "--auto";

    const int loops = m_loopSpin->value();
    if (loops > 0)
        args << "--loop" << QString::number(loops);
    else
        args << "--loop" << "0";

    // upper_control 运行时也要开 can_mcu。让监控退成被动模式，减少总线负担。
    if (m_mon && m_mon->isOpen())
    {
        m_mon->setActivePolling(false);
        logSys("上位机已启动 → CAN 监控切换为【被动模式】（只收不发）");
    }

    logUpper(QString("\n=== [上位机] 启动 ===\n%1 %2\n\n")
                 .arg(exe)
                 .arg(args.join(' ')));
    logSys(QString("启动上位机：%1 %2").arg(exe).arg(args.join(' ')));

    m_upper->setWorkingDirectory(m_binDir);
    m_upper->start(exe, args);
    if (!m_upper->waitForStarted(3000))
    {
        logUpper(QString("启动失败：%1\n").arg(m_upper->errorString()));
        QMessageBox::critical(this, "启动失败", m_upper->errorString());
    }
    updateProcessButtons();
}

void MainWindow::onStopUpper()
{
    if (m_upper->state() == QProcess::NotRunning)
        return;

    // upper_control 是纯发送端，Ctrl-C / terminate 即可；
    // 真正的停机保护在下位机（它收不到流就会保持）。
    logUpper("\n=== [上位机] 终止 ===\n");
    m_upper->terminate();
    if (!m_upper->waitForFinished(3000))
        m_upper->kill();

    // 恢复主动轮询
    if (m_mon && m_mon->isOpen() && !m_monitorPausedForGoto)
    {
        m_mon->setActivePolling(true);
        logSys("上位机已停止 → CAN 监控恢复【主动模式】");
    }
    updateProcessButtons();
}

// =============================================================================
// 回零
// =============================================================================
void MainWindow::onStartGotoZero()
{
    if (m_goto->state() != QProcess::NotRunning)
        return;

    const QString exe = binPath("leg_goto_zero");
    if (!QFileInfo(exe).isExecutable())
    {
        QMessageBox::critical(this, "找不到程序",
                              QString("找不到可执行文件：\n%1").arg(exe));
        return;
    }

    const QString leg = m_legCombo->currentText().left(1);
    const double secs = m_gotoTimeSpin->value();

    const auto r = QMessageBox::question(
        this, "确认回零",
        QString("<b>即将执行：</b><br><br>"
                "&nbsp;&nbsp;目标腿臂：<b>%1</b><br>"
                "&nbsp;&nbsp;运动时长：<b>%2</b> 秒<br>"
                "&nbsp;&nbsp;目标位置：<b>0.00°</b>（4 个关节全部）<br><br>"
                "运动期间该腿臂会<b>真实移动</b>，请确认周围无人员与障碍物。<br>"
                "本程序会先自动关闭 CAN 监控（腾出 can_mcu），结束后再打开。")
            .arg(leg)
            .arg(secs, 0, 'f', 1),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (r != QMessageBox::Yes)
        return;

    // leg_goto_zero 自己独占 can_mcu → 先关监控
    if (m_mon && m_mon->isOpen())
    {
        logSys("回零需要独占 can_mcu → 暂时关闭 CAN 监控");
        m_mon->stop();
        m_monitorPausedForGoto = true;
        setLabelState(m_monStateLabel, "CAN 监控：已暂停（回零中）", kCssOrange);
        m_btnMonitor->setText("打开 CAN 监控");
    }

    QStringList args;
    args << "--leg" << leg;
    args << "--time" << QString::number(secs, 'f', 2);
    args << "--can" << m_canIf;
    args << "--auto";

    logGoto(QString("\n=== [回零] 启动 ===\n%1 %2\n\n")
                .arg(exe)
                .arg(args.join(' ')));
    logSys(QString("启动回零：%1 %2").arg(exe).arg(args.join(' ')));

    m_goto->setWorkingDirectory(m_binDir);
    m_goto->start(exe, args);
    if (!m_goto->waitForStarted(3000))
    {
        logGoto(QString("启动失败：%1\n").arg(m_goto->errorString()));
        QMessageBox::critical(this, "启动失败", m_goto->errorString());
    }

    // 回零结束后自动恢复监控
    connect(m_goto, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this, [this](int, QProcess::ExitStatus)
            {
                if (m_monitorPausedForGoto)
                {
                    m_monitorPausedForGoto = false;
                    logSys("回零已结束 → 重新打开 CAN 监控");
                    onMonitorToggle(true);
                } }, Qt::UniqueConnection);

    updateProcessButtons();
}

void MainWindow::onStopGotoZero()
{
    if (m_goto->state() == QProcess::NotRunning)
        return;

    // leg_goto_zero 没有 stdin 命令，只能发信号；
    // 它被中断后到达的最后一帧目标就是当前保持位置。
    logGoto("\n=== [回零] 用户中止 ===\n");
    if (QMessageBox::question(
            this, "确认中止",
            "中止回零会让腿臂停在当前位置（不是 0 位）。\n\n"
            "确定要中止吗？",
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
        return;

    m_goto->terminate();
    if (!m_goto->waitForFinished(3000))
        m_goto->kill();
    updateProcessButtons();
}

// =============================================================================
// 急停 / 清除报警 / 监控开关
// =============================================================================
void MainWindow::onEstop()
{
    logSys("★★★ 急停触发 ★★★");

    // 1) 让下位机自己走 estop 流程（抱闸 + 锁定，最彻底）
    if (m_lower->state() == QProcess::Running)
    {
        m_lower->write("estop\n");
        logSys("已向下位机发送 estop");
    }
    else
    {
        logSys("下位机未运行，跳过 estop 命令");
    }

    // 2) 无论如何都从 can_mcu 直接广播 0x02（双重保障）
    //    即使下位机进程挂了，只要它在收 CAN 就能被这帧拦住。
    if (m_mon && m_mon->isOpen())
    {
        m_mon->requestStopAll();
        logSys("已通过 can_mcu 向 16 个 ID 广播 0x02（退出流模式）");
    }
    else
    {
        logSys("⚠ CAN 监控未打开，无法直接广播 0x02；"
               "只能依赖下位机自身的 estop");
    }

    // 3) 终止上位机与回零，防止它们继续发目标位置
    if (m_upper->state() != QProcess::NotRunning)
    {
        logSys("终止上位机进程");
        m_upper->terminate();
        if (!m_upper->waitForFinished(1500))
            m_upper->kill();
    }
    if (m_goto->state() != QProcess::NotRunning)
    {
        logSys("终止回零进程");
        m_goto->terminate();
        if (!m_goto->waitForFinished(1500))
            m_goto->kill();
    }

    statusBar()->showMessage("急停已执行：全部关节抱闸。"
                             "解除顺序 → 下位机 resume → arm",
                             60000);

    QMessageBox::information(
        this, "急停已执行",
        "已执行急停：\n"
        "  · 下位机 estop（全部抱闸 + 锁定）\n"
        "  · can_mcu 广播 0x02\n"
        "  · 终止上位机 / 回零进程\n\n"
        "恢复步骤：\n"
        "  1) 确认现场安全\n"
        "  2) 点【resume 解除急停】\n"
        "  3) 点【arm 解抱闸】");
}

void MainWindow::onClearLimitAlarm()
{
    // 两条路：下位机的 clear 命令 + 总线上广播 0x03
    if (m_lower->state() == QProcess::Running)
        m_lower->write("clear\n");

    if (m_mon && m_mon->isOpen())
        m_mon->requestClearLimit();

    logSys("已发送清除软限位报警（clear + 0x03 广播）");
}

void MainWindow::onMonitorToggle(bool on)
{
    if (!on)
    {
        if (m_mon)
            m_mon->stop();
        setLabelState(m_monStateLabel, "CAN 监控：未打开", kCssGray);
        m_btnMonitor->setText("打开 CAN 监控");
        m_btnMonitor->setStyleSheet("");
        logSys("CAN 监控已关闭");
        return;
    }

    if (m_mon && m_mon->isOpen())
        return;

    const QString err = m_mon->start(m_canIf.toStdString(), 200);
    if (!err.isEmpty())
    {
        setLabelState(m_monStateLabel, "CAN 监控：打开失败", kCssRed);
        m_btnMonitor->setText("打开 CAN 监控");
        m_btnMonitor->setStyleSheet("");
        logSys(QString("CAN 监控打开失败：%1（接口 %2）").arg(err).arg(m_canIf));
        return;
    }

    // upper_control 在跑时就先被动
    const bool upperRun = m_upper->state() != QProcess::NotRunning;
    m_mon->setActivePolling(!upperRun);

    setLabelState(m_monStateLabel,
                  QString("CAN 监控：%1 / %2")
                      .arg(m_canIf)
                      .arg(upperRun ? "被动（只收）" : "主动（200ms 轮询）"),
                  kCssGreen);
    m_btnMonitor->setText("关闭 CAN 监控");
    m_btnMonitor->setStyleSheet(
        QString("QPushButton{color:%1;}").arg(kCssRed));
    logSys(QString("CAN 监控已打开：%1（%2）")
               .arg(m_canIf)
               .arg(upperRun ? "被动模式" : "主动模式"));
}

// =============================================================================
// 周期刷新
// =============================================================================
void MainWindow::tickProcesses()
{
    updateProcessButtons();

    // ---- 监控模式仲裁：upper_control 在跑就退成被动 ----
    if (m_mon && m_mon->isOpen() && !m_monitorPausedForGoto)
    {
        const bool upperRun = m_upper->state() != QProcess::NotRunning;
        if (upperRun == m_mon->activePolling())
        {
            // 状态不一致：需要调整
            m_mon->setActivePolling(!upperRun);
            setLabelState(m_monStateLabel,
                          QString("CAN 监控：%1 / %2")
                              .arg(m_canIf)
                              .arg(upperRun ? "被动（只收）" : "主动（200ms 轮询）"),
                          kCssGreen);
        }
    }

    // ---- 心跳 / 报警摘要 ----
    if (m_mon && m_mon->isOpen())
    {
        if (m_mon->heartbeatFresh())
        {
            const uint8_t b = m_mon->heartbeatBits();
            QStringList on;
            if (b & kHbBitStreaming)
                on << "流模式";
            if (b & kHbBitOnline)
                on << "采样中";
            if (b & kHbBitLimitLock)
                on << "限位锁定";
            setLabelState(m_heartbeatLabel,
                          QString("下位机心跳：正常 (%1)")
                              .arg(on.isEmpty() ? "空闲" : on.join("/")),
                          kCssGreen);
        }
        else
        {
            setLabelState(m_heartbeatLabel, "下位机心跳：无响应", kCssRed);
        }

        if (m_mon->limitLocked())
        {
            const int id = m_mon->alarmJointId();
            QString jn = "?";
            if (isJointId(id))
                jn = jointName(idToLeg(id) * kJointCount + idToJoint(id));
            const QString side = m_mon->alarmSide() == kLimitSideMax ? "上限" : "下限";
            setLabelState(m_alarmLabel,
                          QString("软限位报警：%1 %2 @ %3°")
                              .arg(jn)
                              .arg(side)
                              .arg(fmtSignedDeg(m_mon->alarmRad())),
                          kCssRed);
        }
        else
        {
            setLabelState(m_alarmLabel, "软限位：正常", kCssGreen);
        }
    }
}

void MainWindow::tickMotorTable()
{
    if (!m_mon || !m_mon->isOpen())
    {
        // 监控没开时，把在线列标成未知
        for (int i = 0; i < kTotalJoints; ++i)
        {
            auto *it = m_motorTable->item(i, 1);
            if (it && it->text() != "—")
            {
                it->setText("—");
                it->setForeground(QColor(kCssGray));
                m_motorTable->item(i, 2)->setText("—");
                m_motorTable->item(i, 5)->setText("—");
            }
        }
        return;
    }

    for (int i = 0; i < kTotalJoints; ++i)
    {
        const bool online = m_mon->jointOnline(i);
        const double rad = m_mon->jointRad(i);
        const int64_t frames = m_mon->jointFrames(i);

        auto *cOnline = m_motorTable->item(i, 1);
        if (online)
        {
            cOnline->setText("在线");
            cOnline->setForeground(QColor(kCssGreen));
            m_motorTable->item(i, 2)->setText(fmtDeg(rad));

            // 是否超限（关节端，用 lower_control 的默认限位判断）
            const int j = i % kJointCount;
            const double deg = rad * kRadToDeg;
            auto *cLim = m_motorTable->item(i, 6);
            if (deg < kLimDeg[j][0] || deg > kLimDeg[j][1])
            {
                cLim->setText(QString("%1 ~ %2 ⚠ 超限")
                                  .arg(kLimDeg[j][0], 0, 'f', 0)
                                  .arg(kLimDeg[j][1], 0, 'f', 0));
                cLim->setForeground(QColor(kCssRed));
            }
            else
            {
                cLim->setText(QString("%1 ~ %2")
                                  .arg(kLimDeg[j][0], 0, 'f', 0)
                                  .arg(kLimDeg[j][1], 0, 'f', 0));
                cLim->setForeground(QColor(kCssGray));
            }
        }
        else
        {
            cOnline->setText("离线");
            cOnline->setForeground(QColor(kCssRed));
            m_motorTable->item(i, 2)->setText("—");
            m_motorTable->item(i, 5)->setText("—");
        }

        m_motorTable->item(i, 5)->setText(QString::number(frames));
    }
}

void MainWindow::tickBusStatus()
{
    onRefreshBusStatus();
}

// =============================================================================
// 标志 / 入参
// =============================================================================
namespace
{

    struct CliArgs
    {
        QString canIf{"can_mcu"};
        int hz{10};
        int loop{1};
        QString binDir;
        bool autoMonitor{true};
        bool help{false};
        QString screenshot; ///< 调试用：启动后自动截图到该路径再退出
    };

    void printHelp()
    {
        std::printf(
            "\n"
            "3588 腿臂控制系统 · 图形控制界面\n"
            "\n"
            "用法：\n"
            "    sudo ./run_ui_sudo.sh [选项]\n"
            "    或（已具备 root / 仅用于查看时）\n"
            "    ./x3588_ui [选项]\n"
            "\n"
            "选项：\n"
            "    --can <接口>      上位机侧 CAN 接口（默认 can_mcu）\n"
            "    --hz <值>         上位机发送频率，透传给 upper_control（默认 10）\n"
            "    --loop <次>       轨迹循环次数，透传给 upper_control（默认 1）\n"
            "    --bin-dir <目录>  三个可执行文件所在目录（默认与本程序同目录）\n"
            "    --no-monitor      启动时不自动打开 CAN 监控\n"
            "    --screenshot <路径> 启动 1.5 秒后截图保存并退出（调试用）\n"
            "    -h, --help        显示本帮助\n"
            "\n"
            "说明：\n"
            "    · 配置 CAN 接口（ip link set ... up）需要 root，请用 sudo 启动。\n"
            "    · 一旦以 root 启动，界面内所有操作都不再需要输入密码。\n"
            "    · 非 root 也能打开界面、看日志、看关节反馈，只是改不了接口状态。\n"
            "\n");
    }

    bool parseCli(int argc, char **argv, CliArgs *a)
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string k = argv[i];
            auto needValue = [&](const char *name) -> const char *
            {
                if (i + 1 >= argc)
                {
                    std::fprintf(stderr, "错误：%s 缺少参数值\n", name);
                    return nullptr;
                }
                return argv[++i];
            };

            if (k == "-h" || k == "--help")
            {
                a->help = true;
                return true;
            }
            else if (k == "--can")
            {
                const char *v = needValue("--can");
                if (!v)
                    return false;
                a->canIf = QString::fromLocal8Bit(v);
            }
            else if (k == "--hz")
            {
                const char *v = needValue("--hz");
                if (!v)
                    return false;
                a->hz = std::atoi(v);
                if (a->hz <= 0)
                    a->hz = 10;
            }
            else if (k == "--loop")
            {
                const char *v = needValue("--loop");
                if (!v)
                    return false;
                a->loop = std::atoi(v);
                if (a->loop < 0)
                    a->loop = 1;
            }
            else if (k == "--bin-dir")
            {
                const char *v = needValue("--bin-dir");
                if (!v)
                    return false;
                a->binDir = QString::fromLocal8Bit(v);
            }
            else if (k == "--no-monitor")
            {
                a->autoMonitor = false;
            }
            else if (k == "--screenshot")
            {
                const char *v = needValue("--screenshot");
                if (!v)
                    return false;
                a->screenshot = QString::fromLocal8Bit(v);
            }
            else
            {
                std::fprintf(stderr, "错误：未知参数 %s\n（用 --help 查看用法）\n",
                             k.c_str());
                return false;
            }
        }
        return true;
    }

} // namespace

// =============================================================================
// main
// =============================================================================
int main(int argc, char **argv)
{
    CliArgs cli;
    if (!parseCli(argc, argv, &cli))
        return 2;
    if (cli.help)
    {
        printHelp();
        return 0;
    }

    QApplication app(argc, argv);
    app.setApplicationName("x3588_ui");
    app.setApplicationDisplayName("3588 腿臂控制系统");

    // ---- 中文字体：优先 Noto Sans CJK SC（3588 上已装），否则交给 Qt ----
    {
        const QStringList prefer = {
            "Noto Sans CJK SC",
            "Noto Sans CJK",
            "WenQuanYi Micro Hei",
            "Source Han Sans SC",
        };
        const QStringList avail = QFontDatabase().families();
        for (const QString &f : prefer)
        {
            if (avail.contains(f))
            {
                QFont font(f, 11);
                app.setFont(font);
                std::printf("使用字体：%s\n", f.toUtf8().constData());
                break;
            }
        }
    }

    MainWindow w;
    w.setBinDir(cli.binDir);
    w.show();

    // ---- 调试自检：--screenshot <路径>，启动 1.5 秒后截图并退出 ----
    if (!cli.screenshot.isEmpty())
    {
        QTimer::singleShot(1500, &w, [&w, &cli]
                           {
            const QPixmap shot = w.grab();
            if (shot.save(cli.screenshot))
                std::printf("截图已保存：%s (%dx%d)\n",
                            cli.screenshot.toUtf8().constData(),
                            shot.width(), shot.height());
            else
                std::fprintf(stderr, "截图保存失败：%s\n",
                             cli.screenshot.toUtf8().constData());
            QCoreApplication::quit(); });
    }

    return app.exec();
}

#include "x3588_ui.moc"