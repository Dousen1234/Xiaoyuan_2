// =============================================================================
//  claw_can.h  ——  手爪固件（claws_control_1）的最小 CAN 通信层（C++17）
//
//  属于 testcmd_ssj1110 自用实现，**不依赖** claws_control_1/host_demo 的任何代码。
//  Linux 原生 SocketCAN（AF_CAN + raw socket），零第三方依赖。
//
//  协议摘要（Classical CAN 2.0A / 1 Mbit/s / DLC=8 / 小端）
//  ---------------------------------------------------------------------------
//    base+0  命令帧  [0]ver=1  [1]seq  [2]opcode  [3]flags  [4..5]int16 目标(0.01°)
//                   [6]速度/掩码  [7]CRC-8
//    base+1  位置帧  [1]状态   [2..7] 三路角度(0.01°)
//    base+2  状态帧  [1]seq  [2]状态  [3]故障位  [4..5]同步误差(0.01°)
//                   [6]命令龄×10ms  [7]结果|0x80到位
//    base+3  诊断帧  [1..3] 三路霍尔码  [4..6] 三路占空比÷4(‰)  [7]计数器
//    base+4  参数帧  [1]seq [2]参数ID [3]通道 [4..5]int16 值 [6]结果 [7]CRC-8
//    base+5  故障帧  格式同状态帧
//
//  ⚠ 命令帧第 [6] 字节在不同 opcode 下含义不同
//      SET_TARGET / SET_TARGET_Mx : 速度 deci-rpm（0=默认 40，或 40..170）
//      RUN_IN                     : 电机掩码 bit0..bit2（写 0 等价于 7）
//      JOG / SEEK_OPEN            : 电机编号 1..3
// =============================================================================

#pragma once

#include <linux/can.h>

#include <array>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace claw {

// ============================== 协议常量 ==============================
inline constexpr uint8_t  kProtocolVersion = 1;
inline constexpr uint16_t kDefaultBaseId   = 11;
inline constexpr uint16_t kBaseIdMin       = 11;
inline constexpr uint16_t kBaseIdMax       = 120;
inline constexpr uint8_t  kFlagRelative    = 0x01;
inline constexpr int      kMotorCount      = 3;
inline constexpr uint8_t  kAllMotorsMask   = 0x07;

// 固件时序常量（App/Inc/claw_config.h）
inline constexpr int    kCommandWatchdogMs       = 500;
inline constexpr int    kDirectionQuietMs        = 300;
inline constexpr double kHeartbeatSeconds        = 0.2;
inline constexpr double kDirectionSettleSeconds  = 0.35;

// 故障位
inline constexpr uint8_t kFaultMotor1      = 0x01;
inline constexpr uint8_t kFaultMotor2      = 0x02;
inline constexpr uint8_t kFaultMotor3      = 0x04;
inline constexpr uint8_t kFaultSync        = 0x40;
inline constexpr uint8_t kFaultCommTimeout = 0x80;

// ============================== 基础类型 ==============================
class ClawError : public std::runtime_error {
 public:
  explicit ClawError(const std::string& what) : std::runtime_error(what) {}
};

/// Ctrl+C 专用异常，便于主程序区分退出码（130）。
class ClawInterrupted : public ClawError {
 public:
  explicit ClawInterrupted(const std::string& what) : ClawError(what) {}
};

enum class Opcode : uint8_t {
  SetTarget   = 0x01,
  Stop        = 0x02,
  ClearFault  = 0x03,
  SetZero     = 0x04,
  Heartbeat   = 0x05,
  SetTargetM1 = 0x06,
  SetTargetM2 = 0x07,
  SetTargetM3 = 0x08,
  Jog         = 0x09,
  SeekOpen    = 0x0A,
  ParamSet    = 0x0B,
  ParamGet    = 0x0C,
  Grip        = 0x0D,
  RunIn       = 0x0E,
  SetCanId    = 0x0F,
};

enum class State : uint8_t { Idle = 0, Moving = 1, Hold = 2, Fault = 3 };

enum class Result : uint8_t { Ok = 0, Busy = 1, Fault = 2, Range = 3, Invalid = 4 };

const char* state_name(State state);
const char* result_name(Result result);

/// 把故障位翻译成人话。
std::string fault_text(uint8_t mask);

// ============================== 通用小工具 ==============================
/// 秒级单调时钟，进程启动时刻为 0。
double now_seconds();

/// printf 风格拼字符串，省得到处写 ostringstream。
std::string sfmt(const char* fmt, ...);

std::string to_hex(const uint8_t* data, size_t len);

// ============================== 帧编解码 ==============================
using Frame = std::array<uint8_t, 8>;

std::string to_hex(const Frame& frame);

/// CRC-8/SAE-J1850：多项式 0x1D，初值 0xFF，末尾异或 0xFF。与固件逐位一致。
uint8_t crc8(const uint8_t* data, size_t len);

Frame encode_command(uint8_t sequence,
                     Opcode opcode,
                     uint8_t flags = 0,
                     int16_t target_cdeg = 0,
                     uint8_t speed_byte = 0);

/// RUN_IN：恒定 1000‰ 占空比连续转，方向由 direction(0/1) 决定。
Frame encode_run_in(uint8_t sequence, int direction, uint8_t motor_mask = kAllMotorsMask);

/// 相对位移命令（速度可调）。固件要求速度是 0 或 4.0..17.0 rpm。
Frame encode_relative_move(uint8_t sequence,
                           double angle_deg,
                           double speed_rpm,
                           uint8_t motor_mask = kAllMotorsMask);

// ============================== 反馈结构 ==============================
struct PositionFeedback {
  State  state = State::Idle;
  double angles_deg[kMotorCount] = {0.0, 0.0, 0.0};
};

struct StatusFeedback {
  uint8_t sequence       = 0;
  State   state          = State::Idle;
  uint8_t fault_mask     = 0;
  double  sync_error_deg = 0.0;
  int     command_age_ms = 0;
  Result  result         = Result::Ok;
  bool    target_reached = false;
};

struct DiagnosticFeedback {
  uint8_t hall_codes[kMotorCount]    = {0, 0, 0};
  int     duty_permille[kMotorCount] = {0, 0, 0};
  uint8_t counter                    = 0;
};

struct ParameterFeedback {
  uint8_t sequence   = 0;
  uint8_t param_id   = 0;
  int16_t value      = 0;
  Result  result     = Result::Ok;
};

struct Feedback {
  enum class Kind { Unknown, Position, Status, Fault, Diagnostic, Parameter };
  Kind kind = Kind::Unknown;

  PositionFeedback   position{};
  StatusFeedback     status{};
  DiagnosticFeedback diagnostic{};
  ParameterFeedback  parameter{};
};

// ============================== 传输层抽象 ==============================
/// 抽象出来是为了能塞一个"模拟固件"进去做离线自测。
class CanTransport {
 public:
  virtual ~CanTransport() = default;
  virtual void send(const can_frame& frame) = 0;
  /// 有帧返回 true 并填好 frame；超时返回 false。timeout_ms<=0 表示不阻塞。
  virtual bool recv(can_frame& frame, int timeout_ms) = 0;
};

/// 当前支持的通道类型。以后要接创芯 / PEAK 等，在这里加枚举 + 工厂分支即可。
enum class PortKind { SocketCan };

/// 按类型造一个传输层；失败抛 ClawError。
std::shared_ptr<CanTransport> make_transport(PortKind kind, const char* channel);

/// Linux 原生 SocketCAN。
class SocketCan : public CanTransport {
 public:
  explicit SocketCan(const std::string& iface);
  ~SocketCan() override;

  SocketCan(const SocketCan&)            = delete;
  SocketCan& operator=(const SocketCan&) = delete;

  void send(const can_frame& frame) override;
  bool recv(can_frame& frame, int timeout_ms) override;

  const std::string& interface_name() const { return iface_; }

 private:
  int         fd_ = -1;
  std::string iface_;
};

// ============================== 总线封装 ==============================
using FeedbackCallback = std::function<void(const Feedback&)>;

/// 一块驱动板 = 一条独立 CAN 总线 = 一个手爪。
class ClawBus {
 public:
  ClawBus(std::shared_ptr<CanTransport> transport,
          uint16_t base_id = kDefaultBaseId);
  /// 析构时必定尝试发 STOP（失败也静默），保证不会留一个正在动的电机。
  ~ClawBus();

  ClawBus(const ClawBus&)            = delete;
  ClawBus& operator=(const ClawBus&) = delete;

  uint16_t base_id() const { return base_; }
  uint8_t  sequence() const { return sequence_; }

  // ---------- 发送 ----------
  void    send_frame(const Frame& frame);
  uint8_t next_sequence();

  /// 发一条命令，返回它用的 sequence（用来等固件确认）。
  uint8_t command(Opcode opcode,
                  uint8_t flags       = 0,
                  int16_t target_cdeg = 0,
                  uint8_t speed_byte  = 0);

  uint8_t send_run_in(int direction, uint8_t motor_mask = kAllMotorsMask);
  uint8_t send_relative_move(double angle_deg,
                             double speed_rpm,
                             uint8_t motor_mask = kAllMotorsMask);

  /// 只刷固件里的命令时间戳，不改变任何动作状态。
  void heartbeat();

  // ---------- 接收 ----------
  std::optional<Feedback> poll(int timeout_ms = 50);
  Feedback                decode(const can_frame& frame);

  // ---------- 等待 / 阻塞 ----------
  /// 确认命令被固件接受；被拒绝、报故障或没有回帧都抛 ClawError。
  StatusFeedback expect_ok(uint8_t sequence,
                           double  timeout_s = 1.0,
                           const std::string& what = "命令");

  /// 在 seconds 秒内持续喂心跳并读反馈；出现故障立刻抛 ClawError。
  void pump(double seconds,
            const FeedbackCallback& on_feedback = nullptr,
            double heartbeat_period = kHeartbeatSeconds);

  /// 等位置运动结束（固件在 HOLD 且非 grip 时把 [7] 最高位置 1）。
  StatusFeedback wait_target_reached(double timeout_s);

  // ---------- 常用动作 ----------
  std::optional<StatusFeedback> stop(double timeout_s = 0.5);
  StatusFeedback                clear_fault();
  /// 停机后等待换向静默期（固件要求 ≥300 ms，否则下一条命令回 BUSY）。
  void settle(double seconds = kDirectionSettleSeconds);

  const PositionFeedback*   last_position() const { return last_position_ ? &*last_position_ : nullptr; }
  const StatusFeedback*     last_status() const { return last_status_ ? &*last_status_ : nullptr; }
  const DiagnosticFeedback* last_diagnostic() const { return last_diagnostic_ ? &*last_diagnostic_ : nullptr; }

 private:
  std::optional<StatusFeedback> wait_status(uint8_t sequence, double timeout_s);
  void                          check_interrupt() const;

  std::shared_ptr<CanTransport> transport_;
  uint16_t                      base_;
  uint8_t                       sequence_    = 0;
  // 心跳必须由实例级时钟驱动：若每次 pump() 调用都重置计时器，
  // 以 0.1s 步长分片调用时永远攒不满 0.2s，就会漏喂固件 500ms 看门狗。
  double                        next_beat_s_ = 0.0;

  std::optional<PositionFeedback>   last_position_;
  std::optional<StatusFeedback>     last_status_;
  std::optional<DiagnosticFeedback> last_diagnostic_;
};

// ============================== 中断处理 ==============================
/// 由主程序安装 SIGINT 处理器，把标志置 1。
extern volatile std::sig_atomic_t g_interrupt;
inline bool interrupted() { return g_interrupt != 0; }
void install_interrupt_handler();

}  // namespace claw