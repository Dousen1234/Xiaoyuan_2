// =============================================================================
//  claw_can.cpp  ——  claw_can.h 的实现
// =============================================================================

#include "claw_can.h"

#include <errno.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace claw {

volatile std::sig_atomic_t g_interrupt = 0;

namespace {

void on_sigint(int) { g_interrupt = 1; }

double monotonic_seconds() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) / 1e9;
}

const double g_epoch = monotonic_seconds();

}  // namespace

void install_interrupt_handler() {
  struct sigaction sa {};
  sa.sa_handler = on_sigint;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;  // 不设 SA_RESTART，让阻塞 recv 能被打断
  sigaction(SIGINT, &sa, nullptr);
}

double now_seconds() { return monotonic_seconds() - g_epoch; }

std::string sfmt(const char* fmt, ...) {
  char    stack_buf[512];
  va_list args;
  va_start(args, fmt);
  int needed = std::vsnprintf(stack_buf, sizeof(stack_buf), fmt, args);
  va_end(args);
  if (needed < 0) return std::string();
  if (static_cast<size_t>(needed) < sizeof(stack_buf)) return std::string(stack_buf);

  std::vector<char> heap(static_cast<size_t>(needed) + 1);
  va_start(args, fmt);
  std::vsnprintf(heap.data(), heap.size(), fmt, args);
  va_end(args);
  return std::string(heap.data());
}

std::string to_hex(const uint8_t* data, size_t len) {
  static const char* kDigits = "0123456789abcdef";
  std::string        out;
  out.reserve(len * 3);
  for (size_t i = 0; i < len; ++i) {
    if (i) out.push_back(' ');
    out.push_back(kDigits[data[i] >> 4]);
    out.push_back(kDigits[data[i] & 0x0F]);
  }
  return out;
}

std::string to_hex(const Frame& frame) { return to_hex(frame.data(), frame.size()); }

const char* state_name(State state) {
  switch (state) {
    case State::Idle:   return "IDLE";
    case State::Moving: return "MOVING";
    case State::Hold:   return "HOLD";
    case State::Fault:  return "FAULT";
  }
  return "?";
}

const char* result_name(Result result) {
  switch (result) {
    case Result::Ok:      return "OK";
    case Result::Busy:    return "BUSY";
    case Result::Fault:   return "FAULT";
    case Result::Range:   return "RANGE";
    case Result::Invalid: return "INVALID";
  }
  return "?";
}

std::string fault_text(uint8_t mask) {
  if (mask == 0) return "无故障";
  std::string out;
  auto        append = [&out](const std::string& piece) {
    if (!out.empty()) out += " + ";
    out += piece;
  };
  for (int motor = 1; motor <= kMotorCount; ++motor) {
    if (mask & (1u << (motor - 1))) append(sfmt("电机%d(堵转/霍尔异常)", motor));
  }
  if (mask & kFaultSync) append("同步超差或运动超时");
  if (mask & kFaultCommTimeout) append("通信超时(心跳丢失)");

  uint8_t unknown = static_cast<uint8_t>(mask & ~(0x07u | kFaultSync | kFaultCommTimeout));
  if (unknown) append(sfmt("未定义位0x%02X", unknown));
  return out;
}

// ============================== 编解码 ==============================
uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x1D)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return static_cast<uint8_t>(crc ^ 0xFF);
}

Frame encode_command(uint8_t sequence,
                     Opcode  opcode,
                     uint8_t flags,
                     int16_t target_cdeg,
                     uint8_t speed_byte) {
  Frame frame{};
  frame[0] = kProtocolVersion;
  frame[1] = sequence;
  frame[2] = static_cast<uint8_t>(opcode);
  frame[3] = flags;
  // 小端 int16
  frame[4] = static_cast<uint8_t>(static_cast<uint16_t>(target_cdeg) & 0xFF);
  frame[5] = static_cast<uint8_t>((static_cast<uint16_t>(target_cdeg) >> 8) & 0xFF);
  frame[6] = speed_byte;
  frame[7] = crc8(frame.data(), 7);
  return frame;
}

Frame encode_run_in(uint8_t sequence, int direction, uint8_t motor_mask) {
  if (direction != 0 && direction != 1) throw ClawError("RUN_IN 的 direction 只能是 0 或 1");
  if (motor_mask > kAllMotorsMask) throw ClawError("RUN_IN 的电机掩码只能是 0..7");
  // 第 [6] 字节 = 电机掩码（固件把它当 mask 用），写 0 固件会按 7 处理
  return encode_command(sequence, Opcode::RunIn, direction ? 1 : 0, 0, motor_mask);
}

Frame encode_relative_move(uint8_t sequence,
                           double  angle_deg,
                           double  speed_rpm,
                           uint8_t motor_mask) {
  Opcode opcode;
  switch (motor_mask) {
    case kAllMotorsMask: opcode = Opcode::SetTarget;   break;
    case 0x01:           opcode = Opcode::SetTargetM1; break;
    case 0x02:           opcode = Opcode::SetTargetM2; break;
    case 0x04:           opcode = Opcode::SetTargetM3; break;
    default:
      throw ClawError("位置模式只支持三路同时(7)或单路(1/2/4)");
  }

  const int drpm = static_cast<int>(std::lround(speed_rpm * 10.0));
  if (drpm != 0 && (drpm < 40 || drpm > 170))
    throw ClawError(sfmt("速度必须是 0(默认) 或 4.0..17.0 rpm，当前 %.2f rpm", speed_rpm));

  const double cdeg_f = std::round(angle_deg * 100.0);
  if (cdeg_f < -32768.0 || cdeg_f > 32767.0)
    throw ClawError(sfmt("目标角度 %.2f° 超出 int16（0.01° 单位）范围", angle_deg));

  return encode_command(sequence, opcode, kFlagRelative,
                        static_cast<int16_t>(cdeg_f),
                        static_cast<uint8_t>(drpm & 0xFF));
}

// ============================== SocketCan ==============================
SocketCan::SocketCan(const std::string& iface) : iface_(iface) {
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) throw ClawError(sfmt("创建 CAN socket 失败：%s", strerror(errno)));

  ifreq ifr{};
  if (iface.size() >= IFNAMSIZ) {
    ::close(fd_);
    fd_ = -1;
    throw ClawError(sfmt("网卡名过长：%s", iface.c_str()));
  }
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", iface.c_str());
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    const int saved = errno;
    ::close(fd_);
    fd_ = -1;
    throw ClawError(sfmt(
        "找不到 CAN 通道 %s：%s\n"
        "提示：先执行  sudo ip link set %s type can bitrate 1000000\n"
        "          sudo ip link set %s up",
        iface.c_str(), strerror(saved), iface.c_str(), iface.c_str()));
  }

  sockaddr_can addr{};
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    const int saved = errno;
    ::close(fd_);
    fd_ = -1;
    throw ClawError(sfmt("绑定 CAN 通道 %s 失败：%s", iface.c_str(), strerror(saved)));
  }

  // 关掉回环：本程序不需要读回自己发的帧
  int loopback = 0;
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  // 只按帧收，不启用 CAN-FD（固件是 CAN-FD OFF）
  timeval tv{};
  tv.tv_sec  = 0;
  tv.tv_usec = 20000;
  ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

SocketCan::~SocketCan() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SocketCan::send(const can_frame& frame) {
  if (fd_ < 0) throw ClawError("CAN 通道未打开");
  // 用 write 而不是 send：避免 SIGPIPE 把进程打死
  const ssize_t written = ::write(fd_, &frame, sizeof(frame));
  if (written != static_cast<ssize_t>(sizeof(frame)))
    throw ClawError(sfmt("发送 CAN 帧失败：%s", strerror(errno)));
}

bool SocketCan::recv(can_frame& frame, int timeout_ms) {
  if (fd_ < 0) return false;

  // ⚠ 坑（2026-11-16 实测确认）：
  //   Linux 上 SO_RCVTIMEO 设成 {0,0} 并不是"立刻返回"，而是"永不超时=永久阻塞"。
  //   早先这里用 tv{0,0} 实现 timeout_ms<=0，导致 poll(0) 在缓冲区空时把整个
  //   程序卡死在 read() 上（实机现象：打印"目标坐标"之后就再也不动了，
  //   连相对位移命令都没发出去）。
  //   正确做法：timeout_ms<=0 时用 recv(MSG_DONTWAIT)，内核保证立刻返回 EAGAIN。

  ssize_t n;
  if (timeout_ms <= 0) {
    // 非阻塞：不依赖超时值，直接用 MSG_DONTWAIT
    n = ::recv(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
  } else {
    timeval tv{};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    n = ::read(fd_, &frame, sizeof(frame));
  }

  if (n == static_cast<ssize_t>(sizeof(can_frame))) return true;

  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return false;
  if (n < 0) throw ClawError(sfmt("接收 CAN 帧失败：%s", strerror(errno)));
  return false;
}

std::shared_ptr<CanTransport> make_transport(PortKind kind, const char* channel) {
  switch (kind) {
    case PortKind::SocketCan:
      return std::make_shared<SocketCan>(channel ? channel : "can0");
  }
  throw ClawError("不支持的通道类型");
}

// ============================== ClawBus ==============================
ClawBus::ClawBus(std::shared_ptr<CanTransport> transport, uint16_t base_id)
    : transport_(std::move(transport)), base_(base_id) {
  if (!transport_) throw ClawError("ClawBus 需要有效的传输层");
  if (base_ < kBaseIdMin || base_ > kBaseIdMax)
    throw ClawError(sfmt("CAN 基地址必须在 %u..%u，当前 %u", kBaseIdMin, kBaseIdMax, base_));
  next_beat_s_ = now_seconds() + kHeartbeatSeconds;
}

ClawBus::~ClawBus() {
  // 无论正常结束还是异常退出，都先停机再释放总线
  try {
    stop(0.3);
  } catch (...) {
    // 析构里绝不能再抛
  }
}

void ClawBus::check_interrupt() const {
  if (interrupted()) throw ClawInterrupted("用户中断(Ctrl+C)");
}

uint8_t ClawBus::next_sequence() {
  ++sequence_;
  return sequence_;
}

void ClawBus::send_frame(const Frame& frame) {
  can_frame cf{};
  cf.can_id  = base_;
  cf.can_dlc = 8;
  std::memcpy(cf.data, frame.data(), 8);
  transport_->send(cf);
}

uint8_t ClawBus::command(Opcode opcode, uint8_t flags, int16_t target_cdeg, uint8_t speed_byte) {
  const uint8_t seq = next_sequence();
  send_frame(encode_command(seq, opcode, flags, target_cdeg, speed_byte));
  return seq;
}

uint8_t ClawBus::send_run_in(int direction, uint8_t motor_mask) {
  const uint8_t seq = next_sequence();
  send_frame(encode_run_in(seq, direction, motor_mask));
  return seq;
}

uint8_t ClawBus::send_relative_move(double angle_deg, double speed_rpm, uint8_t motor_mask) {
  const uint8_t seq = next_sequence();
  send_frame(encode_relative_move(seq, angle_deg, speed_rpm, motor_mask));
  return seq;
}

void ClawBus::heartbeat() {
  // seq 不变刷时间戳，固件只用它重置 last_command_tick
  send_frame(encode_command(sequence_, Opcode::Heartbeat));
  next_beat_s_ = now_seconds() + kHeartbeatSeconds;
}

std::optional<Feedback> ClawBus::poll(int timeout_ms) {
  can_frame cf{};
  if (!transport_->recv(cf, timeout_ms)) return std::nullopt;
  return decode(cf);
}

Feedback ClawBus::decode(const can_frame& cf) {
  Feedback feedback;

  if (cf.can_id & CAN_ERR_FLAG) return feedback;                  // 错误帧
  if (cf.can_id & CAN_RTR_FLAG) return feedback;                  // 远程帧
  if (cf.can_id & CAN_EFF_FLAG) return feedback;                  // 扩展帧，固件不用
  if (cf.can_dlc != 8) return feedback;
  if (cf.data[0] != kProtocolVersion) return feedback;

  const uint16_t id = static_cast<uint16_t>(cf.can_id & CAN_SFF_MASK);

  if (id == base_ + 1) {
    PositionFeedback p;
    p.state = static_cast<State>(cf.data[1]);
    for (int m = 0; m < kMotorCount; ++m) {
      const int16_t raw = static_cast<int16_t>(
          static_cast<uint16_t>(cf.data[2 + m * 2]) |
          (static_cast<uint16_t>(cf.data[3 + m * 2]) << 8));
      p.angles_deg[m] = raw / 100.0;
    }
    last_position_ = p;
    feedback.kind  = Feedback::Kind::Position;
    feedback.position = p;
    return feedback;
  }

  if (id == base_ + 2 || id == base_ + 5) {
    StatusFeedback s;
    s.sequence   = cf.data[1];
    s.state      = static_cast<State>(cf.data[2]);
    s.fault_mask = cf.data[3];
    const int16_t sync_raw = static_cast<int16_t>(
        static_cast<uint16_t>(cf.data[4]) | (static_cast<uint16_t>(cf.data[5]) << 8));
    s.sync_error_deg = sync_raw / 100.0;
    s.command_age_ms = static_cast<int>(cf.data[6]) * 10;
    s.result         = static_cast<Result>(cf.data[7] & 0x0F);
    s.target_reached = (cf.data[7] & 0x80) != 0;
    last_status_     = s;
    feedback.kind    = (id == base_ + 5) ? Feedback::Kind::Fault : Feedback::Kind::Status;
    feedback.status  = s;
    return feedback;
  }

  if (id == base_ + 3) {
    DiagnosticFeedback d;
    for (int m = 0; m < kMotorCount; ++m) {
      d.hall_codes[m]    = cf.data[1 + m];
      d.duty_permille[m] = static_cast<int>(cf.data[4 + m]) * 4;
    }
    d.counter      = cf.data[7];
    last_diagnostic_ = d;
    feedback.kind    = Feedback::Kind::Diagnostic;
    feedback.diagnostic = d;
    return feedback;
  }

  if (id == base_ + 4) {
    // [1]=seq [2]=参数ID(存在 flags 字段) [3]=参数通道(存在 speed 字段)
    // [4..5]=int16 值 [6]=执行结果 [7]=CRC
    ParameterFeedback p;
    p.sequence = cf.data[1];
    p.param_id = cf.data[2];
    p.value    = static_cast<int16_t>(
        static_cast<uint16_t>(cf.data[4]) | (static_cast<uint16_t>(cf.data[5]) << 8));
    p.result        = static_cast<Result>(cf.data[6] & 0x0F);
    feedback.kind   = Feedback::Kind::Parameter;
    feedback.parameter = p;
    return feedback;
  }

  return feedback;  // Unknown
}

std::optional<StatusFeedback> ClawBus::wait_status(uint8_t sequence, double timeout_s) {
  const double deadline = now_seconds() + timeout_s;
  while (true) {
    const double remaining = deadline - now_seconds();
    if (remaining <= 0.0) return std::nullopt;
    check_interrupt();

    const int slice_ms = static_cast<int>(std::clamp(remaining * 1000.0, 1.0, 50.0));
    auto      item     = poll(slice_ms);
    if (!item) continue;
    if (item->kind == Feedback::Kind::Status || item->kind == Feedback::Kind::Fault) {
      if (item->status.sequence == sequence) return item->status;
    }
  }
}

StatusFeedback ClawBus::expect_ok(uint8_t sequence, double timeout_s, const std::string& what) {
  auto feedback = wait_status(sequence, timeout_s);
  if (!feedback.has_value())
    throw ClawError(sfmt("%s：%.1fs 内没收到 seq=%u 的状态帧（CAN 没通？基地址不对？）",
                         what.c_str(), timeout_s, sequence));
  if (feedback->fault_mask != 0 || feedback->state == State::Fault)
    throw ClawError(sfmt("%s：固件报故障 %s", what.c_str(),
                         fault_text(feedback->fault_mask).c_str()));
  if (feedback->result != Result::Ok)
    throw ClawError(sfmt(
        "%s：固件拒绝，result=%s（BUSY=正在动/还没静默够, RANGE=参数越界, INVALID=该路未标定）",
        what.c_str(), result_name(feedback->result)));
  return *feedback;
}

void ClawBus::pump(double seconds, const FeedbackCallback& on_feedback, double heartbeat_period) {
  const double deadline = now_seconds() + std::max(0.0, seconds);
  while (true) {
    const double now = now_seconds();
    if (now >= deadline) return;
    check_interrupt();

    if (heartbeat_period > 0.0 && now >= next_beat_s_) heartbeat();

    const int slice_ms =
        static_cast<int>(std::clamp((deadline - now) * 1000.0, 1.0, 50.0));
    auto item = poll(slice_ms);
    if (!item) continue;

    if (on_feedback) on_feedback(*item);

    if (item->kind == Feedback::Kind::Status || item->kind == Feedback::Kind::Fault) {
      if (item->status.fault_mask != 0 || item->status.state == State::Fault)
        throw ClawError(
            sfmt("固件报故障 %s", fault_text(item->status.fault_mask).c_str()));
    }
  }
}

StatusFeedback ClawBus::wait_target_reached(double timeout_s) {
  const double deadline = now_seconds() + timeout_s;
  while (true) {
    const double now = now_seconds();
    if (now >= deadline)
      throw ClawError(sfmt("%.1fs 内没等到到位（运动卡住或速度太慢）", timeout_s));
    check_interrupt();

    if (now >= next_beat_s_) heartbeat();

    const int slice_ms = static_cast<int>(std::clamp((deadline - now) * 1000.0, 1.0, 50.0));
    auto      item     = poll(slice_ms);
    if (!item) continue;

    if (item->kind == Feedback::Kind::Status || item->kind == Feedback::Kind::Fault) {
      const StatusFeedback& s = item->status;
      if (s.fault_mask != 0 || s.state == State::Fault)
        throw ClawError(sfmt("固件报故障 %s", fault_text(s.fault_mask).c_str()));
      if (s.target_reached || s.state == State::Hold || s.state == State::Idle) return s;
    }
  }
}

std::optional<StatusFeedback> ClawBus::stop(double timeout_s) {
  const uint8_t seq = command(Opcode::Stop);
  return wait_status(seq, timeout_s);
}

StatusFeedback ClawBus::clear_fault() {
  const uint8_t seq = command(Opcode::ClearFault);
  auto         feedback = wait_status(seq, 1.0);
  if (!feedback.has_value())
    throw ClawError("CLEAR_FAULT：1.0s 内没收到状态帧（CAN 没通？基地址不对？）");

  // 固件清故障前会先校验三路霍尔码是否合法。只接了部分电机时，其余通道的
  // 霍尔线悬空（GPIO_NOPULL）会读成 0x07，固件随即拒绝清理并回 result=FAULT。
  // 这种情况下只要本来就没有故障锁存，就不该阻塞后续动作，仅提示使用者。
  if (feedback->result == Result::Fault && feedback->fault_mask == 0 &&
      feedback->state != State::Fault) {
    std::fprintf(stderr,
                 "      ⚠ 固件拒绝清故障（有霍尔通道悬空，码 0x07），"
                 "但 fault_mask=0，继续运行\n");
    return *feedback;
  }
  if (feedback->fault_mask != 0 || feedback->state == State::Fault)
    throw ClawError(sfmt("CLEAR_FAULT：固件报故障 %s",
                         fault_text(feedback->fault_mask).c_str()));
  if (feedback->result != Result::Ok)
    throw ClawError(sfmt("CLEAR_FAULT：固件拒绝，result=%s", result_name(feedback->result)));
  return *feedback;
}

void ClawBus::settle(double seconds) { pump(seconds); }

}  // namespace claw