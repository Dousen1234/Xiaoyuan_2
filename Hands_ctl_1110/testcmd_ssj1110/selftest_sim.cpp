// =============================================================================
//  selftest_sim.cpp  ——  离线自测：用模拟固件验证整条控制流程，不需要硬件
//
//  把 CanTransport 换成一个"假的固件"，它按 claws_control_1 的行为回帧：
//    - 校验 CRC / 版本号，非法帧直接丢弃
//    - 处理 STOP / CLEAR_FAULT / HEARTBEAT / RUN_IN / SET_TARGET
//    - 500 ms 命令看门狗 → 锁 COMM_TIMEOUT
//    - 300 ms 换向静默 → 早发回 BUSY
//    - 位置运动按时间推进，到位后置 [7] 的 0x80
//
//  编译运行：
//    g++ -std=c++17 -O2 -Wall -Wextra -o selftest_sim claw_can.cpp selftest_sim.cpp
//    ./selftest_sim
// =============================================================================

#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "claw_can.h"

using namespace claw;

namespace {

/// 固件行程上限 CLAW_MAX_TARGET_MDEG = 120000（0.001°），即 120°。
constexpr double kMaxTravelDeg = 120.0;

int g_failures = 0;

void check(bool condition, const std::string& what) {
  if (condition) {
    std::printf("    \033[32m✓\033[0m %s\n", what.c_str());
  } else {
    std::printf("    \033[31m✗ %s\033[0m\n", what.c_str());
    ++g_failures;
  }
}

// ============================ 模拟固件 ============================
class SimBoard : public CanTransport {
 public:
  explicit SimBoard(uint16_t base_id) : base_(base_id) {
    // 固件初始：IDLE，无故障
    state_        = State::Idle;
    fault_mask_   = 0;
    last_command_ = now_seconds();
    last_stop_    = -1000.0;
    seq_          = 0;
  }

  // ---------- CanTransport ----------
  void send(const can_frame& frame) override {
    ++sent_count_;
    if (frame.can_id != base_) return;  // 只认命令 ID

    const uint8_t* d = frame.data;
    if (frame.can_dlc != 8) return;
    if (d[0] != kProtocolVersion) return;
    if (crc8(d, 7) != d[7]) {
      ++bad_crc_;
      return;
    }

    const uint8_t   seq    = d[1];
    const Opcode    opcode = static_cast<Opcode>(d[2]);
    const uint8_t   flags  = d[3];
    const int16_t   target = static_cast<int16_t>(
        static_cast<uint16_t>(d[4]) | (static_cast<uint16_t>(d[5]) << 8));
    const uint8_t   byte6 = d[6];
    const double    now   = now_seconds();

    opcodes_.push_back(static_cast<int>(opcode));

    // ---- 命令看门狗：MOVING 中超过 500ms 没命令 → COMM_TIMEOUT ----
    if ((state_ == State::Moving) && (now - last_command_) * 1000.0 > kCommandWatchdogMs) {
      latch_fault(kFaultCommTimeout);
    }
    last_command_ = now;

    if (opcode == Opcode::Heartbeat) {
      ++heartbeat_count_;
      respond_status(seq, Result::Ok);
      return;
    }

    if (state_ == State::Fault && opcode != Opcode::ClearFault && opcode != Opcode::Stop) {
      respond_status(seq, Result::Fault);
      return;
    }

    // ---- 重复 seq 幂等（固件用 last_action_sequence 去重）----
    if (opcode != Opcode::Heartbeat && opcode != Opcode::Stop) {
      if (seq == last_action_seq_) {
        respond_status(seq, last_action_result_);
        return;
      }
      last_action_seq_ = seq;
    }

    switch (opcode) {
      case Opcode::Stop:
        state_     = State::Idle;
        last_stop_ = now;
        ++stop_count_;
        respond_status(seq, Result::Ok);
        return;

      case Opcode::ClearFault:
        if (state_ == State::Fault) {
          state_      = State::Idle;
          fault_mask_ = 0;
        }
        ++clear_count_;
        respond_status(seq, Result::Ok);
        return;

      case Opcode::RunIn: {
        const int        direction = flags ? 1 : -1;
        const uint8_t    mask      = (byte6 == 0) ? kAllMotorsMask : byte6;
        if (mask > kAllMotorsMask || mask == 0) {
          respond_status(seq, Result::Range);
          return;
        }
        if (state_ == State::Moving) {
          respond_status(seq, Result::Busy);
          return;
        }
        // 换向静默 300ms
        if ((now - last_stop_) * 1000.0 < kDirectionQuietMs) {
          respond_status(seq, Result::Busy);
          return;
        }
        state_    = State::Moving;
        run_dir_  = direction;
        run_mask_ = mask;
        run_count_ += 1;
        run_dirs_.push_back(flags ? 1 : 0);  // 记录原始的 DIR 电平 0/1
        run_masks_.push_back(mask);
        respond_status(seq, Result::Ok);
        return;
      }

      case Opcode::SetTarget:
      case Opcode::SetTargetM1:
      case Opcode::SetTargetM2:
      case Opcode::SetTargetM3: {
        const double speed_rpm = byte6 / 10.0;
        if (byte6 != 0 && (byte6 < 40 || byte6 > 170)) {
          respond_status(seq, Result::Range);
          return;
        }
        if (state_ == State::Moving) {
          respond_status(seq, Result::Busy);
          return;
        }
        if ((now - last_stop_) * 1000.0 < kDirectionQuietMs) {
          respond_status(seq, Result::Busy);
          return;
        }
        const double effective_speed = (speed_rpm == 0.0) ? 4.0 : speed_rpm;
        const double angle           = target / 100.0;
        if (std::fabs(angle) > kMaxTravelDeg) {
          respond_status(seq, Result::Range);
          return;
        }
        state_        = State::Moving;
        move_angle_   = angle;
        move_speed_   = effective_speed;
        move_started_ = now;
        move_duration_ = std::fabs(angle) / (effective_speed * 6.0);
        ++move_count_;
        respond_status(seq, Result::Ok);
        return;
      }

      default:
        respond_status(seq, Result::Ok);
        return;
    }
  }

  bool recv(can_frame& frame, int timeout_ms) override {
    tick();

    if (pending_.empty()) {
      if (timeout_ms > 0) {
        // 真实场景是阻塞等到超时；这里每次返回一小片，由调用方重试
        scheduled_delay_ += timeout_ms;
      }
      return false;
    }

    frame = pending_.front();
    pending_.pop_front();
    return true;
  }

  // ---------- 观测 ----------
  int  run_count() const { return run_count_; }
  int  stop_count() const { return stop_count_; }
  int  heartbeat_count() const { return heartbeat_count_; }
  int  clear_count() const { return clear_count_; }
  int  move_count() const { return move_count_; }
  int  bad_crc() const { return bad_crc_; }
  int  watchdog_trips() const { return watchdog_trips_; }
  const std::vector<int>& opcodes() const { return opcodes_; }
  const std::vector<int>& run_dirs() const { return run_dirs_; }
  const std::vector<uint8_t>& run_masks() const { return run_masks_; }
  State state() const { return state_; }
  uint8_t fault_mask() const { return fault_mask_; }

  /// 注入故障，模拟堵转。
  void inject_stall(int motor) {
    fault_mask_ |= static_cast<uint8_t>(1u << (motor - 1));
    state_ = State::Fault;
    move_duration_ = 0.0;
    respond_status(last_action_seq_, Result::Fault);  // 立刻推一帧故障帧出去
  }

  /// 强制把状态推到 MOVING 但不给命令，用来测看门狗。
  void force_moving() { state_ = State::Moving; }

 private:
  void tick() {
    if (state_ != State::Moving) return;
    const double now = now_seconds();

    if (move_duration_ > 0.0) {
      const double elapsed = now - move_started_;
      position_deg_        = move_started_angle_ + move_angle_ * std::min(1.0, elapsed / move_duration_);
      if (elapsed >= move_duration_) {
        state_         = State::Hold;
        move_duration_ = 0.0;
        target_reached_ = true;
        respond_status(last_action_seq_, Result::Ok);
      }
      return;
    }

    // RUN_IN：每 20ms 发一帧诊断 + 每 100ms 发状态帧
    if (now - last_diag_ >= 0.02) {
      last_diag_ = now;
      emit_diagnostic();
    }
    if (now - last_status_ >= 0.1) {
      last_status_ = now;
      respond_status(last_action_seq_, Result::Ok);
    }

    // 看门狗
    if ((now - last_command_) * 1000.0 > kCommandWatchdogMs) {
      latch_fault(kFaultCommTimeout);
    }
  }

  void latch_fault(uint8_t bit) {
    if (!(fault_mask_ & bit)) ++watchdog_trips_;
    fault_mask_ |= bit;
    state_ = State::Fault;
    move_duration_ = 0.0;
  }

  void emit_diagnostic() {
    can_frame f{};
    f.can_id  = base_ + 3;
    f.can_dlc = 8;
    f.data[0] = kProtocolVersion;
    for (int m = 0; m < kMotorCount; ++m) {
      f.data[1 + m] = static_cast<uint8_t>((run_mask_ >> m) & 1 ? 3 : 0);  // 霍尔码
      f.data[4 + m] = static_cast<uint8_t>(run_mask_ >> m & 1 ? 250 : 0);  // 1000‰ / 4
    }
    f.data[7] = static_cast<uint8_t>(++diag_counter_);
    pending_.push_back(f);
  }

  void respond_status(uint8_t sequence, Result result) {
    can_frame f{};
    f.can_id  = (state_ == State::Fault) ? base_ + 5 : base_ + 2;
    f.can_dlc = 8;
    f.data[0] = kProtocolVersion;
    f.data[1] = sequence;
    f.data[2] = static_cast<uint8_t>(state_);
    f.data[3] = fault_mask_;
    const int16_t sync = 0;
    f.data[4]          = static_cast<uint8_t>(sync & 0xFF);
    f.data[5]          = static_cast<uint8_t>((sync >> 8) & 0xFF);
    f.data[6] = 0;
    f.data[7] = static_cast<uint8_t>(result);
    if (state_ == State::Hold && target_reached_) f.data[7] |= 0x80;
    last_action_result_ = result;
    pending_.push_back(f);
  }

  uint16_t base_;
  State    state_        = State::Idle;
  uint8_t  fault_mask_   = 0;
  double   last_command_ = 0.0;
  double   last_stop_    = -1000.0;
  uint8_t  seq_          = 0;

  uint8_t  last_action_seq_    = 0;
  Result   last_action_result_ = Result::Ok;

  int run_count_   = 0;
  int stop_count_  = 0;
  int heartbeat_count_ = 0;
  int clear_count_ = 0;
  int move_count_  = 0;
  int bad_crc_     = 0;
  int sent_count_  = 0;
  int watchdog_trips_ = 0;

  double scheduled_delay_ = 0.0;

  int            run_dir_  = 0;
  uint8_t        run_mask_ = 0;
  double         last_diag_   = 0.0;
  double         last_status_ = 0.0;
  uint8_t        diag_counter_ = 0;

  double move_angle_        = 0.0;
  double move_speed_        = 0.0;
  double move_started_      = 0.0;
  double move_duration_     = 0.0;
  double move_started_angle_ = 0.0;
  double position_deg_      = 0.0;
  bool   target_reached_    = false;

  std::deque<can_frame> pending_;
  std::vector<int>      opcodes_;
  std::vector<int>      run_dirs_;
  std::vector<uint8_t>  run_masks_;
};

// ============================ 测试用例 ============================
void test_crc() {
  std::printf("\n[1] CRC-8 与固件一致性\n");
  // 已知帧：RUN_IN seq=1 dir=1 mask=7 → 01 01 0e 01 00 00 07 c9
  Frame f = encode_run_in(1, 1, kAllMotorsMask);
  const std::string hex = to_hex(f);
  check(hex == "01 01 0e 01 00 00 07 c9", "RUN_IN 帧与固件参考实现逐字节一致：" + hex);
  check(crc8(f.data(), 7) == f[7], "CRC 自校验通过");

  // 全 0 载荷的 CRC（覆盖 256 个 seq）
  int mismatch = 0;
  for (int s = 0; s <= 255; ++s) {
    Frame g = encode_command(static_cast<uint8_t>(s), Opcode::Stop);
    if (crc8(g.data(), 7) != g[7]) ++mismatch;
  }
  check(mismatch == 0, "256 个 sequence 的 CRC 全部自洽");
}

void test_encoding() {
  std::printf("\n[2] 帧编码\n");
  Frame run_in = encode_run_in(1, 1, 7);
  check(run_in[2] == 0x0E, "RUN_IN opcode = 0x0E");
  check(run_in[3] == 0x01, "RUN_IN direction 落在 flags 字节");
  check(run_in[6] == 0x07, "RUN_IN 掩码落在 byte6");

  Frame move = encode_relative_move(2, 120.0, 8.0, 7);
  check(move[2] == 0x01, "三路相对位移用 opcode 0x01");
  check(move[3] == kFlagRelative, "置了 FLAG_RELATIVE");
  check(move[4] == 0xE0 && move[5] == 0x2E, "120.00° 编码为 12000 = 0x2EE0 小端");
  check(move[6] == 80, "8.0 rpm 编码为 80 deci-rpm");

  bool threw = false;
  try {
    encode_relative_move(3, 10.0, 8.0, 0x03);
  } catch (const ClawError&) {
    threw = true;
  }
  check(threw, "拒绝「任选两路」位置运动");

  threw = false;
  try {
    encode_relative_move(4, 10.0, 2.0, 7);
  } catch (const ClawError&) {
    threw = true;
  }
  check(threw, "拒绝 2.0 rpm（低于下限 4.0）");

  threw = false;
  try {
    encode_relative_move(5, 10.0, 20.0, 7);
  } catch (const ClawError&) {
    threw = true;
  }
  check(threw, "拒绝 20.0 rpm（高于上限 17.0）");
}

void test_decode() {
  std::printf("\n[3] 反馈帧解码\n");
  ClawBus bus(std::make_shared<SimBoard>(kDefaultBaseId), kDefaultBaseId);

  can_frame f{};
  f.can_id  = kDefaultBaseId + 1;
  f.can_dlc = 8;
  for (int i = 0; i < 8; ++i) f.data[i] = 0;
  f.data[0] = kProtocolVersion;
  f.data[1] = static_cast<uint8_t>(State::Hold);
  const int16_t angles[3] = {1234, -567, 0};
  for (int m = 0; m < 3; ++m) {
    f.data[2 + m * 2] = static_cast<uint8_t>(static_cast<uint16_t>(angles[m]) & 0xFF);
    f.data[3 + m * 2] = static_cast<uint8_t>((static_cast<uint16_t>(angles[m]) >> 8) & 0xFF);
  }
  Feedback fb = bus.decode(f);
  check(fb.kind == Feedback::Kind::Position, "认出位置帧");
  check(std::fabs(fb.position.angles_deg[0] - 12.34) < 1e-6, "角度 1 = +12.34°");
  check(std::fabs(fb.position.angles_deg[1] - (-5.67)) < 1e-6, "角度 2 = -5.67°（负数正确）");
  check(fb.position.state == State::Hold, "状态 = HOLD");

  // 状态帧
  can_frame s{};
  s.can_id  = kDefaultBaseId + 2;
  s.can_dlc = 8;
  for (int i = 0; i < 8; ++i) s.data[i] = 0;
  s.data[0] = kProtocolVersion;
  s.data[1] = 7;
  s.data[2] = static_cast<uint8_t>(State::Hold);
  s.data[3] = 0;
  s.data[7] = static_cast<uint8_t>(Result::Ok) | 0x80;
  Feedback fs = bus.decode(s);
  check(fs.kind == Feedback::Kind::Status, "认出状态帧");
  check(fs.status.sequence == 7, "sequence = 7");
  check(fs.status.target_reached, "解析出 target_reached");

  // 参数帧
  can_frame p{};
  p.can_id  = kDefaultBaseId + 4;
  p.can_dlc = 8;
  for (int i = 0; i < 8; ++i) p.data[i] = 0;
  p.data[0] = kProtocolVersion;
  p.data[1] = 4;   // seq
  p.data[2] = 6;   // param_id
  p.data[4] = 0xE8;  // 1000 = 0x03E8
  p.data[5] = 0x03;
  p.data[6] = static_cast<uint8_t>(Result::Ok);
  Feedback fp = bus.decode(p);
  check(fp.kind == Feedback::Kind::Parameter, "认出参数帧");
  check(fp.parameter.param_id == 6 && fp.parameter.value == 1000, "参数 ID / 值正确");

  // 错误帧
  can_frame bad{};
  bad.can_id  = static_cast<canid_t>(kDefaultBaseId + 2) | CAN_ERR_FLAG;
  bad.can_dlc = 8;
  Feedback fbad = bus.decode(bad);
  check(fbad.kind == Feedback::Kind::Unknown, "错误帧被忽略");
}

void test_run_in_flow(uint8_t mask) {
  std::printf("\n[4] RUN_IN 完整流程（mask=%u）\n", mask);
  auto  sim = std::make_shared<SimBoard>(kDefaultBaseId);
  ClawBus bus(sim, kDefaultBaseId);

  bus.clear_fault();
  bus.settle(0.05);

  const int cycles = 2;
  for (int c = 0; c < cycles; ++c) {
    for (int dir = 1; dir >= 0; --dir) {
      const uint8_t seq = bus.send_run_in(dir, mask);
      bus.expect_ok(seq, 1.0, "RUN_IN");
      double remaining = 0.3;
      while (remaining > 0.0) {
        const double step = std::min(0.1, remaining);
        bus.pump(step);
        remaining -= step;
      }
      bus.stop();
      bus.settle(kDirectionSettleSeconds);  // 固件要求 ≥300ms 换向静默
    }
  }

  check(sim->bad_crc() == 0, "没有 CRC 错误的帧");
  check(sim->run_count() == cycles * 2,
        sfmt("收到 %d 条 RUN_IN（期望 %d）", sim->run_count(), cycles * 2));
  check(sim->heartbeat_count() > 0,
        sfmt("心跳发了 %d 次", sim->heartbeat_count()));
  check(sim->stop_count() >= cycles * 2,
        sfmt("STOP 发了 %d 次", sim->stop_count()));
  check(sim->watchdog_trips() == 0, "固件看门狗没有触发（心跳没漏）");
  check(sim->state() != State::Fault, "最终不是 FAULT 状态");

  const std::vector<int>& dirs = sim->run_dirs();
  bool dirs_ok = dirs.size() == 4 && dirs[0] == 1 && dirs[1] == 0 && dirs[2] == 1 &&
                 dirs[3] == 0;
  std::string dir_text;
  for (int d : dirs) dir_text += std::to_string(d) + " ";
  check(dirs_ok, "方向序列 = 1,0,1,0，实际 " + dir_text);

  bool masks_ok = true;
  for (uint8_t m : sim->run_masks()) masks_ok = masks_ok && (m == mask);
  check(masks_ok, "所有 RUN_IN 的掩码一致");
}

void test_position_flow() {
  std::printf("\n[5] position 模式流程\n");
  auto    sim = std::make_shared<SimBoard>(kDefaultBaseId);
  ClawBus bus(sim, kDefaultBaseId);

  bus.clear_fault();
  bus.settle(0.05);

  const double angle = 24.0;  // 8 rpm × 6 × 0.5s
  const double speed = 8.0;
  for (int dir = 1; dir >= 0; --dir) {
    const double signed_angle = (dir == 1) ? angle : -angle;
    const uint8_t seq = bus.send_relative_move(signed_angle, speed, 7);
    bus.expect_ok(seq, 1.0, "位移命令");
    bus.wait_target_reached(3.0);
    bus.settle(0.05);
  }

  check(sim->move_count() == 2, sfmt("收到 %d 条位置命令", sim->move_count()));
  check(sim->bad_crc() == 0, "没有 CRC 错误的帧");
  check(sim->state() != State::Fault, "最终不是 FAULT 状态");
}

void test_range_rejection() {
  std::printf("\n[6] 越界参数被固件拒绝\n");
  auto    sim = std::make_shared<SimBoard>(kDefaultBaseId);
  ClawBus bus(sim, kDefaultBaseId);

  // 直接构造一条 200° 的帧（绕过本地校验），看固件是否回 RANGE
  const uint8_t seq = bus.next_sequence();
  bus.send_frame(encode_command(seq, Opcode::SetTarget, kFlagRelative, 20000, 80));
  bool caught = false;
  try {
    bus.expect_ok(seq, 1.0, "越界位移");
  } catch (const ClawError& exc) {
    caught = std::string(exc.what()).find("RANGE") != std::string::npos;
  }
  check(caught, "200° 被固件判为 RANGE 并抛出");
}

void test_fault_abort() {
  std::printf("\n[7] 故障时立即中止\n");
  auto    sim = std::make_shared<SimBoard>(kDefaultBaseId);
  ClawBus bus(sim, kDefaultBaseId);

  bus.clear_fault();
  bus.settle(0.05);

  const uint8_t seq = bus.send_run_in(1, 7);
  bus.expect_ok(seq, 1.0, "RUN_IN");

  sim->inject_stall(1);  // 模拟电机 1 堵转

  bool caught = false;
  try {
    // 故障帧会在下一轮 pump 里被读到
    for (int i = 0; i < 100; ++i) bus.pump(0.05);
  } catch (const ClawError& exc) {
    caught = std::string(exc.what()).find("固件报故障") != std::string::npos;
    std::printf("      捕获：%s\n", exc.what());
  }
  check(caught, "pump 检测到故障并抛异常");

  // 之后 expect_ok 也应该报故障
  bool caught2 = false;
  try {
    bus.expect_ok(bus.send_run_in(0, 7), 1.0, "反向 RUN_IN");
  } catch (const ClawError& exc) {
    caught2 = std::string(exc.what()).find("固件报故障") != std::string::npos ||
              std::string(exc.what()).find("FAULT") != std::string::npos;
  }
  check(caught2, "FAULT 状态下后续命令被拒绝");
}

void test_heartbeat_not_starved() {
  std::printf("\n[8] 心跳不会被分片喂食饿死（回归测试）\n");
  auto    sim = std::make_shared<SimBoard>(kDefaultBaseId);
  ClawBus bus(sim, kDefaultBaseId);

  const uint8_t seq = bus.send_run_in(1, 7);
  bus.expect_ok(seq, 1.0, "RUN_IN");

  // 用 0.1s 分片持续 2 秒 —— 心跳周期 0.2s，应该发 ~10 次
  for (int i = 0; i < 20; ++i) bus.pump(0.1);
  bus.stop();

  check(sim->watchdog_trips() == 0, "2 秒分片 pump 期间看门狗未触发");
  check(sim->heartbeat_count() >= 8,
        sfmt("心跳发了 %d 次（期望 ≥8）", sim->heartbeat_count()));
}

void test_fault_text() {
  std::printf("\n[9] 故障位翻译\n");
  check(fault_text(0x00) == "无故障", "0x00 → 无故障");
  check(fault_text(0x01).find("电机1") != std::string::npos, "0x01 → 电机1");
  check(fault_text(0x07).find("电机3") != std::string::npos, "0x07 → 含电机3");
  check(fault_text(0x40).find("同步") != std::string::npos, "0x40 → 同步超差");
  check(fault_text(0x80).find("通信超时") != std::string::npos, "0x80 → 通信超时");
  check(fault_text(0x41).find("+") != std::string::npos, "0x41 → 组合故障用 + 连接");
}

// ---------------------------------------------------------------------------
// [10] poll(0) 必须非阻塞 —— 2026-11-16 实机回归
//   现象：标零完成后输入 30°，程序打印「→ 目标坐标 +30.00°」然后永久卡死，
//         连「发送相对位移」都没打出来。
//   原因：execute_move() 开头用 while (bus.poll(0)) 冲缓冲区，而 recv() 里的
//         SO_RCVTIMEO 被设成 {0,0}；Linux 上 {0,0} 的含义是「永不超时 = 永久
//         阻塞」而不是「立刻返回」。缓冲区一空，read() 就再也不返回。
//   修复：timeout_ms <= 0 时改用 recv(MSG_DONTWAIT)。
//   这组测试用假传输层断言「空缓冲区时 poll(0) 立刻返回 nullopt」，
//   并断言有界冲缓冲循环不会因为缓冲里有帧就无限转。
// ---------------------------------------------------------------------------
class EmptyTransport : public CanTransport {
 public:
  void send(const can_frame&) override { ++sent_; }
  bool recv(can_frame&, int timeout_ms) override {
    // 忠实复刻 SocketCan 的契约：超时 <=0 必须立刻返回 false（无帧）
    last_timeout_ = timeout_ms;
    empty_calls_++;
    return false;
  }
  int sent_         = 0;
  int empty_calls_  = 0;
  int last_timeout_ = -999;
};

void test_poll_nonblocking() {
  std::printf("\n[10] poll(0) 必须非阻塞（回归：实机卡死在发送前）\n");

  auto      transport = std::make_shared<EmptyTransport>();
  EmptyTransport* raw = transport.get();
  ClawBus   bus(transport, kDefaultBaseId);

  // 契约 1：poll(0) 在空缓冲区上返回 nullopt，且把 0 原样传给传输层
  const auto got = bus.poll(0);
  check(!got.has_value(), "poll(0) 空缓冲区 → nullopt（不阻塞）");
  check(raw->last_timeout_ == 0, "poll(0) 把 timeout_ms=0 传给了传输层");

  // 契约 2：有界冲缓冲循环（execute_move 用的写法）必须能退出
  //   空缓冲区时第 1 次就 break，不能把 64 次全跑完
  const int before = raw->empty_calls_;
  for (int guard = 0; guard < 64; ++guard) {
    if (!bus.poll(0).has_value()) break;
  }
  check(raw->empty_calls_ - before == 1, "有界冲缓冲循环：空缓冲区时第 1 次就退出");

  // 契约 3：poll(正数) 要把超时值原样传给传输层
  bus.poll(7);
  check(raw->last_timeout_ == 7, "poll(7) 把 timeout_ms=7 传给了传输层");
}

}  // namespace

int main() {
  std::printf("==============================================================================\n");
  std::printf("  testcmd_ssj1110 离线自测（模拟固件，不需要硬件）\n");
  std::printf("==============================================================================\n");

  test_crc();
  test_encoding();
  test_decode();
  test_run_in_flow(7);
  test_run_in_flow(1);
  test_position_flow();
  test_range_rejection();
  test_fault_abort();
  test_heartbeat_not_starved();
  test_fault_text();
  test_poll_nonblocking();

  std::printf("\n==============================================================================\n");
  if (g_failures == 0) {
    std::printf("  \033[32m全部通过\033[0m\n");
    std::printf("==============================================================================\n");
    return 0;
  }
  std::printf("  \033[31m%d 项失败\033[0m\n", g_failures);
  std::printf("==============================================================================\n");
  return 1;
}