// =============================================================================
//  position_ctrl_logic.h  ——  test_positionctrl 的纯逻辑部分（无 I/O，可离线测试）
//
//  抽出来的原因：500 ms 堵转看门狗是本程序唯一的安全保证，必须能脱离硬件
//  被反复验证。这里只放纯函数/纯状态机，不含任何 socket、printf、时钟调用
//  （时间由调用方以 double 秒的形式传入）。
//
//  被 test_positionctrl.cpp 与 selftest_position.cpp 同时使用。
// =============================================================================

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace posctrl {

// ============================== 固件常量（对应 claws_control_1）==============================
// App/Inc/claw_config.h
inline constexpr double kStallWindowSeconds = 0.5;    // ★ 本程序的核心安全窗口（上位机侧）
inline constexpr double kFirmwareStallSeconds = 0.8;  // CLAW_STALL_TIMEOUT_MS，我们总是更早
// 换向静默期沿用 claw::kDirectionSettleSeconds（claw_can.h 已定义），此处不重复。

// 固件行程限幅（0.001° 单位换算成度）
inline constexpr double kZeroedMinDeg  = 0.0;    // CLAW_MIN_TARGET_MDEG
inline constexpr double kPreZeroMinDeg = -30.0;  // CLAW_PREZERO_MIN_TARGET_MDEG
inline constexpr double kMaxTargetDeg  = 120.0;  // CLAW_MAX_TARGET_MDEG

// CLAW_POSITION_DEADBAND_COUNTS = 1，1 count = 360/(6*628.5) ≈ 0.09547°
inline constexpr double kDegPerHallCount = 360.0 / (6.0 * 628.5);
inline constexpr int    kPositionDeadbandCounts = 1;

// 增量下限：小于这个值固件会直接判到位、不进 MOVING，上位机只能空等超时。
// 取 5 个 Hall count（≈0.477°），留一点余量。
inline constexpr double kMinDeltaDeg = kDegPerHallCount * 5.0;

// 名义速度合法范围（CLAW_MIN_SPEED_DRPM / CLAW_MAX_SPEED_DRPM）
inline constexpr double kMinSpeedRpm = 4.0;
inline constexpr double kMaxSpeedRpm = 17.0;

// 固件 CLAW_MOVE_TIMEOUT_MS = 15000，留 1 s 余量
inline constexpr double kMoveTimeoutCapSeconds = 14.0;

// ============================== ★ 位置帧饱和判定 ==============================
//
//  固件的位置帧**不是绝对位置**，而是「上电以来（或上次 SET_ZERO 以来）累计
//  走过的霍尔边沿数」换算出来的角度：
//
//    claw_control.c:919  HallEdgeIRQ()
//        motors[motor].position_counts += logical_step;   // 无界累加，无任何限幅
//    claw_control.c:324  ClawControl_Init()   -> position_counts = 0（上电清零）
//    claw_control.c:851  CLAW_COMMAND_SET_ZERO -> position_counts = 0（唯一清零点）
//
//  而发帧时被硬钳在 int16：
//
//    claw_control.c:1244  ClawControl_BuildPositionFrame()
//        int16_t angle_cdeg = clamp_i16(CountsToAngleMilliDeg(...) / 10L);
//
//  1 count ≈ 0.09547°，所以累计行程一旦超过 ≈3431 counts（327.67°），
//  位置帧就永久贴在 ±327.67°，**真实位置再也读不到**。
//
//  为什么必须特殊处理：固件计算相对目标时用的是**未饱和的 int32 position_counts**
//  （claw_control.c:773），而上位机只能看到饱和的 327.67。两者在饱和后不一致，
//  上位机算出来的「可行增量范围」会严重偏离实际转角，照它输入会出事故。

/// 固件 int16 cdeg 的上饱和值：INT16_MAX / 100 = 327.67°
inline constexpr double kPositionSaturationDeg = 327.67;

/// 判定阈值。比理论饱和值留 5° 余量：
/// 真实位置落在 ±(322.67 .. 327.67) 时位置帧同样可能被钳在天花板上，
/// 与其猜，不如把这一整段都视为「读数不可信」。
inline constexpr double kSaturationWarnDeg = 320.0;

/// 位置帧读数是否已经饱和（含义：这个数字不可信，必须先 SET_ZERO）。
inline bool position_saturated(double reported_deg) {
  return std::fabs(reported_deg) >= kSaturationWarnDeg;
}

enum class SaturationVerdict {
  Valid,        ///< 读数可信，可以正常做相对位移
  Saturated,    ///< 已饱和，读数不可信，必须先 SET_ZERO
};

struct SaturationCheck {
  SaturationVerdict verdict       = SaturationVerdict::Valid;
  double            reported_deg  = 0.0;   ///< 位置帧原始读数
  double            threshold_deg = kSaturationWarnDeg;

  bool ok() const { return verdict == SaturationVerdict::Valid; }
};

/// 检查一帧位置读数是否可以信任。
inline SaturationCheck check_saturation(double reported_deg) {
  SaturationCheck out;
  out.reported_deg = reported_deg;
  out.verdict      = position_saturated(reported_deg) ? SaturationVerdict::Saturated
                                                      : SaturationVerdict::Valid;
  return out;
}

/// 把「角度 + 该角度是否饱和」绑在一起。
/// 单独一个 double 很容易忘记饱和这回事——之前就是因为把饱和的 327.67°
/// 当成真位置，算出了一个会导致实际转角翻倍的「可行增量范围」。
struct AngleState {
  double deg       = 0.0;    ///< 位置帧原始读数（可能就是饱和值）
  bool   saturated = false;  ///< true = deg 不可信，必须先 SET_ZERO

  AngleState() = default;
  explicit AngleState(double reported_deg) { update(reported_deg); }

  /// 用一帧新的位置读数刷新。
  void update(double reported_deg) {
    deg       = reported_deg;
    saturated = position_saturated(reported_deg);
  }
};

// ============================== 霍尔码 ==============================
/// 固件 hall_sequence[6] = {0x1, 0x5, 0x4, 0x6, 0x2, 0x3}。
/// 0x0 和 0x7 非法 —— 通常意味着该通道霍尔线悬空或没接电机。
inline bool hall_code_valid(int code) {
  switch (code) {
    case 0x1: case 0x5: case 0x4:
    case 0x6: case 0x2: case 0x3: return true;
    default: return false;
  }
}

inline uint8_t motor_mask_of(int motor) { return static_cast<uint8_t>(1u << (motor - 1)); }

// ============================== 电机组合（M1/M2/M3 任意子集）==============================
//
//  ★ 固件的位置命令掩码**不是独立字段**，而是编码在 opcode 里：
//      claw_control.c:139  command_motor_mask()
//          SET_TARGET    -> 0x07  (三路同时)
//          SET_TARGET_M1 -> 0x01
//          SET_TARGET_M2 -> 0x02
//          SET_TARGET_M3 -> 0x04
//    所以固件只认这四种掩码。M1+M2 这种两路组合**没有对应的单帧**，
//    只能拆成两条单路命令顺序执行（见 split_move_masks）。

inline constexpr uint8_t kAllMotorMask = 0x07U;

inline int mask_motor_count(uint8_t mask) {
  int n = 0;
  for (int m = 1; m <= 3; ++m)
    if (mask & (1u << (m - 1))) ++n;
  return n;
}

/// 解析 "1" / "12" / "123" / "1,2" / "1+2" / "m12" / "M1 M3" 之类的组合串。
/// 返回 0 表示解析失败（或没选中任何电机）。
///
/// 'm' / 'M' 只允许出现在「开头」或「分隔符之后」作为前缀，
/// 这样 "M1 M3" 合法，而 "1m2" 这种嵌在中间的不合法。
inline uint8_t parse_motor_mask(const std::string& input) {
  uint8_t mask      = 0;
  bool    any       = false;
  bool    at_prefix = true;   // 当前位置是否可以放 m/M 前缀
  for (char ch : input) {
    if (ch >= '1' && ch <= '3') {
      mask       = static_cast<uint8_t>(mask | (1u << (ch - '1')));
      any        = true;
      at_prefix  = false;
    } else if (ch == ',' || ch == '+' || ch == '|' || ch == ' ' || ch == '\t') {
      at_prefix = true;         // 分隔符之后又可以接 m/M
    } else if ((ch == 'm' || ch == 'M') && at_prefix) {
      at_prefix = false;        // 前缀只能有一个，不能 "mm12"
    } else {
      return 0;                 // 含非法字符（含 '0'）
    }
  }
  return any ? mask : static_cast<uint8_t>(0);
}

/// 掩码 -> "M1+M2+M3" 这种人话。
inline std::string mask_text(uint8_t mask) {
  std::string out;
  for (int m = 1; m <= 3; ++m) {
    if ((mask & (1u << (m - 1))) == 0) continue;
    if (!out.empty()) out += '+';
    out += 'M';
    out += static_cast<char>('0' + m);
  }
  return out.empty() ? std::string("(无)") : out;
}

/// 固件是否能用**一条帧**完成这个组合。
inline bool firmware_supported_mask(uint8_t mask) {
  return mask == kAllMotorMask || mask == 0x01U || mask == 0x02U || mask == 0x04U;
}

/// 把选中组合拆成若干条固件能接受的帧掩码，按 M1 -> M2 -> M3 顺序。
///   0x07 -> {0x07}（真·三路同时）
///   0x03 -> {0x01, 0x02}（M1+M2 只能顺序执行）
///   0x06 -> {0x02, 0x04}
///   0x05 -> {0x01, 0x04}
inline std::vector<uint8_t> split_move_masks(uint8_t mask) {
  if (firmware_supported_mask(mask)) return {mask};
  std::vector<uint8_t> out;
  for (int m = 1; m <= 3; ++m) {
    if (mask & (1u << (m - 1))) out.push_back(static_cast<uint8_t>(1u << (m - 1)));
  }
  return out;
}

// ============================== ★ 多路命令的目标坐标 ==============================
//
//  一条多路位置命令里，固件把**所有选中电机驱动到同一个绝对坐标**：
//
//      claw_control.c:772-781  (相对位移分支)
//          for (motor ...) if (选中) position_sum += position_counts[motor];
//          target_mdeg += CountsToAngleMilliDeg(position_sum / selected_count);
//
//  也就是说   目标坐标 = 选中各路当前位置的**平均值** + 输入增量
//
//  ★ 直接后果：三路起点不同的时，"输入 +10°" 并不等于"每路都转 10°"。
//    例：M1=10°、M2=20°、M3=30°，平均 20°，输入 +10° -> 目标 30°
//        M1 实际转 +20°，M2 转 +10°，M3 转 0°。
//    只有三路起点一致时，"每路转相同角度"才成立。调用方必须把这件事
//    算出来告诉用户，否则用户看到的动作完全不可预期。

/// 一条多路命令的参考坐标（选中各路位置的平均值）。
inline double frame_reference_deg(const double pos_deg[3], uint8_t mask) {
  double sum = 0.0;
  int    n   = 0;
  for (int m = 1; m <= 3; ++m) {
    if ((mask & (1u << (m - 1))) == 0) continue;
    sum += pos_deg[m - 1];
    ++n;
  }
  return (n == 0) ? 0.0 : sum / static_cast<double>(n);
}

/// 每路实际会走多少度（正负号含方向）。
inline void displacement_of(const double pos_deg[3], uint8_t mask, double delta,
                            double out[3]) {
  const double target = frame_reference_deg(pos_deg, mask) + delta;
  for (int m = 1; m <= 3; ++m) {
    const bool sel = (mask & (1u << (m - 1))) != 0U;
    out[m - 1]     = sel ? (target - pos_deg[m - 1]) : 0.0;
  }
}

/// 每路预期是否真的会产生霍尔变化。
/// 路径长度落在固件位置死区（CLAW_POSITION_DEADBAND_COUNTS = 1）以内时，
/// 固件在 start_move() 里就直接把该路标成 motor_done，从头到尾不给 PWM。
/// 这类电机必须事先排除在多路堵转看门狗之外，否则 500 ms 后会被误判堵转。
inline void will_move_of(const double pos_deg[3], uint8_t mask, double delta, bool out[3]) {
  double disp[3] = {0.0, 0.0, 0.0};
  displacement_of(pos_deg, mask, delta, disp);
  const double min_step = kDegPerHallCount * static_cast<double>(kPositionDeadbandCounts);
  for (int m = 1; m <= 3; ++m) {
    const bool sel = (mask & (1u << (m - 1))) != 0U;
    out[m - 1]     = sel && std::fabs(disp[m - 1]) > min_step;
  }
}

/// 该路要走的霍尔计数（绝对值），用于超时预算。
inline double max_abs_displacement(const double pos_deg[3], uint8_t mask, double delta) {
  double disp[3] = {0.0, 0.0, 0.0};
  displacement_of(pos_deg, mask, delta, disp);
  double best = 0.0;
  for (int m = 0; m < 3; ++m) best = std::max(best, std::fabs(disp[m]));
  return best;
}

inline double allowed_min_deg(bool zeroed) { return zeroed ? kZeroedMinDeg : kPreZeroMinDeg; }

// ============================== 目标坐标判定 ==============================
enum class TargetVerdict {
  Ok,          ///< 可以发送
  BelowMin,    ///< 低于允许下限（固件会截断到下限，静默少走）
  AboveMax,    ///< 高于允许上限（固件会截断到上限）
  TooSmall,    ///< 增量太小，固件会直接判到位、不会真的动
  Saturated,   ///< ★ 当前位置读数已饱和，算出来的目标坐标不可信
};

struct TargetCheck {
  TargetVerdict verdict      = TargetVerdict::Ok;
  double        target_deg   = 0.0;  ///< 当前位置 + 增量
  double        delta_deg    = 0.0;  ///< 实际要发的相对增量（符号已应用）
  double        allowed_lo   = 0.0;  ///< 本次允许的最小坐标
  double        allowed_hi   = 0.0;  ///< 本次允许的最大坐标
  double        min_delta    = 0.0;  ///< 从当前坐标出发允许的最小增量
  double        max_delta    = 0.0;  ///< 从当前坐标出发允许的最大增量

  bool ok() const { return verdict == TargetVerdict::Ok; }

  const char* verdict_text() const {
    switch (verdict) {
      case TargetVerdict::Ok:        return "可以发送";
      case TargetVerdict::BelowMin:  return "低于固件允许下限，会被截断";
      case TargetVerdict::AboveMax:  return "高于固件允许上限，会被截断";
      case TargetVerdict::TooSmall:  return "增量小于位置死区，不会产生动作";
      case TargetVerdict::Saturated: return "位置读数已饱和，目标坐标不可信";
    }
    return "?";
  }
};

/// 判断「从 current_deg 出发、相对移动 delta_deg（已乘符号）」能不能发。
///
/// saturated 为 true 时直接返回 Saturated 而不做后续计算：此时 current_deg
/// 只是个被钳死的常数，算出来的 min_delta / max_delta 与实际转角差得远。
inline TargetCheck check_target(double current_deg,
                                double delta_deg,
                                bool   zeroed,
                                bool   saturated = false) {
  TargetCheck out;
  out.delta_deg  = delta_deg;
  out.target_deg = current_deg + delta_deg;
  out.allowed_lo = allowed_min_deg(zeroed);
  out.allowed_hi = kMaxTargetDeg;
  out.min_delta  = out.allowed_lo - current_deg;
  out.max_delta  = out.allowed_hi - current_deg;

  // ★ 饱和优先：读数本身就不可信，后面任何判定都是错的
  if (saturated || position_saturated(current_deg)) {
    out.verdict = TargetVerdict::Saturated;
    return out;
  }

  // 再查死区：即使目标坐标合法，太小也不会动
  if (std::fabs(delta_deg) < kMinDeltaDeg) {
    out.verdict = TargetVerdict::TooSmall;
    return out;
  }
  // 用 epsilon 容忍浮点误差（例如 120.0 算成 120.0000000001）
  constexpr double kEps = 1e-6;
  if (out.target_deg < out.allowed_lo - kEps) {
    out.verdict = TargetVerdict::BelowMin;
    return out;
  }
  if (out.target_deg > out.allowed_hi + kEps) {
    out.verdict = TargetVerdict::AboveMax;
    return out;
  }
  out.verdict = TargetVerdict::Ok;
  return out;
}

// ============================== 超时预算 ==============================
/// 按名义速度估算这条命令最多该等多久，再额外给 10 s 通信/加速余量，
/// 最后夹在固件自身 15 s 动作超时之前。
inline double move_budget_seconds(double delta_deg, double speed_rpm) {
  const double nominal = std::fabs(delta_deg) / (speed_rpm * 6.0);
  const double budget  = nominal + 10.0;
  return budget < kMoveTimeoutCapSeconds ? budget : kMoveTimeoutCapSeconds;
}

/// 多电机版本的预算：按选中各路里**走得最多**的那一路估算。
/// 固件按 longest_path 计算 trajectory_ms，所以走得最远的那一路决定总时长。
inline double move_budget_seconds_multi(const double pos_deg[3], uint8_t mask,
                                        double delta_deg, double speed_rpm) {
  const double travel = max_abs_displacement(pos_deg, mask, delta_deg);
  return move_budget_seconds(travel > 0.0 ? travel : std::fabs(delta_deg), speed_rpm);
}

// ============================== ★ 500 ms 堵转看门狗 ==============================
/// 本程序的核心安全组件。
///
/// "有效霍尔变化" 的定义刻意与固件一致：
///   霍尔码发生变化，且**新码**在固件合法扇区表 {1,5,4,6,2,3} 内。
///   非法码跳变（例如 0x07 -> 0x00）不算进展。
///
/// 从最后一次有效变化起超过 kStallWindowSeconds 即判定堵转，调用方应立刻发 STOP。
///
/// 注意：这里不接受"位置角度变化"作为进展依据。固件的位置帧是每 20 ms 的
/// 绝对值快照，而堵转的本质是霍尔不再翻转；用霍尔码判定才与固件一致，
/// 也才能在霍尔线接触不良时及时发现。
class HallWatchdog {
 public:
  void begin(int motor, double now) {
    motor_         = motor;
    last_change_s_ = now;
    previous_code_ = -1;
    valid_edges_   = 0;
    first_seen_    = false;
  }

  /// 喂一帧诊断反馈。返回 true 表示这一帧看到了有效霍尔变化。
  bool feed(int hall_code, double now) {
    bool changed = false;
    if (first_seen_ && hall_code != previous_code_ && hall_code_valid(hall_code)) {
      last_change_s_ = now;
      changed        = true;
      ++valid_edges_;
    }
    // 首次看到的码只作为基准，不算"变化"（否则会把起始状态误算成进展）
    first_seen_    = true;
    previous_code_ = hall_code;
    return changed;
  }

  bool   stalled(double now) const { return now - last_change_s_ >= kStallWindowSeconds; }
  double silent_seconds(double now) const { return now - last_change_s_; }
  int    valid_edges() const { return valid_edges_; }
  int    last_code() const { return previous_code_; }
  bool   first_seen() const { return first_seen_; }

 private:
  int    motor_         = 1;
  int    previous_code_ = -1;
  double last_change_s_ = 0.0;
  int    valid_edges_   = 0;
  bool   first_seen_    = false;
};

// ============================== ★ 多路堵转看门狗 ==============================
//
//  单路版本（HallWatchdog）在多电机场景下**会产生误判**，必须换掉。
//
//  三路同时位移时，如果某一路先到位，固件会在 ClawControl_Task 里
//  对它**单独**断电：
//
//      claw_control.c:1154   if (motor_done[motor]) { set_pwm(motor, 0U); ... }
//
//  这一路的霍尔从此不再翻转。若上位机仍把它当"正在动"，500 ms 后就会
//  判定堵转并发出 STOP —— 把另外两路还在正常运转的电机一起掐死。
//
//  判据：诊断帧（base+3）里带了**每一路**的占空比：
//      claw_can.cpp  d.duty_permille[m] = data[4+m] * 4
//  某一路曾经被驱动过、现在占空比变 0，就说明固件已经把它单独停了，
//  此时该路霍尔不再变化是**正常**的，不能算堵转。
//
//  第二类误判：某一路的增量落在固件位置死区里（path_counts <= 1），
//  它从头到尾都不会动。这类电机由调用方通过 will_move[] 事先排除。
class MultiHallWatchdog {
 public:
  /// mask       : 本次真正下发的那条命令的电机掩码
  /// will_move  : 3 元素数组，第 i 路预期是否真的会转（由上位机按目标坐标预算）
  void begin(uint8_t mask, const bool will_move[3], double now) {
    mask_ = mask;
    for (int i = 0; i < 3; ++i) {
      selected_[i]    = ((mask >> i) & 1U) != 0U;
      will_move_[i]   = selected_[i] && will_move[i];
      prev_code_[i]   = -1;
      first_seen_[i]  = false;
      last_change_[i] = now;
      edges_[i]       = 0;
      driven_[i]      = false;
      done_[i]        = false;
    }
  }

  bool selected(int motor) const { return selected_[motor - 1]; }
  bool done(int motor) const { return done_[motor - 1]; }
  int  edges(int motor) const { return edges_[motor - 1]; }

  /// 这一路此刻是否处在「应该产生霍尔变化」的监视范围内。
  bool watching(int motor) const {
    const int i = motor - 1;
    if (!will_move_[i]) return false;
    if (done_[i]) return false;
    return true;
  }

  /// 还有没有任何一路仍在监视范围内。
  bool any_watching() const {
    for (int i = 0; i < 3; ++i)
      if (watching(i + 1)) return true;
    return false;
  }

  /// 喂一帧霍尔码。返回 true 表示这一帧看到了有效变化。
  bool feed_hall(int motor, int hall_code, double now) {
    const int i = motor - 1;
    if (i < 0 || i >= 3) return false;
    if (!watching(motor)) {
      first_seen_[i] = true;
      prev_code_[i]  = hall_code;
      return false;
    }
    bool changed = false;
    if (first_seen_[i] && hall_code != prev_code_[i] && hall_code_valid(hall_code)) {
      last_change_[i] = now;
      changed         = true;
      ++edges_[i];
    }
    first_seen_[i] = true;
    prev_code_[i]  = hall_code;
    return changed;
  }

  /// 喂一帧占空比（诊断帧里的每路值，‰）。
  /// ★ 必须在 feed_hall() 之前调用，这样"占空比刚归零"的那一帧
  ///   就不会再被当成堵转。
  void feed_duty(int motor, int duty_permille, double now) {
    const int i = motor - 1;
    if (i < 0 || i >= 3) return;
    if (!selected_[i] || !will_move_[i]) return;
    if (duty_permille > 0) {
      driven_[i] = true;
      if (done_[i]) {
        // 又被驱动了，重新开始计时，否则会拿旧的 last_change 立刻误判堵转
        done_[i]        = false;
        last_change_[i] = now;
      }
    } else if (driven_[i]) {
      // 曾经被驱动、现在占空比为 0 => 固件已判定这一路到位并单独断电
      done_[i] = true;
    }
  }

  /// 仍在监视的某一路静默超时 -> 返回该路编号（1..3），0 = 没有任何一路堵转。
  int stalled_motor(double now) const {
    for (int i = 0; i < 3; ++i) {
      if (!watching(i + 1)) continue;
      if (now - last_change_[i] >= kStallWindowSeconds) return i + 1;
    }
    return 0;
  }

  bool   stalled(double now) const { return stalled_motor(now) != 0; }
  double silent_seconds(int motor, double now) const { return now - last_change_[motor - 1]; }

  int total_edges() const {
    int sum = 0;
    for (int i = 0; i < 3; ++i)
      if (will_move_[i]) sum += edges_[i];
    return sum;
  }

  int done_count() const {
    int n = 0;
    for (int i = 0; i < 3; ++i)
      if (will_move_[i] && done_[i]) ++n;
    return n;
  }

 private:
  uint8_t mask_        = 0;
  bool    selected_[3] = {false, false, false};
  bool    will_move_[3] = {false, false, false};
  bool    driven_[3]    = {false, false, false};
  bool    done_[3]      = {false, false, false};
  bool    first_seen_[3] = {false, false, false};
  int     prev_code_[3]  = {-1, -1, -1};
  double  last_change_[3] = {0.0, 0.0, 0.0};
  int     edges_[3]       = {0, 0, 0};
};

}  // namespace posctrl