// =============================================================================
//  test_positionctrl.cpp  ——  功能 2：位置控制（键盘输入角度，单路或多路）
//
//  对一块手爪驱动板上的**一个或多个**电机做相对位置控制：
//  运行时用键盘输入一个角度，选中的电机相对当前位置转动该角度。
//
//  选中的电机组合由你决定，交互里可以随时切换：
//      m1      -> 只控 M1
//      m2      -> 只控 M2
//      m12     -> M1 + M2
//      m123    -> M1 + M2 + M3（真·同时，一条 CAN 帧）
//      m13     -> M1 + M3
//  也支持 m1,3 这种写法。启动默认由 --motors 指定。
//
//  ★ 符号约定：角度为正 = 逆时针。
//    固件并不知道你的机械安装方向，所以本程序给了一个 --sign 开关：
//    如果发现 +10° 实际转成了顺时针，加 --sign -1 即可。
//
//  ★ 安全保证（本程序的核心）：
//    500 ms 内某个**仍在被驱动**的电机霍尔码没有任何**有效**变化 -> 立刻发 STOP 断电。
//    这条判定在上位机侧做，比固件自身的 CLAW_STALL_TIMEOUT_MS(800 ms)
//    堵转保护更早一步，因此不会给固件留下需要 CLEAR_FAULT 的锁存故障。
//    多路同时动时，某一路先到位固件会单独给它断电（占空比变 0），
//    该路霍尔自然不再变化 —— 这**不算**堵转，多路看门狗会自己识别。
//
//  ★★ 关于「多路同时」的重要真相（固件限制，务必知道）
//  ---------------------------------------------------------------------------
//  1. 固件的位置掩码**编码在 opcode 里**，只认四种：
//         SET_TARGET=0x07(三路) / _M1=0x01 / _M2=0x02 / _M3=0x04
//     ⇒ 「M1+M2」这种两路组合**没有单帧可用**，本程序会拆成 M1、M2
//       两条命令**顺序执行**。最终位置一样，但不是同时动。
//     ⇒ 三路全选才是真正的一条帧同时下发。
//
//  2. 多路命令的目标是**同一个绝对坐标**：
//         target = 选中各路当前位置的平均值 + 输入增量        (claw_control.c:772)
//     所以三路起点不同时，「输入 +10°」不等于「每路都转 10°」。
//     本程序会在发命令前把每路实际转角算出来打印，并提示起点偏差。
//
//  用法
//  ---------------------------------------------------------------------------
//    ./test_positionctrl --help
//    ./test_positionctrl --dry-run                # 不碰硬件，只看要发什么帧
//    ./test_positionctrl                          # 打印计划，不进交互
//    ./test_positionctrl --confirm                # 真机交互运行（默认只控 M2）
//    ./test_positionctrl --confirm --motors 123 --max-pwm 600
//    ./test_positionctrl --confirm --motors 1,2 --sign -1
//
//  运行前必须已经：
//    sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
//
//  背景（固件的相对位移限幅，务必知道）
//  ---------------------------------------------------------------------------
//  固件 App/Src/claw_control.c:ClawControl_HandleCommand() 处理相对位移时，
//  先把「当前位置 + 输入增量」换算成绝对坐标，再按下面范围**静默截断**：
//      未执行 SET_ZERO  : -30° .. +120°
//      已执行 SET_ZERO  :   0° .. +120°
//  也就是说：从 0° 连续输入 +50° 三次，第三次不会到达 +150°，而是被截在 +120°。
//  本程序会在发命令前先算出目标坐标并检查，越界则拒绝执行并重新提示，
//  避免你看到"电机没动到位"却不知道为什么。
//  ★ 位置坐标的真实含义（务必知道）
//  ---------------------------------------------------------------------------
//  位置帧里的角度**不是绝对位置**，而是固件从「上电 / 上次 SET_ZERO」以来
//  累计走过的霍尔边沿数换算的角度：
//      claw_control.c:919  HallEdgeIRQ()  position_counts += logical_step  （无界）
//      claw_control.c:324  ClawControl_Init()          position_counts = 0
//      claw_control.c:851  CLAW_COMMAND_SET_ZERO       position_counts = 0
//  发帧时它被硬钳在 int16（BuildPositionFrame 的 clamp_i16）：
//      1 count ≈ 0.09547°，累计行程 > ≈3431 counts（327.67°）后永久贴在 ±327.67°
//  读到 ±327.67° 就意味着**读数已经饱和、真实位置读不到了**。
//  更糟的是固件算相对目标用的是未饱和的 int32 position_counts，
//  所以饱和时上位机自算的「可行增量范围」是错的。
//  本程序因此会在读到饱和值时拒绝进入交互，要求先按 z 标零。
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "claw_can.h"
#include "position_ctrl_logic.h"

using namespace claw;

// #############################################################################
// #                                                                           #
// #                   ★★★   需要你修改的变量都在这里   ★★★                    #
// #                                                                           #
// #############################################################################

// ---- 1. 通信参数 -----------------------------------------------------------
constexpr const char* kChannel = "can0";            // can0..can3 对应四只手爪
constexpr uint16_t    kBaseId  = kDefaultBaseId;    // 固件默认 11

// ---- 2. 默认控制的电机组合 ------------------------------------------------
//  "1" / "2" / "3"    单路
//  "12" / "13" / "23" 两路（固件无单帧，会拆成两条顺序执行）
//  "123"             三路真·同时（一条 CAN 帧）
//  运行时还能用 m1 / m12 / m123 随时切换，这里只是启动默认值。
//  ⚠ 选中的每一路都必须是**真正接着电机**的那一路。没接电机的通道霍尔线
//     悬空，固件读到 0x07(非法)，会在启动位置命令时立刻 latch fault。
constexpr const char* kMotors = "2";

// ---- 3. 运动参数 -----------------------------------------------------------
constexpr double kSpeedRpm = 8.0;   // 名义轨迹速度，固件合法范围 4.0..17.0 rpm

// ---- 4. 逆时针符号 ---------------------------------------------------------
//  +1 : 输入正角度 = 固件正方向
//  -1 : 输入正角度 = 固件反方向（装反了就用这个）
constexpr int kSign = +1;

// ---- 5. PID 上限（可选）----------------------------------------------------
//  0   = 不碰固件参数，用板卡上电默认值。
//        ⚠ 本板 CLAW_PROFILE_GROUP=1，其 minimum == maximum == 100‰，
//          位置 PID 输出被钉死在 100‰，带机械负载时很可能根本转不动，
//          然后 500 ms 就被本程序断电。
//  >0  = 用 PARAM_SET 把该通道的 maximum(id=6) 和 startup(id=7) 一起改成
//        这个值（写 RAM，掉电恢复默认，不会写 Flash）。
//        常见可用值 600..900。
constexpr int kMaxPwmPermille = 0;

// #############################################################################
// #                       ★★★   以下一般不用改   ★★★                         #
// #############################################################################
//
//  安全窗口、行程限幅、霍尔码表、堵转看门狗等纯逻辑都放在
//  position_ctrl_logic.h 里，好让 selftest_position.cpp 能脱离硬件反复验证。
//  下面的 using 声明只是让本文件少写命名空间前缀。

using posctrl::AngleState;
using posctrl::check_saturation;
using posctrl::HallWatchdog;
using posctrl::MultiHallWatchdog;
using posctrl::kAllMotorMask;
using posctrl::kMaxTargetDeg;
using posctrl::kMinDeltaDeg;
using posctrl::kPositionSaturationDeg;
using posctrl::kSaturationWarnDeg;
using posctrl::mask_motor_count;
using posctrl::mask_text;
using posctrl::SaturationVerdict;
using posctrl::split_move_masks;
using posctrl::TargetVerdict;
using posctrl::check_target;
using posctrl::displacement_of;
using posctrl::frame_reference_deg;
using posctrl::hall_code_valid;
using posctrl::kDegPerHallCount;
using posctrl::kFirmwareStallSeconds;
using posctrl::kMaxSpeedRpm;
using posctrl::kMinSpeedRpm;
using posctrl::kMoveTimeoutCapSeconds;
using posctrl::kPreZeroMinDeg;
using posctrl::kStallWindowSeconds;
using posctrl::kZeroedMinDeg;
using posctrl::move_budget_seconds_multi;
using posctrl::parse_motor_mask;
using posctrl::will_move_of;

namespace {

// ============================ 命令行选项 ============================
struct Options {
  std::string channel  = kChannel;
  int         base     = kBaseId;
  uint8_t     motors   = 0;         // 选中的电机掩码（0x01..0x07）
  int         sign     = kSign;
  double      speed    = kSpeedRpm;
  int         max_pwm  = kMaxPwmPermille;
  bool        zeroed   = false;   // 固件已执行过 SET_ZERO 时为 true
  bool        set_zero = false;   // --set-zero：启动流程里先发一次 SET_ZERO
  bool        confirm  = false;
  bool        dry_run  = false;

  Options() { motors = parse_motor_mask(kMotors); }
};

// ============================ 打印小工具 ============================
void rule() {
  std::printf("------------------------------------------------------------------------------\n");
}

void print_options(const Options& opt) {
  rule();
  std::printf("  手爪位置控制  ——  执行计划\n");
  rule();
  std::printf("  CAN 通道        : %s (SocketCAN)\n", opt.channel.c_str());
  std::printf("  固件基地址      : %d  (命令=%d 位置=%d 状态=%d 诊断=%d)\n",
              opt.base, opt.base, opt.base + 1, opt.base + 2, opt.base + 3);
  std::printf("  控制电机        : %s  (掩码=0x%02X, 共 %d 路)\n",
              mask_text(opt.motors).c_str(), opt.motors, mask_motor_count(opt.motors));
  std::printf("  逆时针符号      : %+d  (%s)\n", opt.sign,
              opt.sign > 0 ? "输入正角度 = 固件正方向"
                           : "输入正角度 = 固件反方向");
  std::printf("  名义速度        : %.1f rpm  (%.1f °/s)\n", opt.speed, opt.speed * 6.0);
  if (opt.max_pwm > 0)
    std::printf("  PID 上限        : 改为 %d‰  (maximum + startup, 仅 RAM)\n", opt.max_pwm);
  else
    std::printf("  PID 上限        : 不改（板卡默认；group 1 为 100‰ 恒定）\n");

  // ★ 组合掩码的执行方式：一条帧 or 拆成多条顺序执行
  const auto plan = split_move_masks(opt.motors);
  if (plan.size() == 1 && plan[0] == opt.motors) {
    std::printf("  执行方式        : 一条 CAN 帧同时下发（掩码 0x%02X）\n", opt.motors);
  } else {
    std::printf("  执行方式        : ⚠ 固件无 %s 的单帧，将拆成 %zu 条**顺序**执行：",
                mask_text(opt.motors).c_str(), plan.size());
    for (size_t i = 0; i < plan.size(); ++i)
      std::printf("%s%s", i ? " → " : " ", mask_text(plan[i]).c_str());
    std::printf("\n                     （最终位置相同，但不是同时动）\n");
  }

  if (opt.set_zero)
    std::printf("  启动时标零      : 是（进交互前发一次 SET_ZERO，★ 会把当前位置定义为 0°）\n");
  else
    std::printf("  启动时标零      : 否（需要时可在交互里输入 z）\n");

  // 标零后固件的下限从 -30° 变成 0°，打印时就按实际会生效的范围显示
  const bool will_be_zeroed = opt.zeroed || opt.set_zero;
  const double lo = will_be_zeroed ? kZeroedMinDeg : kPreZeroMinDeg;
  std::printf("  允许的目标坐标  : %+.1f° .. %+.1f°  (%s)\n", lo, kMaxTargetDeg,
              will_be_zeroed ? "按已 SET_ZERO 计算" : "按未标定计算");
  rule();
  std::printf("  ★ 安全保证：%.0f ms 内该电机霍尔码无有效变化，立即发 STOP 断电。\n",
              kStallWindowSeconds * 1000.0);
  std::printf("    该判定在上位机侧，早于固件自身 800 ms 堵转保护。\n");
  rule();
}

// ============================ 参数读写（PARAM_GET / PARAM_SET）============================
//  固件 App/Src/claw_control.c:parameter_command()
//    命令帧 [3]=flags=参数 id，[4..5]=int16 数值，[6]=speed_byte=通道(1..3)
//    参数帧 [1]=seq，[2]=参数 id，[4..5]=回读值，[6]=result
//  本板 group 1 相关 id：
//    5 = minimum，6 = maximum，7 = startup，8 = startup_counts，9 = startup_ms

std::optional<ParameterFeedback> wait_parameter(ClawBus& bus, uint8_t seq, double timeout_s) {
  const double deadline = now_seconds() + timeout_s;
  while (now_seconds() < deadline) {
    if (interrupted()) throw ClawInterrupted("用户中断(Ctrl+C)");
    auto fb = bus.poll(50);
    if (!fb) continue;
    if (fb->kind == Feedback::Kind::Parameter && fb->parameter.sequence == seq)
      return fb->parameter;
  }
  return std::nullopt;
}

std::optional<int16_t> get_pid_param(ClawBus& bus, int channel, uint8_t id, double timeout_s = 1.0) {
  const uint8_t seq = bus.command(Opcode::ParamGet, id, 0, static_cast<uint8_t>(channel));
  auto fb = wait_parameter(bus, seq, timeout_s);
  if (!fb || fb->result != Result::Ok) return std::nullopt;
  return fb->value;
}

bool set_pid_param(ClawBus& bus, int channel, uint8_t id, int16_t value, double timeout_s = 1.0) {
  const uint8_t seq = bus.command(Opcode::ParamSet, id, value, static_cast<uint8_t>(channel));
  auto fb = wait_parameter(bus, seq, timeout_s);
  return fb.has_value() && fb->result == Result::Ok;
}

/// 把该通道的 PID 上限抬起来。
/// 顺序不能反：固件要求 startup <= maximum，所以先抬 maximum 再抬 startup。
void apply_max_pwm(ClawBus& bus, int motor, int pwm) {
  // 固件校验 (parameter_command 末尾)：
  //   maximum 必须在 1..1000，且 minimum <= maximum，且 startup <= maximum。
  // group 1 的 minimum 是 100，所以 pwm < 100 一定被拒。先读出来给明确提示。
  const auto min_now = get_pid_param(bus, motor, 5);
  if (min_now && pwm < *min_now)
    throw ClawError(sfmt(
        "--max-pwm %d 小于 M%d 当前的 minimum=%d‰，固件要求 minimum <= maximum。\n"
        "        请改用 >= %d 的值（例如 600）。",
        pwm, motor, *min_now, *min_now));

  std::printf("      PARAM_SET  M%d maximum(id=6) = %d‰ ... ", motor, pwm);
  std::fflush(stdout);
  if (!set_pid_param(bus, motor, 6, static_cast<int16_t>(pwm)))
    throw ClawError(sfmt("固件拒绝设置 M%d 的 maximum=%d（要求 1..1000 且 >= minimum）",
                         motor, pwm));
  std::printf("OK\n");

  std::printf("      PARAM_SET  M%d startup(id=7) = %d‰ ... ", motor, pwm);
  std::fflush(stdout);
  if (!set_pid_param(bus, motor, 7, static_cast<int16_t>(pwm)))
    throw ClawError(sfmt("固件拒绝设置 M%d 的 startup=%d（要求 1..maximum）", motor, pwm));
  std::printf("OK\n");
}

// ============================ 反馈采样 ============================
/// ★ 三路角度一次性读出来。
/// 固件的位置帧（base+1）本来就带三路 int16 角度，读一帧就有全部三路，
/// 所以多电机控制不需要发三次请求。
struct TrioAngles {
  double deg[3]       = {0.0, 0.0, 0.0};
  bool   saturated[3] = {false, false, false};

  bool any_saturated() const {
    for (int i = 0; i < 3; ++i)
      if (saturated[i]) return true;
    return false;
  }
  /// 选中各路是否全部饱和（只有全部饱和才算致命 —— 部分饱和时
  /// 可以只针对那一路提示，而不是整体拒绝）。
  bool selected_all_saturated(uint8_t mask) const {
    bool any = false;
    for (int m = 1; m <= 3; ++m) {
      if ((mask & (1u << (m - 1))) == 0) continue;
      any = true;
      if (!saturated[m - 1]) return false;
    }
    return any;
  }
};

std::optional<TrioAngles> read_trio(ClawBus& bus, double timeout_s = 1.0) {
  const double deadline = now_seconds() + timeout_s;
  while (now_seconds() < deadline) {
    auto fb = bus.poll(50);
    if (!fb) continue;
    if (fb->kind != Feedback::Kind::Position) continue;
    TrioAngles out;
    for (int i = 0; i < 3; ++i) {
      out.deg[i]       = fb->position.angles_deg[i];
      out.saturated[i] = posctrl::position_saturated(out.deg[i]);
    }
    return out;
  }
  return std::nullopt;
}

/// 读一帧诊断反馈。诊断帧 base+3 每 20 ms 一发，[1..3] 是三路霍尔码。
std::optional<DiagnosticFeedback> read_diagnostic(ClawBus& bus, double timeout_s = 1.0) {
  const double deadline = now_seconds() + timeout_s;
  while (now_seconds() < deadline) {
    auto fb = bus.poll(50);
    if (!fb) continue;
    if (fb->kind == Feedback::Kind::Diagnostic) return fb->diagnostic;
  }
  return std::nullopt;
}

//  500 ms 堵转看门狗见 position_ctrl_logic.h 的 posctrl::HallWatchdog
//  （抽出去是为了能被 selftest_position 离线验证）。

// ============================ 一次位移的结果 ============================
struct MoveOutcome {
  enum class Kind { Reached, Stalled, Timeout };
  Kind   kind      = Kind::Timeout;
  double elapsed_s = 0.0;
  double start_deg[3] = {0.0, 0.0, 0.0};
  double end_deg[3]   = {0.0, 0.0, 0.0};
  uint8_t mask      = 0;
  int    hall_edges = 0;
  int    stalled_motor = 0;   ///< 哪一路堵转（0 = 不是堵转）

  const char* kind_name() const {
    switch (kind) {
      case Kind::Reached: return "到位";
      case Kind::Stalled: return "500ms 无有效霍尔 -> 已断电";
      case Kind::Timeout: return "超时 -> 已停机";
    }
    return "?";
  }
};

// ============================ 执行一次相对位移（可多路）============================
//
//  start_deg 必须是**发命令前**读到的三路位置。固件算相对目标的基准是
//  选中各路当前位置的平均值（claw_control.c:772），下面用它复算每路实际转角。
MoveOutcome execute_move(ClawBus& bus, const Options& opt, double delta_deg,
                         const double start_deg[3], uint8_t mask) {
  MoveOutcome outcome;
  outcome.mask = mask;
  for (int i = 0; i < 3; ++i) {
    outcome.start_deg[i] = start_deg[i];
    outcome.end_deg[i]   = start_deg[i];
  }

  // ★ 先冲掉接收缓冲区里的陈旧帧。
  //   上一轮结束时固件会留下 HOLD + target_reached=1 的状态帧；如果不冲掉，
  //   新命令发出后第一帧就可能读到它，被误判成"立刻到位"。
  //   ⚠ 这里必须是有界的：万一 poll() 将来又退化成阻塞语义，
  //   无界的 while(poll(0)) 会让程序永久卡在发命令之前（2026-11-16 实机踩过）。
  for (int guard = 0; guard < 64; ++guard) {
    if (!bus.poll(0).has_value()) break;
  }

  // ---- 每路实际转角 / 是否会动 ----
  //  ★ 固件把选中各路驱动到**同一个绝对坐标** = 平均位置 + 增量。
  //    起点不一致时，各路实际转角并不等于你输入的增量。
  double disp[3] = {0.0, 0.0, 0.0};
  bool   will_move[3] = {false, false, false};
  displacement_of(start_deg, mask, delta_deg, disp);
  will_move_of(start_deg, mask, delta_deg, will_move);

  // ---- 1. 发命令并确认固件接受 ----
  const uint8_t seq = bus.send_relative_move(delta_deg, opt.speed, mask);
  std::printf("    → 发送相对位移 %+.2f° @ %.1f rpm (seq=%u, 掩码=0x%02X %s)\n",
              delta_deg, opt.speed, seq, mask, mask_text(mask).c_str());
  bus.expect_ok(seq, 1.5, "相对位移命令");

  // ---- 2. 监视循环 ----
  const double t0     = now_seconds();
  const double budget = move_budget_seconds_multi(start_deg, mask, delta_deg, opt.speed);
  double next_beat  = t0 + kHeartbeatSeconds;
  double next_print = t0;

  MultiHallWatchdog dog;
  dog.begin(mask, will_move, t0);

  bool reached = false;
  // ★ 必须先观察到 MOVING，之后才承认 HOLD / target_reached。
  //   上面虽然冲过一次缓冲区，但固件周期帧仍可能在"命令被接受"与"首次看到
  //   MOVING"之间插入一帧旧状态，多这一道闸门可以彻底避免误判。
  bool seen_moving = false;

  while (true) {
    const double now = now_seconds();
    if (interrupted()) throw ClawInterrupted("用户中断(Ctrl+C)");

    // 心跳：固件 500 ms 收不到任何命令会 latch 0x80 通信超时
    if (now >= next_beat) {
      bus.heartbeat();
      next_beat = now + kHeartbeatSeconds;
    }

    // ---- 收帧（一次读多帧，避免堆积）----
    for (int guard = 0; guard < 8; ++guard) {
      auto fb = bus.poll(guard == 0 ? 2 : 0);
      if (!fb) break;

      if (fb->kind == Feedback::Kind::Diagnostic) {
        // ★ 顺序很重要：先喂占空比，再喂霍尔。
        //   某一路刚被固件单独断电（占空比 -> 0）时，同一帧里的霍尔
        //   当然没有变化；先把 done_ 标上，这一帧就不会被当成堵转。
        for (int m = 1; m <= 3; ++m) dog.feed_duty(m, fb->diagnostic.duty_permille[m - 1], now);
        for (int m = 1; m <= 3; ++m) dog.feed_hall(m, fb->diagnostic.hall_codes[m - 1], now);
      } else if (fb->kind == Feedback::Kind::Position) {
        for (int i = 0; i < 3; ++i) outcome.end_deg[i] = fb->position.angles_deg[i];
      } else if (fb->kind == Feedback::Kind::Status || fb->kind == Feedback::Kind::Fault) {
        const StatusFeedback& s = fb->status;
        if (s.fault_mask != 0 || s.state == State::Fault)
          throw ClawError(sfmt("固件报故障 %s（state=%s）",
                               fault_text(s.fault_mask).c_str(), state_name(s.state)));
        if (s.state == State::Moving) seen_moving = true;
        // 到位后固件转 HOLD，并在状态帧 [7] 最高位置 1
        if (seen_moving && (s.target_reached || s.state == State::Hold)) {
          reached = true;
          break;
        }
      }
    }
    if (reached) break;

    // ★ 所有「本该动」的电机都已被固件单独断电 => 本帧位移实际上结束了。
    //   不加这个出口的话，如果状态帧恰好丢了，会一直空等到超时。
    if (!dog.any_watching()) {
      reached = true;
      break;
    }

    // ---- ★ 核心安全判定：某路静默 500 ms 无有效霍尔变化 -> 断电 ----
    const int stalled_motor = dog.stalled_motor(now);
    if (stalled_motor != 0) {
      std::printf("\r    M%d 已静默 %.0f ms 无有效霍尔变化 -> 立即断电\n",
                  stalled_motor, dog.silent_seconds(stalled_motor, now) * 1000.0);
      bus.stop();
      outcome.kind          = MoveOutcome::Kind::Stalled;
      outcome.stalled_motor = stalled_motor;
      outcome.elapsed_s     = now - t0;
      outcome.hall_edges    = dog.total_edges();
      return outcome;
    }

    // ---- 超时 ----
    if (now - t0 > budget) {
      std::printf("\n    %.1fs 仍未到位 -> 停机\n", now - t0);
      bus.stop();
      outcome.kind       = MoveOutcome::Kind::Timeout;
      outcome.elapsed_s  = now - t0;
      outcome.hall_edges = dog.total_edges();
      return outcome;
    }

    // ---- 进度显示（每 0.2 s 一行）----
    if (now >= next_print) {
      std::printf("\r   ");
      for (int m = 1; m <= 3; ++m) {
        if ((mask & (1u << (m - 1))) == 0) continue;
        const double done = outcome.end_deg[m - 1] - start_deg[m - 1];
        std::printf(" M%d %+7.2f°(%5.1f%%)%s", m, outcome.end_deg[m - 1],
                    std::fabs(disp[m - 1]) < 1e-9 ? 100.0 : done / disp[m - 1] * 100.0,
                    dog.done(m) ? "[已停]" : "");
      }
      std::printf("  边沿%d  ", dog.total_edges());
      std::fflush(stdout);
      next_print = now + 0.2;
    }
  }

  std::printf("\r                                                                              \r");
  outcome.kind       = MoveOutcome::Kind::Reached;
  outcome.elapsed_s  = now_seconds() - t0;
  outcome.hall_edges = dog.total_edges();
  return outcome;
}

// ==================== 多路位移的总入口（处理固件的掩码限制）====================
//
//  固件的 SET_TARGET 掩码编码在 opcode 里，只有 0x07 / 0x01 / 0x02 / 0x04 四种。
//  「M1+M2」这种两路组合没有对应单帧，只能拆成 M1、M2 两条命令**顺序**执行。
//
//  ★ 顺序执行的语义要注意：每条命令都是「选中各路平均位置 + 增量」。
//    只选一路时，平均值就是那一路自己，所以每条命令的位移就等于输入增量。
//    因此三路都走相同角度 —— 与「三路同时」的最终位置一致。
MoveOutcome execute_move_any(ClawBus& bus, const Options& opt, double delta_deg) {
  const auto masks = split_move_masks(opt.motors);

  // 顺序执行时每条命令只选一路，必须重读当前位置作为各自的基准，
  // 否则第二条命令会用第一条命令之前的旧坐标去推算转角显示。
  MoveOutcome total;
  total.mask = opt.motors;
  double elapsed = 0.0;
  int    edges   = 0;

  for (size_t i = 0; i < masks.size(); ++i) {
    if (masks.size() > 1) {
      std::printf("    [%zu/%zu] %s%s\n", i + 1, masks.size(),
                  mask_text(masks[i]).c_str(),
                  i == 0 ? "" : "（前一路已完成，继续下一路）");
    }

    auto trio = read_trio(bus, 1.0);
    if (!trio) throw ClawError("读不到位置帧，无法确定起始坐标");
    double start[3] = {trio->deg[0], trio->deg[1], trio->deg[2]};

    MoveOutcome one = execute_move(bus, opt, delta_deg, start, masks[i]);
    elapsed += one.elapsed_s;
    edges   += one.hall_edges;
    for (int m = 0; m < 3; ++m) total.end_deg[m] = one.end_deg[m];

    // 中途堵转/超时就停下，不继续后面的路 —— 避免在异常状态下继续施力
    if (one.kind != MoveOutcome::Kind::Reached) {
      total.kind          = one.kind;
      total.stalled_motor = one.stalled_motor;
      total.elapsed_s     = elapsed;
      total.hall_edges    = edges;
      return total;
    }
    // 换向静默期：固件要求 300 ms 内无同通道霍尔边沿，否则下一条命令回 BUSY
    if (i + 1 < masks.size()) bus.settle(kDirectionSettleSeconds);
  }

  total.kind        = MoveOutcome::Kind::Reached;
  total.elapsed_s   = elapsed;
  total.hall_edges  = edges;
  return total;
}

// ============================ 标零（SET_ZERO）============================
//
//  固件 claw_control.c:843  CLAW_COMMAND_SET_ZERO
//    if (claw_state == CLAW_STATE_MOVING || grip_active)
//      return record_command_result(CLAW_RESULT_BUSY);
//    for (motor = 0; motor < CLAW_MOTOR_COUNT; ++motor) {
//      motors[motor].position_counts = 0;
//      motors[motor].start_counts    = 0;
//    }
//    target_counts   = 0;
//    zero_calibrated = true;
//
//  ★ 三个关键语义：
//    1. SET_ZERO **不会让电机转动**，它只清计数。
//    2. 它一次清**三路**（循环无条件遍历），无法只标一个电机。
//    3. 它把「当前物理位置」定义为 0°，同时把允许范围从 [-30,120] 变为 [0,120]。
//    4. position_counts 与 zero_calibrated 全在 RAM，**掉电即失**。

/// 发一次 SET_ZERO 并验证真的生效了。
/// require_confirm 为 true 时要用户再敲一次 SET_ZERO 才执行。
/// 返回 true = 已标零且已刷新 angles。
bool execute_set_zero(ClawBus& bus, TrioAngles& angles, bool require_confirm) {
  rule();
  std::printf("  标零（SET_ZERO, opcode 0x04）\n");
  rule();
  std::printf("  会将**当前物理位置**定义为 0°：\n");
  std::printf("    - 三路霍尔计数全部清零（固件不支持只标单路）\n");
  std::printf("    - 允许坐标范围从 [-%.0f°, +%.0f°] 变为 [0°, +%.0f°]\n",
              -kPreZeroMinDeg, kMaxTargetDeg, kMaxTargetDeg);
  std::printf("    - 之后 + 方向可走 %.0f°，- 方向一点也不能走\n", kMaxTargetDeg);
  std::printf("    - 仅写 RAM，掉电后失效，下次上电需重新标零\n");
  std::printf("  ★ 不会让电机转动。\n\n");

  if (require_confirm) {
    std::printf("  确认要把当前位置标为 0°吗？输入 SET_ZERO 继续，其他任意键取消 > ");
    std::fflush(stdout);
    std::string answer;
    if (!std::getline(std::cin, answer)) return false;
    const auto first = answer.find_first_not_of(" \t\r\n");
    const auto last  = answer.find_last_not_of(" \t\r\n");
    const std::string text =
        (first == std::string::npos) ? "" : answer.substr(first, last - first + 1);
    if (text != "SET_ZERO") {
      std::printf("  已取消标零。\n");
      return false;
    }
  }

  // 固件在 MOVING / grip 时回 BUSY，先停稳再等静默期。
  bus.stop(0.5);
  bus.settle(kDirectionSettleSeconds);

  const uint8_t seq = bus.command(Opcode::SetZero);
  std::printf("  发送 SET_ZERO (seq=%u) ... ", seq);
  std::fflush(stdout);
  bus.expect_ok(seq, 1.5, "SET_ZERO");
  std::printf("OK\n");

  // 验证：读一帧位置帧，三路应该都回到 0° 附近。
  bus.settle(kDirectionSettleSeconds);
  auto refreshed = read_trio(bus, 1.0);
  if (!refreshed) {
    std::printf("  ⚠ 标零命令被接受，但 1s 内没读到位置帧，无法验证\n");
    return true;
  }

  for (int i = 0; i < 3; ++i) {
    if (refreshed->saturated[i]) {
      std::printf("  ✗ 标零后 M%d 位置帧仍是 %+.2f°（仍饱和）—— 固件可能并未真正清零。\n",
                  i + 1, refreshed->deg[i]);
      return false;
    }
  }

  std::printf("  标零完成，位置帧现在读 M1 %+.2f° M2 %+.2f° M3 %+.2f°（均应在 0 附近）\n",
              refreshed->deg[0], refreshed->deg[1], refreshed->deg[2]);
  angles = *refreshed;
  return true;
}

// ============================ 交互主循环 ============================
int run_interactive(ClawBus& bus, Options& opt) {
  // ---- 1. 清故障 ----
  std::printf("\n[1/5] 清故障并确认固件在线 ...\n");
  const StatusFeedback status = bus.clear_fault();
  std::printf("      OK  状态=%s 故障位=0x%02X\n",
              state_name(status.state), status.fault_mask);
  bus.settle(kDirectionSettleSeconds);

  // ---- 2. 校验所有选中通道的霍尔是否合法 ----
  //  固件启动位置命令时会检查 (motor_mask & CLAW_COMMISSIONED_MASK) == motor_mask
  //  并重新读霍尔，任一选中路非法就 latch fault。所以必须逐个查。
  std::printf("\n[2/5] 检查 %s 的霍尔信号 ...\n", mask_text(opt.motors).c_str());
  auto diag = read_diagnostic(bus);
  if (!diag)
    throw ClawError("3s 内没收到诊断帧（固件在线吗？基地址对吗？）");

  std::printf("      三路霍尔码 = {0x%02X, 0x%02X, 0x%02X}   三路占空比‰ = {%d,%d,%d}\n",
              diag->hall_codes[0], diag->hall_codes[1], diag->hall_codes[2],
              diag->duty_permille[0], diag->duty_permille[1], diag->duty_permille[2]);

  std::string bad_motors;
  for (int m = 1; m <= 3; ++m) {
    if ((opt.motors & (1u << (m - 1))) == 0) continue;
    const int code = diag->hall_codes[m - 1];
    std::printf("      M%d 霍尔码 = 0x%02X  (%s)\n", m, code,
                hall_code_valid(code) ? "合法" : "非法");
    if (!hall_code_valid(code))
      bad_motors += (bad_motors.empty() ? "" : ", ") + sfmt("M%d(0x%02X)", m, code);
  }
  if (!bad_motors.empty())
    throw ClawError(sfmt(
        "以下选中通道的霍尔码非法：%s\n"
        "      可能原因：\n"
        "        - 该通道没有接电机，霍尔线悬空（GPIO_NOPULL 会读成 0x07）\n"
        "        - 电机 / 霍尔线接线松动\n"
        "      固件会因此拒绝任何位置命令并 latch fault。\n"
        "      请先修好接线，或换一个组合（如 --motors 2）。",
        bad_motors.c_str()));

  // ---- 3. 可选：抬高 PID 上限（每一路都要设）----
  if (opt.max_pwm > 0) {
    std::printf("\n[3/5] 抬高 PID 上限到 %d‰（每路各设一次，只改 RAM，掉电恢复）\n",
                opt.max_pwm);
    for (int m = 1; m <= 3; ++m) {
      if ((opt.motors & (1u << (m - 1))) == 0) continue;
      std::printf("      M%d: ", m);
      std::fflush(stdout);
      apply_max_pwm(bus, m, opt.max_pwm);

      const auto max_now = get_pid_param(bus, m, 6);
      const auto min_now = get_pid_param(bus, m, 5);
      const auto st_now  = get_pid_param(bus, m, 7);
      if (max_now && min_now && st_now)
        std::printf("回读 minimum=%d‰  maximum=%d‰  startup=%d‰\n",
                    *min_now, *max_now, *st_now);
      else
        std::printf("⚠ 回读失败\n");
    }
  } else {
    std::printf("\n[3/5] 未指定 --max-pwm，沿用板卡默认 PID 上限\n");
    for (int m = 1; m <= 3; ++m) {
      if ((opt.motors & (1u << (m - 1))) == 0) continue;
      const auto max_now = get_pid_param(bus, m, 6);
      const auto min_now = get_pid_param(bus, m, 5);
      if (max_now && min_now) {
        std::printf("      M%d 当前：minimum=%d‰  maximum=%d‰\n", m, *min_now, *max_now);
        if (*min_now == *max_now)
          std::printf("      ⚠ M%d 的 minimum == maximum == %d‰，位置 PID 输出恒定在 %d‰。\n"
                      "        带机械负载很可能转不动，然后被 500 ms 看门狗断电。\n"
                      "        需要力矩时加 --max-pwm 600 之类的参数。\n",
                      m, *min_now, *max_now);
        else if (*max_now < 400)
          std::printf("      ⚠ M%d 的 maximum 只有 %d‰，力矩可能不足。需要时加 --max-pwm 600。\n",
                      m, *max_now);
      }
    }
  }

  // ---- 4. 标零 + 饱和闸门 ----
  //  位置帧的角度是「上电以来的累计霍尔行程」，超过 ≈327.67° 就被 int16 钳死。
  //  饱和后真实位置读不到，而固件算相对目标用的是未饱和的 int32 计数，
  //  两者不一致 => 上位机自算的「可行增量范围」会误导你。因此这里硬拦。
  std::printf("\n[4/5] 检查位置坐标是否可用 ...\n");
  auto trio = read_trio(bus, 3.0);
  if (!trio) throw ClawError("3s 内没收到位置帧（固件在线吗？）");
  std::printf("      位置帧读数 M1 %+.2f°  M2 %+.2f°  M3 %+.2f°\n",
              trio->deg[0], trio->deg[1], trio->deg[2]);

  // ★ 只关心**选中**的通道：没选中的通道饱和了也无所谓，不发命令给它。
  const bool any_sel_saturated = [&] {
    for (int m = 1; m <= 3; ++m) {
      if ((opt.motors & (1u << (m - 1))) == 0) continue;
      if (trio->saturated[m - 1]) return true;
    }
    return false;
  }();

  if (any_sel_saturated) {
    std::printf(
        "      ✗ 选中通道中有位置已饱和（|读数| >= %.1f°）—— 该读数不可信。\n"
        "\n"
        "      原因：位置帧是固件「上电以来累计霍尔行程」的换算值，\n"
        "            发帧时被硬钳在 int16（clamp_i16），上限就是 %+.2f°。\n"
        "            累计行程超过 ≈3431 个霍尔计数（≈327.67°）后永久贴顶，\n"
        "            真实位置再也读不到。\n"
        "      风险：饱和时固件用的是**未饱和的真实计数**，本程序用的是被钳死的\n"
        "            %+.2f°，两者不一致，算出来的「可行增量范围」会让实际转角\n"
        "            远超你输入的值。\n"
        "\n"
        "      必须先标零（SET_ZERO）才能继续。三条路：\n"
        "        1) 在本程序交互中输入 z，把当前位置标为 0°\n"
        "        2) 重启时加 --set-zero，启动流程里自动标一次\n"
        "        3) 若需要 0° 有明确物理意义，先手动让电机退到张开限位再标零\n",
        kSaturationWarnDeg, kPositionSaturationDeg, kPositionSaturationDeg);

    if (!opt.set_zero) {
      throw ClawError(
          "选中通道的位置读数已饱和，拒绝进入交互。\n"
          "        请加 --set-zero 重新运行，或在已知参考点标零后再来。");
    }
    std::printf("      已指定 --set-zero，开始标零 ...\n\n");
  } else {
    std::printf("      读数可信（远离 ±%.1f° 饱和区）\n", kSaturationWarnDeg);
  }

  // 需要标零的两种情形：① 位置已饱和（必须）② 用户显式要求 --set-zero
  if (opt.set_zero || any_sel_saturated) {
    // 饱和时不再问第二遍：已经明确告知风险，用户加 --set-zero 就是授权
    const bool need_confirm = !any_sel_saturated;
    if (!execute_set_zero(bus, *trio, need_confirm)) {
      throw ClawError("标零被取消或失败，未进入交互。");
    }
    opt.zeroed = true;  // 标零成功后下限变成 0°

    if (any_sel_saturated) {
      for (int m = 1; m <= 3; ++m) {
        if ((opt.motors & (1u << (m - 1))) == 0) continue;
        if (trio->saturated[m - 1])
          throw ClawError(sfmt("标零后 M%d 位置帧仍是 %+.2f°（仍饱和），"
                               "固件可能并未真正清零。\n"
                               "        请检查固件是否在 MOVING / grip 状态，或重新上电。",
                               m, trio->deg[m]));
      }
    }
  }

  // ---- 5. 交互 ----
  std::printf("\n[5/5] 进入交互。每次输入一个相对角度，执行完继续提示下一个。\n");
  std::printf("      命令：m1 / m12 / m123 等切换电机组合   z 标零   q 退出   ? 帮助\n\n");

  const char* zero_hint = opt.zeroed ? "已 SET_ZERO" : "未 SET_ZERO";
  std::printf("      %s，允许目标坐标 %+.1f° .. %+.1f°\n",
              zero_hint, opt.zeroed ? kZeroedMinDeg : kPreZeroMinDeg, kMaxTargetDeg);

  int executed = 0;

  while (true) {
    if (interrupted()) break;

    // ---- 提示行：显示选中各路当前角度 ----
    std::printf("\n[%s] 当前", mask_text(opt.motors).c_str());
    for (int m = 1; m <= 3; ++m) {
      if ((opt.motors & (1u << (m - 1))) == 0) continue;
      std::printf(" M%d %+.2f°%s", m, trio->deg[m - 1],
                  trio->saturated[m - 1] ? "[饱和]" : "");
    }
    std::printf("   输入 角度(度,逆时针为正) / m1|m12|m123 / z / q > ");
    std::fflush(stdout);

    std::string line;
    if (!std::getline(std::cin, line)) {
      if (interrupted() || std::cin.eof()) break;
      std::cin.clear();
      continue;
    }

    // 去掉首尾空白
    const auto first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) continue;
    const auto last = line.find_last_not_of(" \t\r\n");
    const std::string text = line.substr(first, last - first + 1);

    // ---- 退出 ----
    if (text == "q" || text == "Q" || text == "quit" || text == "exit") break;

    // ---- 帮助 ----
    if (text == "?" || text == "h" || text == "help") {
      std::printf(
          "      输入一个数字：相对当前位置转动的角度，逆时针为正。\n"
          "      例：+10 / 10 / -25.5 / 3.7\n"
          "      m1 / m2 / m3       : 只控单路\n"
          "      m12 / m13 / m23    : 控两路（固件无单帧，会拆成两条顺序执行）\n"
          "      m123               : 三路同时，一条 CAN 帧\n"
          "      m1,3  / m1+3       : 同上，逗号/加号也可以\n"
          "      z                  : ★ 标零 SET_ZERO，把当前位置定义为 0°\n"
          "                           （不动电机；三路计数一起清；只写 RAM）\n"
          "      q                  : 退出（退出前会发 STOP）\n"
          "\n"
          "      ★ 多路时目标坐标 = 选中各路当前位置的**平均值** + 输入增量。\n"
          "        所以各路起点不同时，每路实际转角并不相等 —— 执行前会打印出来。\n");
      continue;
    }

    // ---- 标零 ----
    //  三路计数同时清零，所以不必区分当前电机；但角度要重读。
    if (text == "z" || text == "Z" || text == "zero" ||
        text == "SET_ZERO" || text == "set_zero" || text == "标零") {
      if (execute_set_zero(bus, *trio, true)) {
        opt.zeroed = true;
        std::printf("      允许目标坐标已变为 %+.1f° .. %+.1f°\n",
                    kZeroedMinDeg, kMaxTargetDeg);
      }
      continue;
    }

    // ---- 切换电机组合 ----
    //  支持 m1 / m12 / m123 / m1,3 / M1+M3，也允许直接敲 12、123（不带 m）。
    //  判定顺序：先试「电机组合」解析，失败再当角度处理。
    //  注意 "1" / "2" / "3" 这种既像电机号又像角度 1°、2°、3° 的输入，
    //  带 m 前缀时一律当电机组合；不带前缀时当角度（遵守旧的单路习惯）。
    if (!text.empty() && (text[0] == 'm' || text[0] == 'M')) {
      const uint8_t next_mask = parse_motor_mask(text);
      if (next_mask == 0) {
        std::printf("      ⚠ 无法理解 '%s'。电机组合写法：m1 / m12 / m123 / m1,3\n",
                    text.c_str());
        continue;
      }
      if (next_mask == opt.motors) {
        std::printf("      已经是 %s\n", mask_text(opt.motors).c_str());
        continue;
      }
      // 切换前要确认新组合里每一路霍尔都合法，否则后面的命令一定会被固件拒绝
      auto probe = read_diagnostic(bus);
      if (!probe) {
        std::printf("      ⚠ 读不到诊断帧，切换取消\n");
        continue;
      }
      std::string bad;
      for (int m = 1; m <= 3; ++m) {
        if ((next_mask & (1u << (m - 1))) == 0) continue;
        const int code = probe->hall_codes[m - 1];
        if (!hall_code_valid(code))
          bad += (bad.empty() ? "" : ", ") + sfmt("M%d(0x%02X)", m, code);
      }
      if (!bad.empty()) {
        std::printf("      ⚠ 以下通道霍尔码非法（可能没接电机），切换取消：%s\n",
                    bad.c_str());
        continue;
      }
      auto fresh = read_trio(bus, 1.0);
      if (!fresh) {
        std::printf("      ⚠ 读不到位置帧，切换取消\n");
        continue;
      }
      opt.motors = next_mask;
      trio       = *fresh;
      std::printf("      → 已切换到 %s", mask_text(opt.motors).c_str());
      for (int m = 1; m <= 3; ++m) {
        if ((opt.motors & (1u << (m - 1))) == 0) continue;
        std::printf("  M%d %+.2f°%s", m, trio->deg[m - 1],
                    trio->saturated[m - 1] ? "[饱和!需按 z]" : "");
      }
      std::printf("\n");
      if (split_move_masks(opt.motors).size() > 1)
        std::printf("      ⚠ 该组合固件没有单帧，将拆成 %zu 条**顺序**执行\n",
                    split_move_masks(opt.motors).size());
      if (opt.max_pwm > 0) {
        std::printf("      → 同步为新组合的每一路设置 PID 上限 %d‰\n", opt.max_pwm);
        for (int m = 1; m <= 3; ++m) {
          if ((opt.motors & (1u << (m - 1))) == 0) continue;
          apply_max_pwm(bus, m, opt.max_pwm);
        }
      }
      continue;
    }

    // ---- 解析角度 ----
    char*  end_ptr = nullptr;
    const double raw_deg = std::strtod(text.c_str(), &end_ptr);
    if (end_ptr == text.c_str() || (end_ptr && *end_ptr != '\0')) {
      std::printf("      ⚠ 无法理解 '%s'，请输入数字或 m1/m12/m123/z/q/?\n", text.c_str());
      continue;
    }
    if (!std::isfinite(raw_deg)) {
      std::printf("      ⚠ 角度不是有限数值\n");
      continue;
    }

    // ★ 符号映射：用户输入正 = 逆时针 = opt.sign 方向
    const double delta = opt.sign * raw_deg;

    // ---- 目标坐标的基准 = 选中各路位置的**平均值**（固件就这么算）----
    double ref_deg = frame_reference_deg(trio->deg, opt.motors);
    bool   any_selected_saturated = false;
    double spread = 0.0;
    {
      double lo = 1e9, hi = -1e9;
      int n = 0;
      for (int m = 1; m <= 3; ++m) {
        if ((opt.motors & (1u << (m - 1))) == 0) continue;
        if (trio->saturated[m - 1]) any_selected_saturated = true;
        lo = std::min(lo, trio->deg[m - 1]);
        hi = std::max(hi, trio->deg[m - 1]);
        ++n;
      }
      if (n > 1) spread = hi - lo;
    }

    // ---- 增量过小 / 目标坐标越界：统一在 check_target() 里判定 ----
    // 判定逻辑在 position_ctrl_logic.h，与 selftest_position 共用同一份实现，
    // 避免"测试通过但实机用另一套阈值"的偏差。
    const auto check = check_target(ref_deg, delta, opt.zeroed, any_selected_saturated);
    if (!check.ok()) {
      const double lo = check.allowed_lo;
      switch (check.verdict) {
        case TargetVerdict::Saturated:
          std::printf(
              "      ✗ 选中通道位置坐标已饱和（量程上限 %+.2f°），\n"
              "        固件用的是未饱和的真实计数，本程序用的是被钳死的读数，\n"
              "        两者不一致会导致实际转角远超你输入的值。\n"
              "        请先输入 z 标零（SET_ZERO），再重新输入角度。\n",
              kPositionSaturationDeg);
          break;
        case TargetVerdict::TooSmall:
          std::printf("      ⚠ |%+.3f°| < %.3f°（≈5 个霍尔计数）。\n"
                      "        固件位置死区是 %d 个霍尔计数，这么小的增量会被\n"
                      "        直接判定到位、不会进入 MOVING。\n",
                      delta, kMinDeltaDeg, posctrl::kPositionDeadbandCounts);
          break;
        case TargetVerdict::BelowMin:
        case TargetVerdict::AboveMax:
          std::printf(
              "      ⚠ 目标坐标 %+.2f° 超出固件允许范围 %+.1f° .. %+.1f°\n"
              "        参考坐标 %+.2f°（%s的平均值） + 增量 %+.2f°。\n"
              "        固件会把它静默截断到 %+.1f°，本程序不予发送。\n"
              "        可行增量范围：%+.2f° .. %+.2f°%s\n",
              check.target_deg, lo, kMaxTargetDeg, ref_deg,
              mask_text(opt.motors).c_str(), delta,
              check.verdict == TargetVerdict::BelowMin ? lo : kMaxTargetDeg,
              check.min_delta, check.max_delta,
              opt.zeroed ? "" : "（若已执行 SET_ZERO，下限是 0°，可加 --zeroed）");
          break;
        case TargetVerdict::Ok:
          break;
      }
      continue;
    }

    // ---- 起点不一致时提前告知：每路实际转角会不同 ----
    if (spread > 1.0) {
      double disp[3] = {0.0, 0.0, 0.0};
      displacement_of(trio->deg, opt.motors, delta, disp);
      std::printf("      ⚠ 选中各路起点相差 %.2f°，固件会把它们驱动到**同一个**目标坐标\n"
                  "        %+.2f°（平均值 %+.2f° + %+.2f°），所以每路实际转角不同：\n",
                  spread, check.target_deg, ref_deg, delta);
      for (int m = 1; m <= 3; ++m) {
        if ((opt.motors & (1u << (m - 1))) == 0) continue;
        std::printf("          M%d  %+.2f° → %+.2f°  实际转过 %+.2f°\n", m,
                    trio->deg[m - 1], trio->deg[m - 1] + disp[m - 1], disp[m - 1]);
      }
      std::printf("        若希望每路都转过 %+.2f°，请改为逐路单独控制（m1 / m2 / m3）。\n",
                  delta);
    }

    // ---- 实际执行 ----
    std::printf("      → 目标坐标 %+.2f°（相对 %+.2f°）\n", check.target_deg, delta);

    // 单次命令被拒（BUSY / RANGE / FAULT）不应该终止整个交互会话：报错、
    // 冲掉陈旧帧、等静默期，然后继续提示下一个角度。
    MoveOutcome outcome;
    try {
      outcome = execute_move_any(bus, opt, delta);
    } catch (const ClawInterrupted&) {
      throw;
    } catch (const ClawError& exc) {
      std::printf("      ✗ %s\n", exc.what());
      if (std::string(exc.what()).find("故障") != std::string::npos) {
        std::printf("      → 已锁存故障，正在尝试 CLEAR_FAULT ...\n");
        try {
          bus.clear_fault();
          std::printf("      → 故障已清除\n");
        } catch (const ClawError& clear_exc) {
          std::printf("      ✗ 清除失败：%s\n", clear_exc.what());
          std::printf("      → 请检查接线，或按 q 退出后重新启动程序\n");
        }
      }
      bus.settle(kDirectionSettleSeconds);
      auto refreshed = read_trio(bus, 1.0);
      if (refreshed) trio = *refreshed;
      continue;
    }
    ++executed;

    std::printf("      结果：%s  用时 %.2fs  霍尔边沿 %d\n", outcome.kind_name(),
                outcome.elapsed_s, outcome.hall_edges);
    std::printf("      结束位置");
    for (int m = 1; m <= 3; ++m) {
      if ((opt.motors & (1u << (m - 1))) == 0) continue;
      std::printf("  M%d %+.2f°", m, outcome.end_deg[m - 1]);
    }
    std::printf("\n");

    if (outcome.kind == MoveOutcome::Kind::Stalled) {
      std::printf("      ⚠ M%d 被 500 ms 看门狗断电。常见原因：\n", outcome.stalled_motor);
      std::printf("        - 已经顶到机械限位（这是你期望的停止方式）\n");
      std::printf("        - PID 上限太低，力矩不足以克服负载（试 --max-pwm 600）\n");
      std::printf("        - 机械卡死或电源掉压\n");
      std::printf("      ★ 若本次是多路顺序执行，剩下的电机**没有**再动，位置可能不同步。\n");
    }

    // 换向静默期：固件要求 300 ms 内无同通道霍尔边沿，否则下一条命令回 BUSY
    bus.settle(kDirectionSettleSeconds);

    auto refreshed = read_trio(bus, 1.0);
    if (refreshed) {
      trio = *refreshed;
    } else {
      // 读不到新帧时退回运动循环里最后看到的那个位置帧读数
      for (int m = 0; m < 3; ++m) {
        trio->deg[m]       = outcome.end_deg[m];
        trio->saturated[m] = posctrl::position_saturated(outcome.end_deg[m]);
      }
    }
  }

  // ---- 退出前收尾 ----
  std::printf("\n停机 ... ");
  std::fflush(stdout);
  auto final_status = bus.stop(1.0);
  if (final_status) {
    std::printf("状态=%s 故障位=0x%02X\n",
                state_name(final_status->state), final_status->fault_mask);
    if (final_status->fault_mask) {
      std::fprintf(stderr, "      ⚠ %s\n", fault_text(final_status->fault_mask).c_str());
      return 3;
    }
  } else {
    std::printf("（没收到状态帧）\n");
  }
  std::printf("共执行 %d 次位移，已安全退出。\n", executed);
  return 0;
}

// ============================ DRY RUN ============================
int dry_run(const Options& opt) {
  std::printf("\n---------- DRY RUN：以下是将要发送的帧（不打开 CAN 总线）----------\n");
  uint8_t seq = 0;

  ++seq;
  std::printf("  1) CLEAR_FAULT      id=0x%03X data=[%s]\n",
              opt.base, to_hex(encode_command(seq, Opcode::ClearFault)).c_str());

  if (opt.max_pwm > 0) {
    // PARAM_SET: [3]=flags=参数 id, [4..5]=数值, [6]=speed_byte=通道(1..3)
    //  ★ 每路都要各发一次，因为 PID 参数是 per-channel 的。
    for (int m = 1; m <= 3; ++m) {
      if ((opt.motors & (1u << (m - 1))) == 0) continue;
      const uint8_t channel = static_cast<uint8_t>(m);
      ++seq;
      std::printf("     PARAM_SET max    id=0x%03X data=[%s]   id=6 maximum=%d‰ 通道=%u\n",
                  opt.base,
                  to_hex(encode_command(seq, Opcode::ParamSet, 6,
                                        static_cast<int16_t>(opt.max_pwm), channel))
                      .c_str(),
                  opt.max_pwm, channel);
      ++seq;
      std::printf("     PARAM_SET start  id=0x%03X data=[%s]   id=7 startup=%d‰ 通道=%u\n",
                  opt.base,
                  to_hex(encode_command(seq, Opcode::ParamSet, 7,
                                        static_cast<int16_t>(opt.max_pwm), channel))
                      .c_str(),
                  opt.max_pwm, channel);
    }
  }

  const auto masks = split_move_masks(opt.motors);
  std::printf("\n  每次你在键盘输入角度 D，将发送以下 %zu 条帧（顺序）：\n\n", masks.size());
  const double samples[] = {+10.0, -10.0, +50.0};
  for (double user_deg : samples) {
    const double delta = opt.sign * user_deg;
    for (size_t i = 0; i < masks.size(); ++i) {
      ++seq;
      std::printf("   输入 %+6.1f°  ->  relative %+6.1f°  id=0x%03X data=[%s]  掩码=0x%02X %s\n",
                  user_deg, delta, opt.base,
                  to_hex(encode_relative_move(seq, delta, opt.speed, masks[i])).c_str(),
                  masks[i], mask_text(masks[i]).c_str());
    }
    if (masks.size() > 1) std::printf("    （以上为顺序执行，每条之间会等 %.0f ms 静默期）\n",
                                      kDirectionSettleSeconds * 1000.0);
  }
  std::printf("\n  之后每 %.1fs 一次 HEARTBEAT，直到到位 / 500ms 无霍尔 / 超时。\n",
              kHeartbeatSeconds);
  std::printf("  到位或断电后发 STOP，然后重新提示输入下一个角度。\n");

  if (opt.set_zero) {
    ++seq;
    std::printf("\n  标零（--set-zero 或交互输入 z）将发送：\n\n");
    std::printf("  SET_ZERO          id=0x%03X data=[%s]  三路计数清零，不动电机\n",
                opt.base, to_hex(encode_command(seq, Opcode::SetZero)).c_str());
  }

  std::printf("---------- DRY RUN 结束，未发送任何数据 ----------\n");
  return 0;
}

// ============================ 命令行解析 ============================
void usage(const char* prog) {
  std::printf(
      "手爪位置控制（testcmd_ssj1110 功能 2）——  单路或多路\n"
      "\n"
      "对一块手爪上的**一个或多个**电机做相对位置控制。运行时用键盘输入角度，\n"
      "选中的电机相对当前位置转动该角度。输入正角度 = 逆时针。\n"
      "\n"
      "用法: %s [选项]\n"
      "\n"
      "  -h, --help              显示本帮助\n"
      "      --channel NAME      覆盖 kChannel，如 can1\n"
      "      --base ID           覆盖 kBaseId（%u..%u）\n"
      "      --motors COMB       覆盖 kMotors，启动时控制的电机组合\n"
      "                          COMB 写法：1 / 2 / 3 / 12 / 13 / 23 / 123\n"
      "                          也接受 1,2 或 1+2\n"
      "                          ★ 固件位置掩码只认三路(0x07)或单路(0x01/0x02/0x04)，\n"
      "                            所以两路组合（12/13/23）会拆成两条命令**顺序**执行，\n"
      "                            最终位置相同但不是同时动。只有 123 是真正的一帧同时下发。\n"
      "      --motor 1|2|3       已弃用，等价于 --motors N（单路）\n"
      "      --sign +1|-1        覆盖 kSign；+10° 实际转反了就改成 -1\n"
      "      --speed RPM         覆盖 kSpeedRpm（4.0..17.0）\n"
      "      --max-pwm N         用 PARAM_SET 把选中每一路的 PID maximum/startup\n"
      "                          改为 N‰（只改 RAM，掉电恢复）；0 = 不改\n"
      "      --set-zero          启动时发一次 SET_ZERO（把当前位置定义为 0°）\n"
      "                          ★ 不会转动电机；三路计数一起清；只写 RAM\n"
      "      --zeroed            固件已执行过 SET_ZERO，目标下限从 -30° 改为 0°\n"
      "                          （与 --set-zero 不同：它不发帧，只改本程序的判定）\n"
      "      --confirm           确认驱动电机（不加则只打印计划）\n"
      "      --dry-run           不打开总线，只打印要发的帧\n"
      "\n"
      "★ 多路目标的坐标语义（固件行为，必须知道）\n"
      "  多路命令的目标不是「每路各自 +D」，而是：\n"
      "      target = 选中各路当前位置的平均值 + D        (claw_control.c:772)\n"
      "  所以各路起点不同时，每路实际转角并不相等。\n"
      "  本程序会在执行前把每路实际转角算出来并打印。\n"
      "  若确实需要每路都转过 D，请逐路单独控制。\n"
      "\n"
      "安全：500 ms 内某个**仍在被驱动**的电机霍尔码无有效变化，立即发 STOP 断电。\n"
      "      多路时某一路先到位（固件会单独给它断电）不算堵转。\n"
      "\n"
      "默认只打印计划；加 --confirm 才会真正驱动电机。\n",
      prog, kBaseIdMin, kBaseIdMax);
}

double parse_double(const char* text, const char* name) {
  char*        end   = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0')
    throw ClawError(sfmt("%s 需要是数字，收到 '%s'", name, text));
  return value;
}

long parse_long(const char* text, const char* name) {
  char*      end   = nullptr;
  const long value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0')
    throw ClawError(sfmt("%s 需要是整数，收到 '%s'", name, text));
  return value;
}

Options parse_args(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) throw ClawError(sfmt("%s 缺少参数值", name));
      return argv[++i];
    };

    if (arg == "-h" || arg == "--help") {
      usage(argv[0]);
      std::exit(0);
    } else if (arg == "--confirm") {
      opt.confirm = true;
    } else if (arg == "--dry-run") {
      opt.dry_run = true;
    } else if (arg == "--zeroed") {
      opt.zeroed = true;
    } else if (arg == "--set-zero") {
      opt.set_zero = true;
    } else if (arg == "--channel") {
      opt.channel = need_value("--channel");
    } else if (arg == "--base") {
      opt.base = static_cast<int>(parse_long(need_value("--base"), "--base"));
    } else if (arg == "--motors") {
      const uint8_t mask = parse_motor_mask(need_value("--motors"));
      if (mask == 0)
        throw ClawError("--motors 需要是 1..3 的组合，如 1 / 12 / 123 / 1,3");
      opt.motors = mask;
    } else if (arg == "--motor") {
      // 已弃用的单路写法，等价于 --motors N
      const long value = parse_long(need_value("--motor"), "--motor");
      if (value < 1 || value > kMotorCount)
        throw ClawError("--motor 只能是 1、2 或 3（建议改用 --motors）");
      opt.motors = static_cast<uint8_t>(1u << (value - 1));
    } else if (arg == "--sign") {
      const long value = parse_long(need_value("--sign"), "--sign");
      if (value != 1 && value != -1) throw ClawError("--sign 只能是 +1 或 -1");
      opt.sign = static_cast<int>(value);
    } else if (arg == "--speed") {
      opt.speed = parse_double(need_value("--speed"), "--speed");
    } else if (arg == "--max-pwm") {
      opt.max_pwm = static_cast<int>(parse_long(need_value("--max-pwm"), "--max-pwm"));
    } else {
      throw ClawError(sfmt("未知选项 %s（用 --help 查看）", arg.c_str()));
    }
  }

  if (opt.base < kBaseIdMin || opt.base > kBaseIdMax)
    throw ClawError(sfmt("--base 必须在 %u..%u", kBaseIdMin, kBaseIdMax));
  if (opt.motors == 0 || (opt.motors & ~kAllMotorMask) != 0)
    throw ClawError("--motors 必须是 1..3 的非空组合（掩码 0x01..0x07）");
  if (opt.speed < kMinSpeedRpm || opt.speed > kMaxSpeedRpm)
    throw ClawError(sfmt("--speed 必须在 %.1f..%.1f rpm，当前 %.2f",
                         kMinSpeedRpm, kMaxSpeedRpm, opt.speed));
  if (opt.max_pwm < 0 || opt.max_pwm > 1000)
    throw ClawError("--max-pwm 必须在 0..1000");
  return opt;
}

}  // namespace

int main(int argc, char** argv) {
  install_interrupt_handler();

  Options opt;
  try {
    opt = parse_args(argc, argv);
  } catch (const ClawError& exc) {
    std::fprintf(stderr, "参数错误：%s\n", exc.what());
    std::fprintf(stderr, "用 --help 查看用法。\n");
    return 1;
  }

  print_options(opt);

  if (opt.dry_run) return dry_run(opt);

  if (!opt.confirm) {
    std::printf("\n当前为【计划模式】，未发送任何数据。\n");
    std::printf("确认无误后加 --confirm 真正运行，例如：\n");
    std::printf("  %s --confirm --motors %s --sign %+d", argv[0],
                mask_text(opt.motors).c_str(), opt.sign);
    if (opt.max_pwm > 0) std::printf(" --max-pwm %d", opt.max_pwm);
    if (opt.set_zero) std::printf(" --set-zero");
    if (opt.zeroed) std::printf(" --zeroed");
    std::printf("\n");
    return 0;
  }

  try {
    auto transport = make_transport(PortKind::SocketCan, opt.channel.c_str());
    ClawBus bus(transport, static_cast<uint16_t>(opt.base));
    return run_interactive(bus, opt);
  } catch (const ClawInterrupted& exc) {
    std::fprintf(stderr, "\n用户中断（Ctrl+C），已发送 STOP。\n");
    return 130;
  } catch (const ClawError& exc) {
    std::fprintf(stderr, "\n运行中止：%s\n", exc.what());
    return 2;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "\n运行中止（未知错误）：%s\n", exc.what());
    return 2;
  }
}