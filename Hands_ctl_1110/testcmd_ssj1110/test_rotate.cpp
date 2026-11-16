// =============================================================================
//  test_rotate.cpp  ——  功能 1：三指同速往复旋转
//
//  一组手爪（3 个电机）同时以某速度持续旋转一段时间，
//  再以同样速度反向旋转同样时间，如此往复若干轮。
//
//  用法
//  ---------------------------------------------------------------------------
//    ./test_rotate --help                    # 查看全部参数
//    ./test_rotate --dry-run                 # 不碰硬件，只打印将要发的帧
//    ./test_rotate                           # 打印计划，不发帧
//    ./test_rotate --confirm                 # 真机运行
//    ./test_rotate --confirm --seconds 3 --cycles 2
//
//  运行前必须已经：
//    sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "claw_can.h"

using namespace claw;

// #############################################################################
// #                                                                           #
// #                   ★★★   需要你修改的变量都在这里   ★★★                    #
// #                                                                           #
// #############################################################################

// ---- 0. 模式选择 -----------------------------------------------------------
//  kModeRunIn    : 用固件 RUN_IN 命令 → 真正无限制连续旋转，占空比由固件写死
//                  1000‰，所以 kRotateSpeedRpm **不生效**，只有 kRotateSeconds 生效。
//                  这是最接近"持续旋转一段时间"的实现。
//  kModePosition : 用相对位移命令 → kRotateSpeedRpm 生效，但行程被固件限制在
//                  0..120°，所以只能"转过去再转回来"（摆幅 = 速度 × 时间，
//                  超 120° 会被截断）。
enum Mode { kModeRunIn, kModePosition };
constexpr Mode kMode = kModeRunIn;

// ---- 1. 核心参数（你主要改这两行）------------------------------------------
constexpr double kRotateSeconds  = 30.0;   // ★ 每个方向持续多少秒
constexpr double kRotateSpeedRpm = 0.5;   // ★ 旋转速度(rpm)。仅 position 模式生效，合法 4.0..17.0

// ---- 2. 往复次数与方向 -----------------------------------------------------
constexpr int kCycles     = 100;    // 正转→反转 算 1 轮，共做几轮
constexpr int kDirForward = 1;    // 第一段的原始 DIR 电平，0 或 1；第二段自动取反

// ---- 3. 通信参数 -----------------------------------------------------------
constexpr const char* kChannel = "can0";  // 一块驱动板 = 一条总线。can0..can3 对应四个手爪
constexpr uint16_t    kBaseId  = kDefaultBaseId;   // 固件默认 11；改过 ID 的手爪填实际基地址
constexpr uint8_t     kMotorMask = 0x2;            // 7 = 三个电机全动；1/2/4 只动单个
//  当前只接了 M2 一个电机，所以用 2。注意：mask 必须只包含**真正接着电机**的通道，
//  否则那些悬空通道的霍尔读到 0x07（非法），固件会立刻 latch fault 并拒绝启动。
//  想用创芯分析仪(cxcanfd)之类非 SocketCAN 通道时，以后在 claw_can.h 的
//  PortKind 里加枚举，再在 make_transport() 里加分支即可。
constexpr PortKind kPort = PortKind::SocketCan;

// ---- 4. 安全限制 -----------------------------------------------------------
constexpr double kMaxTotalSeconds = 30000.0;  // 整个程序累计运动时间的硬上限，防止误改参数跑飞

// #############################################################################
// #                       ★★★   以下一般不用改   ★★★                         #
// #############################################################################

constexpr int    kSettleMs             = 350;  // 换向静默期，固件要求 ≥300ms
constexpr double kSegmentTimeoutMargin = 3.0;  // position 模式等到位时额外给的余量(秒)
constexpr double kMaxTravelDeg         = 120.0;  // 固件 CLAW_MAX_TARGET_MDEG
constexpr double kMinSpeedRpm          = 4.0;
constexpr double kMaxSpeedRpm          = 17.0;

namespace {

// ============================ 执行计划 ============================
struct Plan {
  Mode     mode          = kMode;
  double   seconds       = kRotateSeconds;
  double   speed_rpm     = kRotateSpeedRpm;
  int      cycles        = kCycles;
  int      dir_forward   = kDirForward;
  int      dir_reverse   = 1 - kDirForward;
  uint8_t  motor_mask    = kMotorMask;
  uint16_t base_id       = kBaseId;
  std::string channel    = kChannel;

  // position 模式才有意义
  double angle_deg           = 0.0;  // 速度 × 时间 算出来的名义摆幅
  double effective_angle_deg = 0.0;  // 被 120° 截断后的真实摆幅
  double effective_seconds   = 0.0;  // 真实每段耗时
  bool   angle_clipped       = false;

  double total_motion_seconds() const { return seconds * 2.0 * cycles; }
};

/// 校验并补全派生字段。代码里的常量值和命令行覆盖值都走这里，保证规则只有一份。
void validate_and_fill(Plan& plan) {
  if (plan.seconds <= 0.0) throw ClawError("每段时长必须大于 0");
  if (plan.cycles <= 0) throw ClawError("往复轮数必须大于 0");
  if (plan.dir_forward != 0 && plan.dir_forward != 1)
    throw ClawError("正向 DIR 只能是 0 或 1");
  if (plan.motor_mask == 0 || plan.motor_mask > kAllMotorsMask)
    throw ClawError("电机掩码只能在 1..7");
  if (plan.base_id < kBaseIdMin || plan.base_id > kBaseIdMax)
    throw ClawError(sfmt("CAN 基地址必须在 %u..%u", kBaseIdMin, kBaseIdMax));

  plan.dir_reverse = 1 - plan.dir_forward;

  if (plan.total_motion_seconds() > kMaxTotalSeconds)
    throw ClawError(sfmt(
        "累计运动时间 %.1fs 超过安全上限 %.0fs，请减小时长或轮数",
        plan.total_motion_seconds(), kMaxTotalSeconds));

  if (plan.mode == kModeRunIn) {
    plan.angle_deg           = 0.0;
    plan.effective_angle_deg = 0.0;
    plan.effective_seconds   = plan.seconds;
    plan.angle_clipped       = false;
    return;
  }

  if (plan.speed_rpm < kMinSpeedRpm || plan.speed_rpm > kMaxSpeedRpm)
    throw ClawError(sfmt("position 模式下速度必须落在 %.1f..%.1f rpm，当前 %.2f",
                         kMinSpeedRpm, kMaxSpeedRpm, plan.speed_rpm));
  if (plan.motor_mask != kAllMotorsMask && plan.motor_mask != 0x01 &&
      plan.motor_mask != 0x02 && plan.motor_mask != 0x04)
    throw ClawError("位置模式只支持三路同时(mask=7)或单路(1/2/4)");

  // rpm → 度/秒 = rpm * 360 / 60 = rpm * 6
  plan.angle_deg           = plan.speed_rpm * 6.0 * plan.seconds;
  plan.effective_angle_deg = std::min(plan.angle_deg, kMaxTravelDeg);
  plan.angle_clipped       = plan.angle_deg > kMaxTravelDeg;
  plan.effective_seconds   = plan.effective_angle_deg / (plan.speed_rpm * 6.0);
}

// ============================ 打印 ============================
void print_rule() {
  std::printf("==============================================================================\n");
}

void print_plan(const Plan& p) {
  print_rule();
  std::printf("  手爪往复旋转测试  ——  执行计划\n");
  print_rule();
  std::printf("  CAN 通道        : %s (SocketCAN)\n", p.channel.c_str());
  std::printf("  固件基地址      : %u  (命令帧 ID=%u, 反馈帧 ID=%u..%u)\n",
              p.base_id, p.base_id, p.base_id + 1, p.base_id + 5);
  std::printf("  电机掩码        : %u  (7=三路同时)\n", p.motor_mask);
  std::printf("  模式            : %s\n", p.mode == kModeRunIn ? "run_in" : "position");
  std::printf("  每段时长        : %.2f s\n", p.seconds);
  std::printf("  往复轮数        : %d\n", p.cycles);
  std::printf("  累计运动        : %.1f s\n", p.total_motion_seconds());

  if (p.mode == kModeRunIn) {
    std::printf("  方向            : 正向 DIR=%d → 反向 DIR=%d\n",
                p.dir_forward, p.dir_reverse);
    std::printf("  速度            : ⚠ 由固件固定 1000‰ 占空比，kRotateSpeedRpm 不生效\n");
    std::printf("                    （要改物理速度需改固件 CLAW_RUN_IN_DUTY_PERMILLE 并重新烧写）\n");
  } else {
    std::printf("  速度            : %.1f rpm\n", p.speed_rpm);
    std::printf("  单向摆幅        : %.1f°  (速度×时间算出 %.1f°)\n",
                p.effective_angle_deg, p.angle_deg);
    if (p.angle_clipped) {
      std::printf("  ⚠ 摆幅超过固件 120° 行程上限，已被截断；\n");
      std::printf("    实际每段约 %.2fs，而不是 %.2fs\n",
                  p.effective_seconds, p.seconds);
    }
  }
  print_rule();
  if (p.mode == kModeRunIn) {
    std::printf("  ⚠ 安全提示：RUN_IN 是 1000‰ 满占空比开环输出，固件没有力矩/电流环。\n");
    std::printf("    必须确认三指在整个行程内都不会顶到机械限位，否则 800 ms 内无霍尔\n");
    std::printf("    变化会被固件判为堵转并锁定故障（此时程序会立即停机并报错）。\n");
    print_rule();
  }
}

/// 反馈打印回调，节流到每 0.5 秒最多一条，避免刷屏。
class Reporter {
 public:
  void operator()(const Feedback& fb) {
    const double now = now_seconds();

    if (fb.kind == Feedback::Kind::Fault) {
      std::fprintf(stderr, "\n    [故障帧] %s 状态=%s\n",
                   fault_text(fb.status.fault_mask).c_str(), state_name(fb.status.state));
      return;
    }
    if (now - last_print_ < 0.5) return;

    if (fb.kind == Feedback::Kind::Diagnostic) {
      last_print_ = now;
      std::printf("    [诊断] 占空比‰={%d,%d,%d}  霍尔码={%u,%u,%u}\n",
                  fb.diagnostic.duty_permille[0], fb.diagnostic.duty_permille[1],
                  fb.diagnostic.duty_permille[2], fb.diagnostic.hall_codes[0],
                  fb.diagnostic.hall_codes[1], fb.diagnostic.hall_codes[2]);
    } else if (fb.kind == Feedback::Kind::Position) {
      last_print_ = now;
      std::printf("    [位置] %+8.2f°  %+8.2f°  %+8.2f°  状态=%s\n",
                  fb.position.angles_deg[0], fb.position.angles_deg[1],
                  fb.position.angles_deg[2], state_name(fb.position.state));
    }
  }

 private:
  double last_print_ = 0.0;
};

// ============================ DRY RUN ============================
int dry_run(const Plan& p) {
  std::printf("\n---------- DRY RUN：以下是将要发送的帧（不打开 CAN 总线）----------\n");
  uint8_t seq = 0;

  if (p.mode == kModeRunIn) {
    for (int cycle = 1; cycle <= p.cycles; ++cycle) {
      for (int pass = 0; pass < 2; ++pass) {
        const char* label     = (pass == 0) ? "正向" : "反向";
        const int   direction = (pass == 0) ? p.dir_forward : p.dir_reverse;
        ++seq;
        const Frame f = encode_run_in(seq, direction, p.motor_mask);
        std::printf("  第%d轮 %s RUN_IN  id=0x%03X data=[%s]  保持 %.2fs\n",
                    cycle, label, p.base_id, to_hex(f).c_str(), p.seconds);
        std::printf("                   ↓ 期间每 %.1fs 发一次 HEARTBEAT\n", kHeartbeatSeconds);
      }
      std::printf("                   → 每段之间停机并静默 %.2fs\n", kDirectionSettleSeconds);
    }
  } else {
    for (int cycle = 1; cycle <= p.cycles; ++cycle) {
      for (int pass = 0; pass < 2; ++pass) {
        const char* label = (pass == 0) ? "正向" : "反向";
        const double sign = (pass == 0) ? +1.0 : -1.0;
        ++seq;
        const Frame f = encode_relative_move(seq, sign * p.effective_angle_deg,
                                             p.speed_rpm, p.motor_mask);
        std::printf("  第%d轮 %s 相对位移 id=0x%03X data=[%s]  %+.1f°\n",
                    cycle, label, p.base_id, to_hex(f).c_str(),
                    sign * p.effective_angle_deg);
      }
    }
  }
  std::printf("---------- DRY RUN 结束，未发送任何数据 ----------\n");
  return 0;
}

// ============================ 动作 ============================
void run_segment_run_in(ClawBus& bus, const Plan& p, int direction, const char* label,
                        Reporter& reporter) {
  std::printf("  → %s：RUN_IN 1000‰，DIR=%d，持续 %.2fs\n", label, direction, p.seconds);

  const uint8_t seq = bus.send_run_in(direction, p.motor_mask);
  bus.expect_ok(seq, 1.5, sfmt("%s RUN_IN 命令", label));

  // 分片 pump（每片 0.1s）保持进度显示；心跳由 ClawBus 的实例级时钟负责，不会漏喂
  double remaining = p.seconds;
  while (remaining > 0.0) {
    const double step = std::min(0.1, remaining);
    bus.pump(step, [&reporter](const Feedback& fb) { reporter(fb); });
    remaining -= step;
    std::printf("\r    已转 %5.2f / %.2f s", p.seconds - remaining, p.seconds);
    std::fflush(stdout);
  }
  std::printf("\n");

  bus.stop();
  std::printf("    %s结束，发 STOP\n", label);
}

void run_segment_position(ClawBus& bus, const Plan& p, int direction, const char* label) {
  const double angle =
      p.effective_angle_deg * (direction == p.dir_forward ? 1.0 : -1.0);
  std::printf("  → %s：相对位移 %+.1f° @ %.1f rpm\n", label, angle, p.speed_rpm);

  const double started = now_seconds();
  const uint8_t seq = bus.send_relative_move(angle, p.speed_rpm, p.motor_mask);
  bus.expect_ok(seq, 1.5, sfmt("%s位移命令", label));

  const double budget = std::fabs(angle) / (p.speed_rpm * 6.0) + kSegmentTimeoutMargin;
  bus.wait_target_reached(budget);
  std::printf("    %s完成，实际耗时 %.2fs\n", label, now_seconds() - started);
}

int run(const Plan& p) {
  print_plan(p);
  Reporter reporter;

  auto transport = make_transport(kPort, p.channel.c_str());
  ClawBus bus(transport, p.base_id);

  // ---- 1. 清故障，确保从干净的 IDLE 起步 ----
  std::printf("\n[1/3] 清故障并确认固件在线 ...\n");
  StatusFeedback status = bus.clear_fault();
  std::printf("      OK  状态=%s 故障位=0x%02X\n",
              state_name(status.state), status.fault_mask);
  bus.settle(kDirectionSettleSeconds);

  // ---- 2. 往复旋转 ----
  std::printf("\n[2/3] 开始往复旋转，共 %d 轮\n", p.cycles);
  const double motion_started = now_seconds();

  for (int cycle = 1; cycle <= p.cycles; ++cycle) {
    std::printf("\n--- 第 %d/%d 轮 ---\n", cycle, p.cycles);

    if (p.mode == kModeRunIn) {
      run_segment_run_in(bus, p, p.dir_forward, "正向", reporter);
      bus.settle(kDirectionSettleSeconds);
      run_segment_run_in(bus, p, p.dir_reverse, "反向", reporter);
      bus.settle(kDirectionSettleSeconds);
    } else {
      run_segment_position(bus, p, p.dir_forward, "正向");
      bus.settle(kDirectionSettleSeconds);
      run_segment_position(bus, p, p.dir_reverse, "反向");
      bus.settle(kDirectionSettleSeconds);
    }

    const double spent = now_seconds() - motion_started;
    if (spent > kMaxTotalSeconds) {
      std::fprintf(stderr, "  ⚠ 累计运行 %.1fs 已达上限，提前结束\n", spent);
      break;
    }
  }

  // ---- 3. 收尾 ----
  std::printf("\n[3/3] 停机\n");
  auto final_status = bus.stop(1.0);
  if (final_status) {
    std::printf("      状态=%s 故障位=0x%02X\n",
                state_name(final_status->state), final_status->fault_mask);
    if (final_status->fault_mask) {
      std::fprintf(stderr, "      ⚠ %s\n",
                   fault_text(final_status->fault_mask).c_str());
      return 3;
    }
  }
  return 0;
}

// ============================ 命令行 ============================
struct Args {
  bool        confirm  = false;
  bool        dry_run  = false;
  bool        has_mode = false;
  Mode        mode     = kMode;
  double      seconds  = kRotateSeconds;
  double      speed    = kRotateSpeedRpm;
  int         cycles   = kCycles;
  int         dir      = kDirForward;
  std::string channel  = kChannel;
  int         base     = kBaseId;
  uint8_t     mask     = kMotorMask;
};

void usage(const char* prog) {
  std::printf(
      "手爪三指同速往复旋转（testcmd_ssj1110 功能 1）\n"
      "\n"
      "用法: %s [选项]\n"
      "\n"
      "  -h, --help              显示本帮助\n"
      "      --mode MODE         覆盖 kMode：run_in | position\n"
      "      --seconds SEC       覆盖 kRotateSeconds\n"
      "      --speed RPM         覆盖 kRotateSpeedRpm（仅 position 模式生效）\n"
      "      --cycles N          覆盖 kCycles\n"
      "      --dir 0|1           覆盖 kDirForward\n"
      "      --channel NAME      覆盖 kChannel，如 can1\n"
      "      --base ID           覆盖 kBaseId（%u..%u）\n"
      "      --mask 1..7         覆盖 kMotorMask，7=三路同时\n"
      "      --confirm           确认驱动电机（不加则只打印计划）\n"
      "      --dry-run           不打开总线，只打印要发的帧\n"
      "\n"
      "默认只打印计划；加 --confirm 才会真正驱动电机。\n",
      prog, kBaseIdMin, kBaseIdMax);
}

double parse_double(const char* text, const char* name) {
  char* end  = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0')
    throw ClawError(sfmt("%s 需要是数字，收到 '%s'", name, text));
  return value;
}

long parse_long(const char* text, const char* name) {
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0')
    throw ClawError(sfmt("%s 需要是整数，收到 '%s'", name, text));
  return value;
}

Args parse_args(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string opt = argv[i];
    auto need_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) throw ClawError(sfmt("%s 缺少参数值", name));
      return argv[++i];
    };

    if (opt == "-h" || opt == "--help") {
      usage(argv[0]);
      std::exit(0);
    } else if (opt == "--confirm") {
      args.confirm = true;
    } else if (opt == "--dry-run") {
      args.dry_run = true;
    } else if (opt == "--mode") {
      const std::string value = need_value("--mode");
      if (value == "run_in")       args.mode = kModeRunIn;
      else if (value == "position") args.mode = kModePosition;
      else throw ClawError("--mode 只能是 run_in 或 position");
      args.has_mode = true;
    } else if (opt == "--seconds") {
      args.seconds = parse_double(need_value("--seconds"), "--seconds");
    } else if (opt == "--speed") {
      args.speed = parse_double(need_value("--speed"), "--speed");
    } else if (opt == "--cycles") {
      args.cycles = static_cast<int>(parse_long(need_value("--cycles"), "--cycles"));
    } else if (opt == "--dir") {
      args.dir = static_cast<int>(parse_long(need_value("--dir"), "--dir"));
    } else if (opt == "--channel") {
      args.channel = need_value("--channel");
    } else if (opt == "--base") {
      args.base = static_cast<int>(parse_long(need_value("--base"), "--base"));
    } else if (opt == "--mask") {
      args.mask = static_cast<uint8_t>(parse_long(need_value("--mask"), "--mask"));
    } else {
      throw ClawError(sfmt("未知选项 %s（用 --help 查看）", opt.c_str()));
    }
  }

  if (args.base < kBaseIdMin || args.base > kBaseIdMax)
    throw ClawError(sfmt("--base 必须在 %u..%u", kBaseIdMin, kBaseIdMax));
  if (args.mask == 0 || args.mask > kAllMotorsMask)
    throw ClawError("--mask 只能在 1..7");
  if (args.dir != 0 && args.dir != 1) throw ClawError("--dir 只能是 0 或 1");
  return args;
}

/// 用代码顶部的常量造一份默认计划。
Plan default_plan() {
  Plan plan;
  plan.mode        = kMode;
  plan.seconds     = kRotateSeconds;
  plan.speed_rpm   = kRotateSpeedRpm;
  plan.cycles      = kCycles;
  plan.dir_forward = kDirForward;
  plan.motor_mask  = kMotorMask;
  plan.base_id     = kBaseId;
  plan.channel     = kChannel;
  return plan;
}

/// 把命令行覆盖叠在默认计划上，然后统一校验。
Plan plan_from_args(const Args& args) {
  Plan plan     = default_plan();
  plan.mode     = args.mode;
  plan.seconds  = args.seconds;
  plan.speed_rpm = args.speed;
  plan.cycles   = args.cycles;
  plan.dir_forward = args.dir;
  plan.motor_mask  = args.mask;
  plan.base_id     = static_cast<uint16_t>(args.base);
  plan.channel     = args.channel;
  validate_and_fill(plan);
  return plan;
}

}  // namespace

int main(int argc, char** argv) {
  install_interrupt_handler();

  Args args;
  Plan plan;
  try {
    args = parse_args(argc, argv);
    plan = plan_from_args(args);
  } catch (const ClawError& exc) {
    std::fprintf(stderr, "参数错误：%s\n", exc.what());
    std::fprintf(stderr, "用 --help 查看用法。\n");
    return 1;
  }

  if (args.dry_run) {
    print_plan(plan);
    return dry_run(plan);
  }

  if (!args.confirm) {
    print_plan(plan);
    std::printf("\n当前为【计划模式】，未发送任何数据。\n");
    std::printf("确认无误后加 --confirm 真正运行，例如：\n");
    std::printf("  %s --confirm\n", argv[0]);
    return 0;
  }

  try {
    return run(plan);
  } catch (const ClawInterrupted& exc) {
    std::fprintf(stderr, "\n用户中断（Ctrl+C），已发送 STOP：%s\n", exc.what());
    return 130;
  } catch (const ClawError& exc) {
    std::fprintf(stderr, "\n运行中止：%s\n", exc.what());
    return 2;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "\n运行中止（未知错误）：%s\n", exc.what());
    return 2;
  }
}