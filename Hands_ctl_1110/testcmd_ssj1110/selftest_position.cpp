// =============================================================================
//  selftest_position.cpp  ——  离线验证 test_positionctrl 的安全逻辑
//
//  不需要硬件、不需要 root、不需要起 CAN 通道。验证两件事：
//    1. ★ 500 ms 无有效霍尔变化 → 判定堵转
//    2. 目标坐标越界 / 增量过小 → 提前拒绝（而不是让固件静默截断）
//
//  编译运行：
//    ./build.sh && ./selftest_position
// =============================================================================

#include <cmath>
#include <cstdio>
#include <string>

#include "position_ctrl_logic.h"

using namespace posctrl;

namespace {

int g_failures = 0;

void check(bool condition, const std::string& what) {
  if (condition) {
    std::printf("    \033[32m✓\033[0m %s\n", what.c_str());
  } else {
    std::printf("    \033[31m✗ %s\033[0m\n", what.c_str());
    ++g_failures;
  }
}

void section(const char* title) {
  std::printf("\n[%s]\n", title);
}

// ---------------------------------------------------------------- 霍尔码合法性
void test_hall_validity() {
  section("霍尔码合法性");
  const int legal[] = {0x1, 0x5, 0x4, 0x6, 0x2, 0x3};
  bool all_legal = true;
  for (int code : legal) all_legal = all_legal && hall_code_valid(code);
  check(all_legal, "固件扇区表 {1,5,4,6,2,3} 全部判为合法");
  check(!hall_code_valid(0x0), "0x0 判为非法（霍尔全低，通常未接电机）");
  check(!hall_code_valid(0x7), "0x7 判为非法（霍尔全高/悬空，GPIO_NOPULL 的典型读数）");

  check(motor_mask_of(1) == 0x01, "M1 -> 掩码 0x01");
  check(motor_mask_of(2) == 0x02, "M2 -> 掩码 0x02");
  check(motor_mask_of(3) == 0x04, "M3 -> 掩码 0x04");
}

// ---------------------------------------------------------------- ★ 500ms 看门狗
void test_watchdog_healthy_motion() {
  section("看门狗：正常转动时不该误报");
  HallWatchdog dog;
  dog.begin(2, 0.0);

  // 每 20 ms 一帧诊断，霍尔码按固件顺序 1->5->4->6->2->3->1 循环
  const int seq[] = {0x1, 0x5, 0x4, 0x6, 0x2, 0x3};
  double t = 0.0;
  int idx = 0;
  bool ever_stalled = false;
  for (int i = 0; i < 200; ++i) {  // 4 秒
    dog.feed(seq[idx % 6], t);
    ++idx;
    t += 0.020;
    if (dog.stalled(t)) ever_stalled = true;
  }
  check(!ever_stalled, "4 秒连续翻转期间从未判定堵转");
  check(dog.valid_edges() >= 190, "有效边沿计数正常累加（>=190）");
}

void test_watchdog_stall_detection() {
  section("看门狗：★ 500ms 无变化必须断电");
  HallWatchdog dog;
  dog.begin(2, 0.0);
  dog.feed(0x1, 0.0);  // 首帧建立基准

  // 之后霍尔码一直不动
  check(!dog.stalled(0.49), "静默 490 ms 时判定为未堵转");
  check(dog.stalled(0.50), "★ 静默 500 ms 整即判定堵转");
  check(dog.stalled(2.00), "静默 2 s 仍判定堵转");
  check(dog.valid_edges() == 0, "全程无有效边沿");

  // 中途恢复一次翻转，计时器必须重置
  HallWatchdog dog2;
  dog2.begin(2, 0.0);
  dog2.feed(0x1, 0.0);
  check(dog2.stalled(0.50), "静默 500 ms 判定堵转");
  dog2.feed(0x5, 0.60);  // 恢复
  check(!dog2.stalled(0.70), "恢复翻转后立刻解除堵转");
  check(!dog2.stalled(1.09), "恢复后 490 ms 仍未堵转");
  check(dog2.stalled(1.11), "恢复后 500 ms 再次判定堵转");
}

void test_watchdog_invalid_codes_not_progress() {
  section("看门狗：非法霍尔码跳变不算进展");
  HallWatchdog dog;
  dog.begin(2, 0.0);
  dog.feed(0x1, 0.0);

  // 0x1 -> 0x7 -> 0x0 -> 0x7 都是非法的"抖动"，霍尔线松了才会这样
  dog.feed(0x7, 0.10);
  check(dog.valid_edges() == 0, "0x1 -> 0x7 不计为有效边沿");
  check(!dog.stalled(0.40), "此时静默 400 ms");
  dog.feed(0x0, 0.45);
  check(dog.valid_edges() == 0, "0x7 -> 0x0 不计为有效边沿");
  check(dog.stalled(0.50), "★ 非法抖动不能拖延断电，500 ms 照样断");

  // 从非法码回到合法码也不算（固件要求连续合法扇区推进）
  HallWatchdog dog3;
  dog3.begin(2, 0.0);
  dog3.feed(0x7, 0.0);   // 基准就是非法码
  dog3.feed(0x1, 0.10);  // 回到合法码
  check(dog3.valid_edges() == 1, "从非法码进入合法码算一次有效变化（可继续转）");
}

void test_watchdog_same_code_not_progress() {
  section("看门狗：同一个码重复上报不算进展");
  HallWatchdog dog;
  dog.begin(1, 0.0);
  int code = 0x1;
  double t = 0.0;
  for (int i = 0; i < 100; ++i) {  // 2 秒都是同一个码
    dog.feed(code, t);
    t += 0.020;
  }
  check(dog.valid_edges() == 0, "2 秒内码不变 -> 有效边沿 0");
  check(dog.stalled(2.0), "仍然判定堵转");
}

void test_watchdog_faster_than_firmware() {
  section("看门狗：必须早于固件 800ms 堵转保护");
  check(kStallWindowSeconds < kFirmwareStallSeconds,
        "500 ms < 固件 800 ms（撞限位时固件不锁 fault，无需 CLEAR_FAULT）");
  check(std::fabs(kStallWindowSeconds - 0.5) < 1e-9, "窗口精确为 0.500 s");
}

// ---------------------------------------------------------------- 目标越界
void test_target_bounds_prezero() {
  section("目标越界：未标零（-30..+120）");
  auto c = check_target(0.0, 10.0, false);
  check(c.ok(), "从 0° 走 +10° 允许");
  check(std::fabs(c.target_deg - 10.0) < 1e-9, "目标坐标算成 +10°");

  auto c2 = check_target(0.0, 50.0, false);
  check(c2.ok(), "从 0° 走 +50° 允许");
  check(std::fabs(c2.max_delta - 120.0) < 1e-9, "提示的最大增量 = 120°，对应上限 120°");

  auto c3 = check_target(0.0, -20.0, false);
  check(c3.ok(), "从 0° 走 -20° 允许（下限 -30°）");
  check(std::fabs(c3.min_delta - (-30.0)) < 1e-9, "提示的最小增量 = -30°");

  auto c4 = check_target(0.0, -40.0, false);
  check(!c4.ok(), "从 0° 走 -40° 被拒（会到 -40°，低于 -30°）");
  check(c4.verdict == TargetVerdict::BelowMin, "判定为低于下限");
}

void test_target_bounds_zeroed() {
  section("目标越界：已标零（0..+120）");
  auto c = check_target(0.0, -10.0, true);
  check(!c.ok(), "标零后从 0° 走 -10° 被拒");
  check(c.verdict == TargetVerdict::BelowMin, "判定为低于下限");
  check(std::fabs(c.min_delta) < 1e-9, "标零后提示的最小增量 = 0°");

  auto c2 = check_target(50.0, -10.0, true);
  check(c2.ok(), "标零后从 50° 走 -10° 允许（到 40°）");

  auto c3 = check_target(50.0, -60.0, true);
  check(!c3.ok(), "标零后从 50° 走 -60° 被拒（到 -10°）");
  check(std::fabs(c3.min_delta - (-50.0)) < 1e-9, "提示最小增量 = -50°");
}

void test_target_repeated_moves_clamp() {
  section("目标越界：连续 +50° 累计会被拦");
  // 模拟：0 -> 50 -> 100 -> 应被拒（150 > 120）
  double pos = 0.0;
  auto a = check_target(pos, 50.0, false);
  check(a.ok(), "第 1 次 +50°（到 50°）允许");
  pos = a.target_deg;

  auto b = check_target(pos, 50.0, false);
  check(b.ok(), "第 2 次 +50°（到 100°）允许");
  pos = b.target_deg;

  auto d = check_target(pos, 50.0, false);
  check(!d.ok(), "★ 第 3 次 +50°（到 150°）被拒，不会静默截断在 120°");
  check(d.verdict == TargetVerdict::AboveMax, "判定为高于上限");
  check(std::fabs(d.max_delta - 20.0) < 1e-9, "提示当前最多还能走 +20°");
}

void test_target_tiny_delta() {
  section("目标过小：低于死区直接拒绝");
  auto c = check_target(0.0, 0.05, false);
  check(!c.ok(), "0.05° 被拒（固件会直接判到位、不进 MOVING）");
  check(c.verdict == TargetVerdict::TooSmall, "判定为增量过小");

  auto c2 = check_target(0.0, 0.0, false);
  check(!c2.ok(), "0° 被拒");

  auto c3 = check_target(0.0, kMinDeltaDeg + 0.1, false);
  check(c3.ok(), "略大于阈值的增量允许通过");

  check(kDegPerHallCount > 0.09 && kDegPerHallCount < 0.10,
        "1 Hall count ≈ 0.0955°（与固件 628.5:1 一致）");
  check(kMinDeltaDeg > kPositionDeadbandCounts * kDegPerHallCount,
        "阈值大于固件死区，留有余量");
}

void test_target_epsilon_tolerance() {
  section("目标越界：浮点边界容差");
  // 从 70° 走 +50° = 120.0°，正好卡在上限，必须允许
  auto c = check_target(70.0, 50.0, false);
  check(c.ok(), "正好落在 +120° 上限的移动允许通过");

  // 从 70° 走 +50.000001° 应仍允许（epsilon 容忍）
  auto c2 = check_target(70.0, 50.0000001, false);
  check(c2.ok(), "微小浮点超出仍允许（1e-6 容差）");

  // 超出 0.1° 必须拒
  auto c3 = check_target(70.0, 50.1, false);
  check(!c3.ok(), "超出上限 0.1° 被拒");
}

// ---------------------------------------------------------------- ★ 位置饱和
void test_saturation_detection() {
  section("位置帧饱和判定（int16 钳位）");

  // 固件 BuildPositionFrame 把角度硬钳在 int16 cdeg：上限 INT16_MAX/100
  check(std::fabs(kPositionSaturationDeg - 327.67) < 1e-9,
        "饱和值常量 = 327.67°（INT16_MAX / 100）");
  check(kSaturationWarnDeg < kPositionSaturationDeg,
        "判定阈值留了余量（阈值 < 理论饱和值）");

  // ★ 实测现场读数：三路全部贴在 +327.67°
  check(position_saturated(+327.67), "+327.67° 判为饱和（实测现场值）");
  check(position_saturated(-327.67), "-327.67° 判为饱和（反向量程）");
  check(position_saturated(+327.0),  "+327.0° 判为饱和（贴近量程上限）");

  // 正常范围不能被误判
  check(!position_saturated(0.0),    "0.0° 判为正常");
  check(!position_saturated(+120.0), "+120.0°（固件上限）判为正常");
  check(!position_saturated(-30.0),  "-30.0°（未标零下限）判为正常");
  check(!position_saturated(+319.9), "+319.9° 判为正常（恰好低于阈值）");
  check(position_saturated(+320.0),  "+320.0° 判为饱和（恰好到阈值）");
}

// ---------------------------------------------------------------- 饱和时拒绝发命令
void test_target_rejects_saturated() {
  section("★ 饱和时 check_target 必须拦截（防止实际转角翻倍）");

  // 场景重演：实机读到 +327.67°，用户输入 +5°。
  // 旧逻辑会算出 target=+332.67 > 120 -> AboveMax，并给出
  // 「可行增量范围 -357.67° .. -207.67°」——这个范围是错的，
  // 因为固件用的是未饱和的真实计数。
  const auto bad = check_target(+327.67, +5.0, false);
  check(!bad.ok(), "饱和 + 增量 +5° 被拒绝");
  check(bad.verdict == TargetVerdict::Saturated,
        "判定为 Saturated（而不是 AboveMax）——不再给出误导性的可行范围");

  // 即使目标坐标"看起来"合法也不能发
  const auto sneaky = check_target(+327.67, -210.0, false);
  check(!sneaky.ok(), "饱和 + 增量 -210° 同样被拒绝（目标看似在范围内）");
  check(sneaky.verdict == TargetVerdict::Saturated, "仍然判为 Saturated");

  // 显式传 saturated 标志也能拦
  const auto flagged = check_target(0.0, +10.0, false, /*saturated=*/true);
  check(flagged.verdict == TargetVerdict::Saturated, "saturated 参数为 true 时直接拒绝");

  // 不饱和时行为不变
  const auto fine = check_target(0.0, +10.0, true);
  check(fine.ok(), "标零后 0° + 10° 正常放行（行为未变）");
}

// ---------------------------------------------------------------- AngleState 绑定
void test_angle_state() {
  section("AngleState：角度与饱和标记绑定");

  AngleState a;
  check(!a.saturated && a.deg == 0.0, "默认构造：0° 且未饱和");

  a.update(+327.67);
  check(a.saturated, "update(+327.67) 后标记为饱和");
  check(std::fabs(a.deg - 327.67) < 1e-9, "保留了原始读数（用于报错展示）");

  // ★ 标零后的关键场景：固件计数清零，位置帧回到 0°
  a.update(0.0);
  check(!a.saturated, "update(0.0) 后饱和标记解除（即标零生效）");

  a.update(+50.0);
  check(!a.saturated, "正常行程 50° 不报饱和");

  AngleState b(327.67);
  check(b.saturated, "单参构造会把饱和判定算好");
}

// ---------------------------------------------------------------- 超时预算
void test_move_budget() {
  section("超时预算");
  const double b10 = move_budget_seconds(10.0, 8.0);
  check(std::fabs(b10 - 10.208) < 0.01, "10° @ 8rpm ≈ 10.21 s");
  check(b10 < kMoveTimeoutCapSeconds, "未超 14 s 上限");

  const double bbig = move_budget_seconds(120.0, 4.0);
  check(std::fabs(bbig - kMoveTimeoutCapSeconds) < 1e-9, "120° @ 4rpm 被夹到 14 s");

  const double bneg = move_budget_seconds(-10.0, 8.0);
  check(std::fabs(bneg - b10) < 1e-9, "负增量预算与正增量相同");

  check(kMoveTimeoutCapSeconds < 15.0, "预算始终早于固件 15 s 动作超时");
}

// ---------------------------------------------------------------- 电机组合解析
void test_parse_motor_mask() {
  section("电机组合解析");
  check(parse_motor_mask("1") == 0x01, "\"1\" -> 0x01");
  check(parse_motor_mask("2") == 0x02, "\"2\" -> 0x02");
  check(parse_motor_mask("3") == 0x04, "\"3\" -> 0x04");
  check(parse_motor_mask("12") == 0x03, "\"12\" -> 0x03 (M1+M2)");
  check(parse_motor_mask("13") == 0x05, "\"13\" -> 0x05 (M1+M3)");
  check(parse_motor_mask("23") == 0x06, "\"23\" -> 0x06 (M2+M3)");
  check(parse_motor_mask("123") == 0x07, "\"123\" -> 0x07 (三路)");
  check(parse_motor_mask("321") == 0x07, "顺序无关：\"321\" -> 0x07");
  check(parse_motor_mask("m12") == 0x03, "\"m12\" 允许 m 前缀");
  check(parse_motor_mask("M1 M3") == 0x05, "\"M1 M3\" 允许空格分隔");
  check(parse_motor_mask("m1,3") == 0x05, "\"m1,3\" 允许逗号分隔");
  check(parse_motor_mask("1+2") == 0x03, "\"1+2\" 允许加号分隔");
  check(parse_motor_mask("1|2") == 0x03, "\"1|2\" 允许竖线分隔");
  check(parse_motor_mask("0") == 0, "\"0\" 非法（电机号只有 1..3）");
  check(parse_motor_mask("4") == 0, "\"4\" 非法");
  check(parse_motor_mask("14") == 0, "\"14\" 含非法位 -> 0");
  check(parse_motor_mask("") == 0, "空串 -> 0");
  check(parse_motor_mask("abc") == 0, "非数字 -> 0");
  check(parse_motor_mask("1m2") == 0, "m 在中途 -> 0");
  check(parse_motor_mask("m1 m3") == 0x05, "\"m1 m3\" 每段都允许 m 前缀");
  check(parse_motor_mask("mm12") == 0, "双写前缀 \"mm12\" -> 0");
  check(parse_motor_mask("m") == 0, "只有前缀没有数字 -> 0");

  check(mask_motor_count(0x07) == 3, "0x07 有 3 路");
  check(mask_motor_count(0x03) == 2, "0x03 有 2 路");
  check(mask_motor_count(0x04) == 1, "0x04 有 1 路");
  check(mask_motor_count(0x00) == 0, "0x00 有 0 路");

  check(mask_text(0x07) == "M1+M2+M3", "mask_text(0x07)");
  check(mask_text(0x03) == "M1+M2", "mask_text(0x03)");
  check(mask_text(0x05) == "M1+M3", "mask_text(0x05)");
  check(mask_text(0x02) == "M2", "mask_text(0x02)");
  check(mask_text(0x00) == "(无)", "mask_text(0x00)");

  // ★ 固件的位置掩码编码在 opcode 里，只有这四种
  check(firmware_supported_mask(0x07), "0x07 有单帧（SET_TARGET）");
  check(firmware_supported_mask(0x01), "0x01 有单帧（SET_TARGET_M1）");
  check(firmware_supported_mask(0x02), "0x02 有单帧（SET_TARGET_M2）");
  check(firmware_supported_mask(0x04), "0x04 有单帧（SET_TARGET_M3）");
  check(!firmware_supported_mask(0x03), "★ 0x03(M1+M2) 固件无单帧");
  check(!firmware_supported_mask(0x05), "★ 0x05(M1+M3) 固件无单帧");
  check(!firmware_supported_mask(0x06), "★ 0x06(M2+M3) 固件无单帧");
}

// ---------------------------------------------------------------- 掩码拆分
void test_split_move_masks() {
  section("掩码拆分（两路组合必须顺序执行）");
  {
    const auto m = split_move_masks(0x07);
    check(m.size() == 1 && m[0] == 0x07, "0x07 -> {0x07} 一条帧");
  }
  {
    const auto m = split_move_masks(0x02);
    check(m.size() == 1 && m[0] == 0x02, "0x02 -> {0x02} 一条帧");
  }
  {
    const auto m = split_move_masks(0x03);
    check(m.size() == 2 && m[0] == 0x01 && m[1] == 0x02, "0x03 -> {0x01,0x02}");
  }
  {
    const auto m = split_move_masks(0x06);
    check(m.size() == 2 && m[0] == 0x02 && m[1] == 0x04, "0x06 -> {0x02,0x04}");
  }
  {
    const auto m = split_move_masks(0x05);
    check(m.size() == 2 && m[0] == 0x01 && m[1] == 0x04, "0x05 -> {0x01,0x04}");
  }
  // 拆分后每一份都必须是固件支持的
  bool all_ok = true;
  for (uint8_t mask = 1; mask <= 7; ++mask)
    for (uint8_t one : split_move_masks(mask))
      all_ok = all_ok && firmware_supported_mask(one);
  check(all_ok, "0x01..0x07 拆分后每份都被固件支持");
}

// ---------------------------------------------------------------- 多路坐标语义
void test_multi_motor_coordinates() {
  section("多路坐标语义（固件用平均值）");

  // 三路起点相同时，输入 D 就是每路都转 D
  {
    const double pos[3] = {10.0, 10.0, 10.0};
    check(std::fabs(frame_reference_deg(pos, 0x07) - 10.0) < 1e-9,
          "三路同为 10° -> 参考坐标 10°");
    double disp[3] = {0, 0, 0};
    displacement_of(pos, 0x07, 5.0, disp);
    check(std::fabs(disp[0] - 5.0) < 1e-9 && std::fabs(disp[1] - 5.0) < 1e-9 &&
              std::fabs(disp[2] - 5.0) < 1e-9,
          "起点相同 +5° -> 每路都转 5°");
  }

  // ★ 起点不同时，各路实际转角不同（这是最容易被误解的地方）
  {
    const double pos[3] = {10.0, 20.0, 30.0};
    check(std::fabs(frame_reference_deg(pos, 0x07) - 20.0) < 1e-9,
          "10/20/30 -> 参考坐标 = 平均值 20°");
    double disp[3] = {0, 0, 0};
    displacement_of(pos, 0x07, 10.0, disp);
    // 目标 = 20 + 10 = 30；各自位移 = 30 - pos[i]
    check(std::fabs(disp[0] - 20.0) < 1e-9, "M1 10° -> 30°，实际转 +20°");
    check(std::fabs(disp[1] - 10.0) < 1e-9, "M2 20° -> 30°，实际转 +10°");
    check(std::fabs(disp[2] - 0.0) < 1e-9, "★ M3 30° -> 30°，实际转 0°（不动）");
  }

  // 只选部分路时，参考坐标只算选中的
  {
    const double pos[3] = {10.0, 20.0, 100.0};
    check(std::fabs(frame_reference_deg(pos, 0x03) - 15.0) < 1e-9,
          "选 M1+M2 (10/20) -> 参考坐标 15°（不含 M3 的 100°）");
    check(std::fabs(frame_reference_deg(pos, 0x04) - 100.0) < 1e-9,
          "只选 M3 -> 参考坐标 100°");
    double disp[3] = {0, 0, 0};
    displacement_of(pos, 0x03, 5.0, disp);
    check(std::fabs(disp[2]) < 1e-9, "未选中的 M3 位移为 0");
  }

  // ★ 位移小于位置死区时，固件直接判到位，从不给 PWM
  {
    const double pos[3] = {10.0, 10.0, 10.0};
    bool will[3] = {false, false, false};
    will_move_of(pos, 0x07, 0.01, will);
    check(!will[0] && !will[1] && !will[2], "0.01° 三路都不算「会动」（<= 死区）");

    will_move_of(pos, 0x07, 5.0, will);
    check(will[0] && will[1] && will[2], "5° 三路都算「会动」");
  }
  {
    // M2 起点已经等于目标 -> 只有 M1/M3 会动
    const double pos[3] = {10.0, 20.0, 20.0};
    bool will[3] = {false, false, false};
    will_move_of(pos, 0x07, 10.0, will);
    // 目标 = (10+20+20)/3 + 10 = 16.667 + 10 = 26.667
    check(will[0], "M1 10° -> 26.67°，会动");
    check(will[1], "M2 20° -> 26.67°，会动");
    check(will[2], "M3 20° -> 26.67°，会动");
  }
}

// ---------------------------------------------------------------- 多路超时预算
void test_move_budget_multi() {
  section("多路超时预算（按最大位移算）");
  const double pos[3] = {10.0, 20.0, 30.0};
  // 目标 = 20 + 10 = 30 -> 位移 20 / 10 / 0 -> 最大 20
  const double b = move_budget_seconds_multi(pos, 0x07, 10.0, 8.0);
  const double expect = move_budget_seconds(20.0, 8.0);
  check(std::fabs(b - expect) < 1e-9, "按最大位移 20° 算预算（不是按输入 10°）");

  // 起点一致时，预算就按输入增量算
  const double same[3] = {5.0, 5.0, 5.0};
  const double b2 = move_budget_seconds_multi(same, 0x07, 10.0, 8.0);
  check(std::fabs(b2 - move_budget_seconds(10.0, 8.0)) < 1e-9,
        "起点一致 -> 预算按输入增量 10°");
}

// ---------------------------------------------------------------- ★ 多路看门狗
void test_multi_watchdog_healthy() {
  section("多路看门狗：三路正常转动");
  MultiHallWatchdog dog;
  bool will[3] = {true, true, true};
  dog.begin(0x07, will, 0.0);
  check(dog.any_watching(), "开始时三路都在监视");
  check(dog.selected(1) && dog.selected(2) && dog.selected(3), "三路都被选中");

  const int seq[] = {0x1, 0x5, 0x4, 0x6, 0x2, 0x3};
  double t = 0.0;
  bool   ever_stalled = false;
  for (int i = 0; i < 60; ++i) {
    t += 0.02;
    for (int m = 1; m <= 3; ++m) {
      dog.feed_duty(m, 600, t);
      dog.feed_hall(m, seq[i % 6], t);
    }
    if (dog.stalled(t)) ever_stalled = true;
  }
  check(!ever_stalled, "1.2 s 正常转动全程未报堵转");
  check(dog.total_edges() > 0, "有霍尔边沿被计入");
  check(dog.stalled_motor(t) == 0, "最终判定：无堵转");
}

void test_multi_watchdog_stall() {
  section("多路看门狗：全部停转 -> 判堵转");
  MultiHallWatchdog dog;
  bool will[3] = {true, true, true};
  dog.begin(0x07, will, 0.0);

  // 前 100 ms 正常转动
  const int seq[] = {0x1, 0x5, 0x4, 0x6, 0x2, 0x3};
  double t = 0.0;
  for (int i = 0; i < 5; ++i) {
    t += 0.02;
    for (int m = 1; m <= 3; ++m) {
      dog.feed_duty(m, 600, t);
      dog.feed_hall(m, seq[i % 6], t);
    }
  }
  check(dog.stalled_motor(t) == 0, "0.1 s 时还没堵转");

  // 之后占空比仍然存在（固件还在驱动）但霍尔不再变化
  for (int i = 0; i < 30; ++i) {
    t += 0.02;
    for (int m = 1; m <= 3; ++m) {
      dog.feed_duty(m, 600, t);
      dog.feed_hall(m, 0x4, t);
    }
    if (dog.stalled(t)) break;
  }
  check(dog.stalled_motor(t) != 0, "★ 500 ms 无有效霍尔 -> 判堵转");
  check(dog.stalled_motor(t) == 1, "最先超时的报 M1");
}

// ---------------------------------------------------------------- ★★ 核心安全用例
void test_multi_watchdog_one_motor_done() {
  section("★ 核心安全用例：某一路先到位被固件断电，不算堵转");

  //  真实场景（claw_control.c:1154）：三路一起动，M2 先到达目标，
  //  固件对 M2 单独 set_pwm(0) 并 continue，M2 的霍尔立刻停止变化。
  //  此时 M1/M3 仍在正常转动 —— 绝不能被判成堵转然后 STOP 掉健康电机。
  MultiHallWatchdog dog;
  bool will[3] = {true, true, true};
  dog.begin(0x07, will, 0.0);

  const int seq[] = {0x1, 0x5, 0x4, 0x6, 0x2, 0x3};
  double t = 0.0;
  int  idx = 0;

  // 阶段 1：三路一起转 100 ms
  for (int i = 0; i < 5; ++i) {
    t += 0.02;
    for (int m = 1; m <= 3; ++m) {
      dog.feed_duty(m, 600, t);
      dog.feed_hall(m, seq[idx % 6], t);
    }
    ++idx;
  }
  check(!dog.stalled(t), "阶段 1：三路正常，无堵转");

  // 阶段 2：M2 到位 -> 固件给它断电（duty 变 0），M2 霍尔冻结
  //        M1/M3 继续转 1 s（远超过 500 ms 窗口）
  for (int i = 0; i < 50; ++i) {
    t += 0.02;
    dog.feed_duty(1, 600, t);
    dog.feed_hall(1, seq[idx % 6], t);
    dog.feed_duty(2, 0, t);              // ★ M2 已被固件单独断电
    dog.feed_hall(2, seq[(idx - 1) % 6], t);  // 霍尔冻结在上一个码
    dog.feed_duty(3, 600, t);
    dog.feed_hall(3, seq[idx % 6], t);
    ++idx;
  }
  check(!dog.stalled(t), "★★ M2 断电停转 1 s，M1/M3 仍在转 -> 不判堵转");
  check(dog.done(2), "M2 被标记为已完成");
  check(!dog.done(1) && !dog.done(3), "M1/M3 未标记完成");
  check(dog.watching(1) && dog.watching(3), "M1/M3 仍在监视");
  check(!dog.watching(2), "M2 已不在监视范围");
  check(dog.done_count() == 1, "已完成 1 路");

  // 阶段 3：M1/M3 也到位断电
  for (int i = 0; i < 3; ++i) {
    t += 0.02;
    for (int m = 1; m <= 3; ++m) {
      dog.feed_duty(m, 0, t);
      dog.feed_hall(m, 0x4, t);
    }
  }
  check(!dog.any_watching(), "★ 三路都断电 -> 不再监视（运动循环可退出）");
  check(!dog.stalled(t), "全断电后也不判堵转");
}

void test_multi_watchdog_done_then_resume() {
  section("多路看门狗：断电后又重新给占空比 -> 恢复监视");
  MultiHallWatchdog dog;
  bool will[3] = {true, false, false};
  dog.begin(0x01, will, 0.0);

  dog.feed_duty(1, 600, 0.02);
  dog.feed_hall(1, 0x1, 0.02);
  check(!dog.done(1), "1 号在动");

  dog.feed_duty(1, 0, 0.04);
  check(dog.done(1), "占空比归零 -> 标记完成");

  dog.feed_duty(1, 600, 0.06);
  check(!dog.done(1), "★ 占空比恢复 -> 撤销完成标记");
  check(dog.watching(1), "重新进入监视");
}

void test_multi_watchdog_no_move_motor_ignored() {
  section("多路看门狗：本来就不动的电机不触发堵转");
  MultiHallWatchdog dog;
  // 只选 M1；M2/M3 既未选中也不会动
  bool will[3] = {true, false, false};
  dog.begin(0x01, will, 0.0);

  const int seq[] = {0x1, 0x5, 0x4, 0x6, 0x2, 0x3};
  double t = 0.0;
  for (int i = 0; i < 40; ++i) {
    t += 0.02;
    dog.feed_duty(1, 600, t);
    dog.feed_hall(1, seq[i % 6], t);
    // M2/M3 完全没喂 —— 它们本来就不该动
  }
  check(!dog.stalled(t), "★ 未选中的路不参与堵转判定");
  check(!dog.selected(2) && !dog.selected(3), "M2/M3 未被选中");
  check(!dog.watching(2) && !dog.watching(3), "M2/M3 不在监视范围");
}

void test_multi_watchdog_invalid_codes() {
  section("多路看门狗：非法霍尔码不算进展");
  MultiHallWatchdog dog;
  bool will[3] = {true, false, false};
  dog.begin(0x01, will, 0.0);

  double t = 0.0;
  // 一直读到非法码 0x7 / 0x0，不能算作有进展
  const int bad[] = {0x7, 0x0, 0x7, 0x0};
  for (int i = 0; i < 40; ++i) {
    t += 0.02;
    dog.feed_duty(1, 600, t);
    dog.feed_hall(1, bad[i % 4], t);
    if (dog.stalled(t)) break;
  }
  check(dog.stalled_motor(t) == 1, "★ 非法码不产生有效边沿 -> 仍会判堵转");
  check(dog.edges(1) == 0, "非法码不增加边沿计数");
}

void test_multi_watchdog_same_code() {
  section("多路看门狗：同一霍尔码重复不算进展");
  MultiHallWatchdog dog;
  bool will[3] = {true, false, false};
  dog.begin(0x01, will, 0.0);

  double t = 0.0;
  for (int i = 0; i < 40; ++i) {
    t += 0.02;
    dog.feed_duty(1, 600, t);
    dog.feed_hall(1, 0x5, t);   // 一直是同一个合法码
    if (dog.stalled(t)) break;
  }
  check(dog.stalled_motor(t) == 1, "★ 同码重复 -> 判堵转");
  check(dog.edges(1) == 0, "同码不增加边沿计数");
}

}  // namespace

int main() {
  std::printf("==============================================================================\n");
  std::printf("  selftest_position —— test_positionctrl 安全逻辑离线验证\n");
  std::printf("  不碰硬件、不需要 CAN 通道\n");
  std::printf("==============================================================================\n");

  test_hall_validity();
  test_watchdog_healthy_motion();
  test_watchdog_stall_detection();
  test_watchdog_invalid_codes_not_progress();
  test_watchdog_same_code_not_progress();
  test_watchdog_faster_than_firmware();
  test_target_bounds_prezero();
  test_target_bounds_zeroed();
  test_target_repeated_moves_clamp();
  test_target_tiny_delta();
  test_target_epsilon_tolerance();
  test_saturation_detection();
  test_target_rejects_saturated();
  test_angle_state();
  test_move_budget();
  test_parse_motor_mask();
  test_split_move_masks();
  test_multi_motor_coordinates();
  test_move_budget_multi();
  test_multi_watchdog_healthy();
  test_multi_watchdog_stall();
  test_multi_watchdog_one_motor_done();
  test_multi_watchdog_done_then_resume();
  test_multi_watchdog_no_move_motor_ignored();
  test_multi_watchdog_invalid_codes();
  test_multi_watchdog_same_code();

  std::printf("\n==============================================================================\n");
  if (g_failures == 0) {
    std::printf("  \033[32m全部通过\033[0m\n");
  } else {
    std::printf("  \033[31m%d 项失败\033[0m\n", g_failures);
  }
  std::printf("==============================================================================\n");
  return g_failures == 0 ? 0 : 1;
}