/**
 * @file walk_control_V1.cpp
 * @brief 四腿（腿臂）机器人【下位机】控制程序：
 *        ① 接收上位机 10Hz 关节目标流（ID 1~16），拆分到 4 条 CAN 总线
 *           × 4 电机，由电机内部位置环 PD 执行；
 *        ② 关节级键盘命令（move/goto/pos/estop/resume），现场调试用。
 *
 * 【上位机 ⇄ 本机（下位机）协议】（协议全文见 docs/step_protocol.pdf）：
 *   物理层：CAN 2.0A 标准帧，1 Mbps。上位机经其自己的 USB-CAN 适配器接入
 *     本机一条【独立】SocketCAN 接口（启动时输入接口名，例如 can4/can5 或
 *     第二只适配器的 canN），不与电机总线 can0~can3 共用——若共用，上位机
 *     发的 ID 1~4 会被腿 1 的真电机直接执行，下位机无法区分帧的来源。
 *   帧结构与钛虎写命令帧【同构】：DLC=5，[功能码(1B)][int32 小端(4B)]；
 *   但 int32 语义重定义为「关节端角度(rad) × 10000」（1 LSB = 0.0001 rad
 *   ≈ 0.0057°，范围 ±214748 rad，远大于任何关节行程）。
 *
 *   上位机 → 本机（ID = 1~16，一帧一个关节）：
 *     [0x1E][int32 小端：目标关节端角 × 10000]（DLC=5）
 *       —— 单关节目标帧，10Hz 周期发送，收到即作为该关节最新目标；
 *     [0x02]（DLC=1）—— 退出流模式，电机保持当前目标；
 *     [0x03]（DLC=1）—— 清除软限位报警锁定（整机恢复接收目标）；
 *     [0x41]（DLC=1）—— 状态请求，本机立即回该 ID 的实际关节角。
 *   本机 → 上位机：
 *     ID 1~16（DLC=5）[0x41][int32 小端：实际关节端角 × 10000]
 *       —— 10Hz 周期回传（数据源 = 各总线 10Hz 轮询电机 0x41 三合一读）；
 *     ID 0x161（DLC=1）心跳，200ms 周期：bit0=流模式，bit1=样点在线，
 *       bit2=软限位报警锁定中；
 *     ID 0x162（DLC=8）软限位报警，200ms 周期（锁定期间持续重发，见下）。
 *
 *   ID 拆分映射（下位机核心职责）：上位机 ID = (腿号-1)×4 + 关节号，即
 *       ID 1~4   → 腿1（can0）电机 ID 1~4（J1~J4）
 *       ID 5~8   → 腿2（can1）电机 ID 1~4
 *       ID 9~12  → 腿3（can2）电机 ID 1~4
 *       ID 13~16 → 腿4（can3）电机 ID 1~4
 *     下位机解码后完成 关节端 → 电机输出端 换算（J2/J3 锥齿轮 1:2，×2），
 *     再按钛虎原生协议 [0x1E][int32 编码器 cnt] 经鲲弘 USB-CAN 下发真电机。
 *
 *   保护：目标越限 → 目标被【钳位到限位值】下发（关节走到限位并停住），
 *         并按实际位置判定是否真的到达限位；一旦到达即触发【软限位报警】：
 *           · 报警帧 ID 0x162（DLC=8）周期性上报（首次立即发，之后 200ms 重发）；
 *           · 整机锁定：所有关节目标冻结在锁定瞬间的值，不再接受新目标；
 *           · 解除方式：上位机发 [0x03]（DLC=1）专用清除命令；
 *         急停锁定期间丢弃目标；流模式 1s 无新样点自动退出（电机保持、不抱闸）；
 *         estop/quit 抱闸；move/goto 在流模式/急停锁定中暂缓或拒绝。
 *
 * 【软限位报警帧 0x162 格式】（DLC=8，标准帧）
 *   data[0] = 0x62            报警功能码（与 0x41 反馈帧区分）
 *   data[1] = 关节 ID 1~16    触发报警的关节（全局关节号，即上位机发目标用的 ID）
 *   data[2] = 方向/侧别       0x01 = 撞上限（max），0x02 = 撞下限（min）
 *   data[3] = 保留（0）
 *   data[4..7] = int32 小端   当前实际关节端角 × 10000（每次重发刷新），
 *                             这样上位机能看出关节是否还在运动
 *
 *   限位值本身不必上送：上位机按 ID 换算关节号后本地查表即得（见
 *   tools/up_walk_control_V1.py 的 JOINT_LIMIT_DEG）。
 *
 * 命令一览（运行后输入，help 可随时查看）：
 *   move <腿1-4> <J1-J4> <cw|ccw> <角度°> [时长s]   相对转动（例：move 1 J1 cw 90）
 *   goto <腿1-4> <J1-J4> <角度°> [时长s]            转到绝对关节角（例：goto 1 J2 45）
 *   pos  [腿1-4]                                     查询关节当前位置（读 10Hz 反馈缓存，不占总线）
 *   stat                                             流模式统计（各腿收帧/钳位/电机在线/软限位）
 *   estop                                            急停：全部电机立即抱闸并锁定
 *   resume                                           解除急停锁定（电机保持抱闸，等新目标）
 *   clear                                            清除软限位报警锁定（同上位机 [0x03]）
 *   quit                                             退出（全部电机抱闸后退出）
 *
 * ======================= 启动参数（可调项，--help 查看全部） =======================
 *
 * 不带任何参数运行 = 历史交互流程：依次询问「启用的腿」与「上位机 CAN 接口名」。
 * 带参数时跳过对应提问，便于脚本化启动（systemd / ROS2 launch）与现场试参。
 * 所有参数默认值等于历史硬编码值 ⇒ 改参数不影响协议，也不改变默认行为。
 *
 *   --legs "1 2 3 4"          启用的腿（all = 全部）
 *   --host <接口名>            上位机 CAN 接口（不可用 can0~can3）
 *   --no-host                 不启用上位机链路（仅键盘命令）
 *   --interactive             强制回到交互式提问
 *   --dry-run                 只校验参数并打印生效配置后退出（不碰 CAN，返回 0）
 *
 *   --limits "a,b;c,d;e,f;g,h"  J1~J4 软限位 {min,max}（关节端度）
 *   --trigger-deg <度>         限位触发阈值（默认 0.5，须大于 PD 稳态误差）
 *   --rearm-deg <度>           解除锁定后重新武装的回差（默认 2.0）
 *   --dir-sign "+1,+1,+1,+1"   关节方向符号（仅 ±1）
 *   --gear "101,162,162,101"   关节端总减速比（仅影响速度换算）
 *   --motor-scale "1,2,2,1"    电机输出端角 / 关节角
 *
 *   --kp / --kd <int>          位置环增益（默认 4000 / 60，写入电机固件）
 *   --stream-timeout-ms <ms>   断流超时（默认 1000）
 *   --heartbeat-ms <ms>        心跳 0x161 周期（默认 200）
 *   --alarm-ms <ms>            报警 0x162 重发周期（默认 200）
 *   --feedback-hz <hz>         反馈回传频率（默认 10，范围 1~100）
 *
 * 退出码：0 正常；-1 输入中断/交互失败；2 命令行参数非法。
 *
 * ============================= 单位与符号约定 =============================
 *
 * 本 demo 的角度一律按【关节端】计算（区别于 taihu_singleleg_step_xCAN 按
 * 电机输出端）。JointModule 的位置 API 是电机输出端弧度，换算在 demo 层完成：
 *
 *   电机输出端角 = 关节角 × kMotorRadPerJointRad × kJointDirSign
 *
 * 机器人构型：4 条腿，每腿 4 关节 YPPY（腿 i ↔ CAN 通道 can{i-1}）：
 *
 *   | 关节 | CAN ID | 构型 | 电机减速比 | 附加传动            | 关节端总减速比 |
 *   |------|--------|------|-----------|---------------------|----------------|
 *   | J1   | 1      | Y    | 101       | 直驱（无）          | 101            |
 *   | J2   | 2 (P1) | P    | 81        | 锥齿轮 1:2（关节减速）| 81×2 = 162   |
 *   | J3   | 3 (P2) | P    | 81        | 锥齿轮 1:2（关节减速）| 81×2 = 162   |
 *   | J4   | 4      | Y    | 101       | 直驱（无）          | 101            |
 *
 * 上表为**默认值**；减速比、传动系数、方向符号、软限位、KP/KD、通讯周期
 * 均可在启动时用命令行参数覆盖（`--help` 查看），不带参数时等于上表。
 * 符号约定：与 taihu_singleleg_step_xCAN 保持一致——电机输出端正角度 = 逆时针。
 * 因此本 demo 约定：关节端正角度 = 逆时针（ccw），cw = 负角度。
 * 依据：xCAN demo 的关键帧 {0, 180, 180, 0} 里 ID2/ID3 同为 +180°（两个 P 型锥齿轮
 * 在电机端同向），故 kJointDirSign = {+1, +1, +1, +1}。若现场发现某关节因锥齿轮
 * 装反而方向相反，把 kJointDirSign 对应元素改为 -1 即可（限位是关节端语义，不用改）。
 *
 * 软件限位（关节端，度）与电机端等效范围（× kMotorRadPerJointRad）：
 *   | 关节 | 构型         | 关节端限位      | 电机端等效       |
 *   |------|--------------|-----------------|------------------|
 *   | J1   | Y 直驱       | ±120°           | ±120°            |
 *   | J2   | P 锥齿轮 1:2 | ±120°           | ±240°            |
 *   | J3   | P 锥齿轮 1:2 | -30° ~ +180°    | -60° ~ +360°     |
 *   | J4   | Y 直驱       | ±90°            | ±90°             |
 *   判定一律按**关节端**进行。上位机目标帧越界时**钳位到限位值**后下发（关节
 *   走到限位并停住），并按实际位置判定是否真的到达限位；键盘 move/goto 的
 *   目标角越界时仍然直接拒绝（不下发任何运动指令），保持原有调试行为。
 *
 * 减速比说明：kJointGearRatios 只参与**速度**换算（JointModule::velToRadPerS /
 *   radPerSToVel）；位置换算是 radToCnt/cntToRad（262144 cnt = 2π，输出端），
 *   **不含减速比**，故 xCAN demo 的 {101,81,81,101} 与本文件的 {101,162,162,101}
 *   差异不影响位置/方向/限位。
 *
 * ============================= PD 控制 =============================
 *
 *   「PD 控制」由钛虎电机固件内部位置环完成：KP/KD（默认 4000/60）在初始化
 *   时经 0x2B/0x2D 下发；下位机把最新目标以 5ms（200Hz）周期重发 0x1E，
 *   上位机 10Hz 阶梯样点之间的过渡由电机内部 PD 平滑追踪，下位机不做软件
 *   插值（与 taihu_pd_leg / taihu_singleleg_step_xCAN 的纯 PD 模式一致）。
 *
 * ============================= 线程模型 =============================
 *
 *   SocketCanInterface 无内部锁、非线程安全 ⇒ 每条 CAN 接口由单一线程独占：
 *
 *   主线程：只做键盘输入与命令解析/路由（move/goto 入队到对应腿；estop/
 *     resume/quit 置标志；pos/stat 读原子缓存打印），绝不直接操作 CAN；
 *   4 个总线线程（busThread，每腿一个）：独占 can0~can3，负责该总线电机的
 *     全部收发——5ms 周期下发目标（0x1E）、10Hz 读实际位置（0x41）写反馈
 *     缓存、执行键盘 move/goto 五次多项式轨迹、急停抱闸；
 *   上位机线程（hostThread）：独占上位机接口，收 ID 1~16 目标帧 → 限位
 *     检查（越限钳位）→ 写目标缓存；10Hz 回传实际角；心跳（0x161，200ms，
 *     bit2 上报软限位锁定）；软限位报警帧（0x162）边沿触发 + 200ms 重发；
 *     按需 [0x41] 立即回复；断流 1s 保护。
 *
 *   线程间只通过无锁原子缓存传递数据（最新值语义，天然适配 10Hz 覆盖式流）：
 *     g_target_rad[腿][关节]  host 线程/总线线程写 → 总线线程读（下发源）
 *     g_fb_rad[腿][关节]      总线线程写 → host 线程/主线程读（回传与 pos 源）
 *     g_limit_locked 等       总线线程写 → host 线程/主线程读（报警源与部位）
 *
 * 命令执行期间其它关节保持当前目标（每 tick 重发），到位后不抱闸、
 * 由电机内部位置环持续保持（腿不塌）；estop / quit 才发 0x02 抱闸。
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "taihu/can_interface.h"
#include "taihu/config.h"
#include "taihu/joint_module.h"
#include "taihu/socket_can.h"
#include "taihu/taihu_tools.h"

namespace
{

    // config.h 的公共常量引入本命名空间（kLoopPeriodMs / kSettleThresholdRad /
    // kSettleTimeoutMs / kPollPeriodMs / kDegToRad）
    using taihu::kDegToRad;
    using taihu::kLoopPeriodMs;
    using taihu::kPollPeriodMs;
    using taihu::kSettleThresholdRad;
    using taihu::kSettleTimeoutMs;

    // =====================================================================
    // 机器人构型常量（4 腿 YPPY）
    // =====================================================================

    constexpr int kLegCount = 4;   ///< 腿数（腿 1~4 ↔ can0~can3）
    constexpr int kJointCount = 4; ///< 每腿关节数（J1~J4 ↔ 电机 ID 1~4）

    /// 四个电机的 CAN ID（J1~J4 顺序）
    const uint32_t kMotorIds[kJointCount] = {1, 2, 3, 4};

    // =====================================================================
    // 【运行时可变参数】—— 下列变量在启动参数解析阶段写入一次，
    // 解析结束后即视为只读；默认值与历史版本逐项一致，不带参数运行时
    // 行为与旧版本完全相同。
    //
    // 可用命令行参数覆盖（见 --help）：
    //   --legs / --host / --limits / --dir-sign / --gear / --motor-scale
    //   --kp / --kd / --trigger-deg / --rearm-deg / --stream-timeout-ms
    //   --heartbeat-ms / --alarm-ms / --feedback-hz
    // =====================================================================

    /// 关节端总减速比（速度换算用：转子 → 关节）
    ///   J1/J4 直驱 = 电机减速比 101；J2/J3 = 电机 81 × 锥齿轮 2 = 162
    /// 可用 --gear 覆盖。注意：只影响速度换算，不影响位置/方向/限位。
    double kJointGearRatios[kJointCount] = {101.0, 162.0, 162.0, 101.0};

    /// 位置换算系数：电机输出端角 = 关节角 × 该系数
    ///   Y 直驱 = 1；P 锥齿轮 1:2（电机端转 2、关节转 1）= 2
    /// 可用 --motor-scale 覆盖（仅现场传动比改动时才需要）。
    double kMotorRadPerJointRad[kJointCount] = {1.0, 2.0, 2.0, 1.0};

    /// 关节方向符号（+1 = 关节端正方向对应电机端正方向）。
    ///
    /// 符号基准 = taihu_singleleg_step_xCAN（现场已验证方向正确）：该 demo 直接把
    /// 关键帧角度按【电机输出端】下发，且注明「逆时针为正」，其关键帧
    ///   {0.0, 180.0, 180.0, 0.0}
    /// 中 ID2/ID3 同为 **+180°**，说明两个 P 型锥齿轮（J2/J3）在电机端同向，
    /// 故关节端与电机端同号，四个关节均取 +1。
    /// 若现场发现某关节因锥齿轮装反而方向相反，只改对应元素为 -1 即可（限位是
    /// 关节端语义，改符号后限位表不需要跟着改）。可用 --dir-sign 覆盖，
    /// 例如 --dir-sign +1,-1,+1,+1。
    double kJointDirSign[kJointCount] = {1.0, 1.0, 1.0, 1.0};

    /// 软件限位（**关节端**，度）：{min, max}。判定与钳位均按关节端角进行。
    ///
    /// 电机端等效范围 = 关节端限位 × kMotorRadPerJointRad（J2/J3 锥齿轮 1:2 为 ×2）：
    ///   J1 直驱      ：关节端 ±120°       ↔ 电机端 ±120°
    ///   J2 锥齿轮 1:2：关节端 ±120°       ↔ 电机端 ±240°
    ///   J3 锥齿轮 1:2：关节端 −30°~+180°  ↔ 电机端 −60°~+360°
    ///   J4 直驱      ：关节端 ±90°        ↔ 电机端 ±90°
    /// 可用 --limits 覆盖，格式：min1,max1;min2,max2;min3,max3;min4,max4
    double kJointLimitDeg[kJointCount][2] = {
        {-120.0, 120.0}, ///< J1（Y 直驱，电机端 ±120°）
        {-120.0, 120.0}, ///< J2（P1 锥齿轮 1:2，CAN ID 2，电机端 ±240°）
        {-30.0, 180.0},  ///< J3（P2 锥齿轮 1:2，CAN ID 3，电机端 −60°~+360°）
        {-90.0, 90.0},   ///< J4（Y 直驱，电机端 ±90°）
    };

    // —— 位置环增益（与 taihu_singleleg_step_xCAN 一致） ——
    //    可用 --kp / --kd 覆盖。注意：这是写进电机固件的参数，改动后
    //    上电初始化时生效。
    int32_t kPositionKp = 4000;
    int32_t kPositionKd = 60;

    // =====================================================================
    // 上位机 ⇄ 下位机 通讯协议常量（协议全文见 docs/step_protocol.pdf）
    //
    // 设计原则：帧结构与钛虎电机写命令帧【同构】——DLC=5，[功能码 1B]
    // [int32 小端 4B]，上位机侧的收发代码可以完全复用控制真电机的写法；
    // 只是把 int32 的语义从「编码器 cnt」重定义为「关节端角度 × 10000」，
    // 把帧 ID 从「电机 ID」重定义为「全局关节 ID 1~16」。
    //
    //   上位机 → 本机：功能码 0x1E（目标关节端角，int32 = rad × 10000）
    //                  功能码 0x02（退出流模式，1 字节）
    //                  功能码 0x41（状态请求 → 本机回实际关节角）
    //   本机 → 上位机：功能码 0x41 回复（实际关节端角，int32 = rad × 10000）
    //                  心跳 0x161（1 字节状态位，200ms 周期）
    //
    // 帧 ID 映射（标准帧）：ID 1~16 ↔ 全局关节号（(腿-1)×4 + 关节号）
    // =====================================================================

    /// 全局关节 ID 基址：ID 1~16 = 腿1 J1 ~ 腿4 J4
    constexpr uint16_t kJointIdBase = 1;
    /// 全局关节数（= 腿数 × 每腿关节数）
    constexpr int kTotalJoints = kLegCount * kJointCount;
    /// 全局关节 ID → 腿下标 0~3
    inline int idToLeg(uint16_t id) { return (id - kJointIdBase) / kJointCount; }
    /// 全局关节 ID → 关节下标 0~3
    inline int idToJoint(uint16_t id) { return (id - kJointIdBase) % kJointCount; }
    /// (腿下标, 关节下标) → 全局关节 ID
    inline uint16_t jointToId(int leg_idx, int joint_idx)
    {
        return kJointIdBase + static_cast<uint16_t>(leg_idx * kJointCount + joint_idx);
    }
    /// 是否为合法的全局关节 ID（1~16）
    inline bool isJointId(uint16_t id)
    {
        return id >= kJointIdBase && id < kJointIdBase + kTotalJoints;
    }

    /// 角度定点缩放：协议 int32 = 关节端角度(rad) × 10000
    /// （1 LSB = 0.0001 rad ≈ 0.0057°，int32 范围 ±214748 rad）
    constexpr double kAngleScale = 10000.0;

    /// 心跳帧 ID（0x161，DLC=1 状态字节；避开关节 ID 1~16）
    constexpr uint16_t kHeartbeatId = 0x161;
    /// 心跳发送周期（ms）——可用 --heartbeat-ms 覆盖
    int kHeartbeatMs = 200;

    /// 上位机发送频率（Hz，协议约定，用于断流判定与回传节流）
    constexpr int kHostStreamHz = 10;
    /// 反馈回传周期（ms）= 1000/kHostStreamHz；实际角由总线线程 10Hz 轮询电机
    /// 得到，回传同频即可保证每帧都是新鲜值（200Hz 回传只会重复缓存，徒增带宽）
    /// 可用 --feedback-hz 覆盖（合法范围 1~100Hz）。
    int kFeedbackMs = 1000 / kHostStreamHz;
    /// 总线线程读电机实际位置（0x41 三合一）的周期（ms，10Hz）
    int kReadPeriodMs = 100;
    /// 断流保护：流模式下超过此时长无新样点 → 自动退出流模式（电机保持）
    /// 可用 --stream-timeout-ms 覆盖。
    int kStreamTimeoutMs = 1000;

    // —— 协议功能码（与钛虎电机协议同码，帧结构同构） ——
    constexpr uint8_t kHostCmdSetPos = 0x1E;     ///< 目标位置（int32 = rad × 10000）
    constexpr uint8_t kHostCmdStop = 0x02;       ///< 退出流模式（电机保持）
    constexpr uint8_t kHostCmdClearLimit = 0x03; ///< 清除软限位报警锁定
    constexpr uint8_t kHostCmdGetSt = 0x41;      ///< 状态请求 / 状态回复回显

    // =====================================================================
    // 软限位报警（本机 → 上位机）
    //
    // 触发条件：实际关节端角到达软限位（按 g_fb_rad 判定，不是按目标判定）——
    //   只有关节**真的**运动到限位才报警，上位机发出的越限目标本身不直接报警
    //   （它会被钳位到限位值后下发，关节到位后才触发）。
    // 触发后：整机锁定（所有关节目标冻结），并每 200ms 重发报警帧，直到
    //   上位机发 [0x03] 清除。
    // =====================================================================

    /// 软限位报警帧 ID（0x162，紧跟心跳 0x161；避开关节 ID 1~16 与 0x161）
    constexpr uint16_t kLimitAlarmId = 0x162;
    /// 报警帧功能码（data[0]），与 0x41 反馈帧区分
    constexpr uint8_t kLimitAlarmCode = 0x62;
    /// 报警帧中「撞哪一侧」的取值（data[2]）
    constexpr uint8_t kLimitSideMax = 0x01; ///< 撞上限（kJointLimitDeg[j][1]）
    constexpr uint8_t kLimitSideMin = 0x02; ///< 撞下限（kJointLimitDeg[j][0]）
    /// 报警帧重发周期（ms）——与心跳同频，锁定期间持续上报
    /// 可用 --alarm-ms 覆盖。
    int kLimitAlarmMs = 200;

    /// 限位触发阈值（度）：实际角进入「限位 - 阈值 ~ 限位」区间即判定到达。
    /// 取值需大于电机 PD 稳态误差（否则永远报不出来），又要足够小以免误报。
    /// 可用 --trigger-deg 覆盖。
    double kLimitTriggerDeg = 0.5;
    /// 限位解除回差（度）：实际角退出「限位 - 回差」以外才认为离开限位。
    /// 仅在需要「位置回退自动清除」时使用；当前策略是锁定需上位机 [0x03]，
    /// 此常量用于锁定解除后重新武装（re-arm）的判定，防止抖动反复触发。
    /// 可用 --rearm-deg 覆盖。
    double kLimitRearmDeg = 2.0;

    // —— 轨迹与执行参数 ——
    constexpr double kDefaultMoveSec = 2.0; ///< 默认轨迹时长 (s)
    constexpr double kMinMoveSec = 0.1;     ///< 最短轨迹时长 (s)
    constexpr double kMaxMoveSec = 60.0;    ///< 最长轨迹时长 (s)
    constexpr double kMaxAngleDeg = 360.0;  ///< 单命令最大角度 (°)

    /// 到位确认阈值（关节端 rad）
    constexpr double kSettleRad = taihu::kSettleThresholdRad;
    /// 总线线程空闲轮询周期（也决定空闲时 estop 的响应上限）(ms)
    constexpr int kIdlePollMs = 50;

    const char *kJointNames[kJointCount] = {"J1", "J2", "J3", "J4"};

    // =====================================================================
    // 单位换算（关节端 ⇄ 电机输出端）
    // =====================================================================

    /// 关节端弧度 → 电机输出端弧度
    double jointToMotorRad(int joint, double joint_rad)
    {
        return joint_rad * kMotorRadPerJointRad[joint] * kJointDirSign[joint];
    }

    /// 电机输出端弧度 → 关节端弧度
    double motorToJointRad(int joint, double motor_rad)
    {
        return motor_rad / (kMotorRadPerJointRad[joint] * kJointDirSign[joint]);
    }

    /// 五次多项式平滑函数（quintic smoothstep）：s(0)=0、s(1)=1，
    /// 一阶、二阶导数在两端均为 0 —— 位置/速度/加速度连续，起停无冲击。
    double quinticSmoothstep(double t)
    {
        return 6.0 * t * t * t * t * t - 15.0 * t * t * t * t + 10.0 * t * t * t;
    }

    // =====================================================================
    // 上位机 ⇄ 下位机 CAN 帧编解码（与钛虎写命令帧同构：小端 int32 定点）
    // =====================================================================

    /// 32 位小端写（d[0] 为最低字节，与钛虎 sendWriteCmd 数据域一致）。
    inline void put_i32le(uint8_t *d, int32_t v)
    {
        d[0] = static_cast<uint8_t>(v & 0xFF);
        d[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
        d[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
        d[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
    }

    /// 32 位小端读（符号扩展）。
    inline int32_t get_i32le(const uint8_t *d)
    {
        return static_cast<int32_t>(static_cast<uint32_t>(d[0]) | (static_cast<uint32_t>(d[1]) << 8) | (static_cast<uint32_t>(d[2]) << 16) | (static_cast<uint32_t>(d[3]) << 24));
    }

    /// 关节端角度(rad) → 协议定点值（× kAngleScale，四舍五入并饱和到 int32）。
    inline int32_t radToProto(double rad)
    {
        const double v = rad * kAngleScale;
        if (v > 2147483647.0)
            return 2147483647;
        if (v < -2147483648.0)
            return -2147483647 - 1;
        return static_cast<int32_t>(std::lround(v));
    }

    /// 协议定点值 → 关节端角度(rad)。
    inline double protoToRad(int32_t v)
    {
        return static_cast<double>(v) / kAngleScale;
    }

    // =====================================================================
    // 键盘命令（主线程解析 → 路由到对应腿的队列 → 该腿 busThread 执行）
    // =====================================================================

    /// 一条待执行的命令
    struct Command
    {
        enum class Type
        {
            Move,
            Goto
        } type;
        int leg = 0;                         ///< 腿号 1~4
        int joint = 0;                       ///< 关节下标 0~3
        bool relative = true;                ///< Move=true（相对），Goto=false（绝对）
        double angle_deg = 0.0;              ///< 角度（相对命令已含符号：cw 为负 / ccw 为正）
        double duration_s = kDefaultMoveSec; ///< 轨迹时长
    };

    // =====================================================================
    // 一条腿：CAN 接口 + 四个电机 + 该腿的线程间共享状态
    // =====================================================================

    struct Leg
    {
        int index = 0;                    ///< 腿号 1~4
        std::string ifname;               ///< CAN 接口名（"can0"~"can3"）
        std::atomic<bool> enabled{false}; ///< 是否成功打开并初始化（主线程启动前写，
                                          ///  之后只读；用 atomic 避免与线程启动的数据竞争）

        taihu::SocketCanInterface can;                           ///< 该腿独立总线（busThread 独占）
        std::unique_ptr<taihu::JointModule> motors[kJointCount]; ///< J1~J4

        // —— 使能（解抱闸）状态。协议无独立使能命令：去使能(0x02)=抱闸、
        //    使能=解抱闸，setTargetCurrent(0) 即「使能+零转矩」。上电默认
        //    去使能，故必须显式使能后 0x1E 位置帧才会被执行。
        //    armed 由 busThread 写（initLeg 在起线程前写），arm_req 由主线程
        //    置位（arm 命令），busThread 消费——主线程绝不直接碰 CAN。
        std::atomic<bool> armed{false};   ///< 四个电机是否已使能（解抱闸）
        std::atomic<bool> arm_req{false}; ///< 主线程请求使能（busThread 消费）

        // —— 目标缓存（关节端 rad）：host 线程写（流模式）/ busThread 写
        //    （move/goto 与保持），busThread 读（5ms 下发源）。
        //    单写单读、最新值语义，无锁原子即可（对齐 double 在 x86-64 上原子）。
        std::atomic<double> target_rad[kJointCount];
        std::atomic<bool> have_target{false}; ///< 是否已有目标（未收到前不下发）

        // —— 反馈缓存（关节端 rad）：busThread 10Hz 写，host 线程（回传）与
        //    主线程（pos/stat 打印）读。
        std::atomic<double> fb_rad[kJointCount];
        std::atomic<bool> motor_online[kJointCount]; ///< 各关节最近一次 0x41 读成功

        // —— 该腿的键盘命令队列（主线程入队，busThread 出队执行）——
        std::queue<Command> cmd_queue;
        std::mutex cmd_mutex;
        std::condition_variable cmd_cv;

        // —— 流模式统计（host 线程写，主线程 stat 读）——
        std::atomic<uint32_t> rx_frames{0}; ///< 该腿累计收到的 0x1E 目标帧数
        /// 该腿累计「未按原值执行」的帧数：越限被钳位 + 急停/限位锁定期间丢弃。
        /// 注意：越限帧是【钳位后照常下发】，并非丢弃，此计数含钳位事件。
        std::atomic<uint32_t> dropped{0};
    };

    // =====================================================================
    // 全局状态（主线程 / 4 个总线线程 / host 线程共享）
    // =====================================================================

    Leg g_legs[kLegCount]; ///< 四条腿（下标 = 腿号-1）

    std::atomic<bool> g_running{true};       ///< 所有线程运行标志
    std::atomic<bool> g_estop{false};        ///< 急停请求标志（总线线程每 tick 检查，处理后清除）
    std::atomic<bool> g_estop_locked{false}; ///< 急停锁定：抱闸后拒绝一切运动目标，直到 resume

    // —— 流模式状态（host 线程写，总线线程/主线程读）——
    std::atomic<bool> g_stream_armed{false};     ///< 流模式激活（收到首帧 0x1E 即置位）
    std::atomic<bool> g_stream_online{false};    ///< 最近 1s 内收到过样点（供心跳上报）
    std::atomic<int64_t> g_stream_last_rx_ms{0}; ///< 最近一次收到样点的时刻（ms，单调钟）
    std::atomic<int64_t> g_host_started_ms{0};   ///< host 链路打开时刻（回传节流起点）

    // —— 软限位报警状态 ——
    /// 报警锁定：有任一关节到达限位并报警后置位。置位期间所有关节目标冻结
    /// （不再接受上位机新目标、也不接受新的 move/goto），直到 clearLimitAlarm()
    /// 被上位机 [0x03] 或键盘 clear 命令解除。
    std::atomic<bool> g_limit_locked{false};
    /// 触发报警的关节（全局关节 ID 1~16）；0 = 尚未报警
    std::atomic<uint16_t> g_limit_alarm_id{0};
    /// 触发方向：kLimitSideMax / kLimitSideMin；0 = 尚未报警
    std::atomic<uint8_t> g_limit_alarm_side{0};
    /// 报警首次触发时刻（ms）——用于理解锁定持续时间
    std::atomic<int64_t> g_limit_alarm_ms{0};
    /// 累计报警次数（stat 显示用）
    std::atomic<uint32_t> g_limit_alarm_count{0};

    // =====================================================================
    // 总线线程原语（一条腿的全部 CAN 收发集中在该腿的 busThread）
    // =====================================================================

    /// 单调毫秒时间戳（steady_clock，跨线程共享时刻用）。
    inline int64_t nowMs()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    /// 读取一条腿四个关节的当前角度（关节端 rad，0x41 三合一读）。
    /// 单关节读失败不中断：该关节标记离线、沿用上次反馈值，其余关节照常。
    /// @return 全部读取成功返回 true；任一失败返回 false（电机离线/总线异常）。
    bool readLegJointRad(Leg &leg, double out[kJointCount])
    {
        bool all_ok = true;
        for (int j = 0; j < kJointCount; ++j)
        {
            int32_t cur_ma = 0;
            double vel = 0.0, motor_pos = 0.0;
            if (leg.motors[j]->readCurrentVelocityPosition(cur_ma, vel, motor_pos))
            {
                out[j] = motorToJointRad(j, motor_pos);
                leg.motor_online[j].store(true, std::memory_order_relaxed);
            }
            else
            {
                out[j] = leg.fb_rad[j].load(std::memory_order_relaxed); // 沿用旧值
                leg.motor_online[j].store(false, std::memory_order_relaxed);
                all_ok = false;
            }
        }
        return all_ok;
    }

    /// 对一条腿下发四个关节目标位置（关节端 rad → 电机输出端 → 0x1E cnt）。
    void sendLegTargets(const Leg &leg, const double joint_rad[kJointCount])
    {
        for (int j = 0; j < kJointCount; ++j)
            leg.motors[j]->setTargetPosition(jointToMotorRad(j, joint_rad[j]));
    }

    /// 使能一条腿（解除抱闸）。必须在 busThread 内调用（独占该腿 CAN）。
    ///
    /// 顺序有讲究：
    ///   1. 读一次实测位置，写入目标缓存与反馈缓存 —— 使能后电机立即以
    ///      当前位置为目标闭环保持，不会因目标缓存里的旧值（可能是 0）
    ///      而突然甩向零位；
    ///   2. 逐电机 setTargetCurrent(0)：进入电流模式（使能 + 零转矩），
    ///      解除抱闸。这是协议里唯一的使能途径；
    ///   3. 再次下发当前位置为目标：切回位置模式，由电机内部 PD 闭环托住
    ///      该姿态（腿不塌）。
    ///
    /// 注意：本函数会在不阻塞的前提下尽快返回，逐电机各一次写帧的代价可
    /// 忽略（读位置的往返已由 readLegJointRad 完成）。
    void enableLeg(Leg &leg)
    {
        // 1. 抓当前姿态作为保持目标（离线电机沿用上次反馈值）
        double cur[kJointCount];
        readLegJointRad(leg, cur);
        for (int j = 0; j < kJointCount; ++j)
        {
            leg.fb_rad[j].store(cur[j], std::memory_order_relaxed);
            leg.target_rad[j].store(cur[j], std::memory_order_relaxed);
        }
        leg.have_target.store(true, std::memory_order_relaxed);

        // 2. 使能（解抱闸）：电流 0 = 使能 + 零转矩
        int ok_count = 0;
        for (int j = 0; j < kJointCount; ++j)
        {
            leg.motors[j]->clearError(); // 清错误位（欠压等锁存错误会挡住使能）
            if (leg.motors[j]->setTargetCurrent(0))
                ++ok_count;
        }

        // 3. 立刻切回位置模式，以刚抓到的姿态为目标闭环保持
        sendLegTargets(leg, cur);

        leg.armed.store(true, std::memory_order_relaxed);
        std::printf("[腿%d][使能] 抱闸已解除（%d/4 电机应答），当前姿态已作为保持目标\n",
                    leg.index, ok_count);
        for (int j = 0; j < kJointCount; ++j)
            std::printf("          %s %+7.2f°\n", kJointNames[j],
                        cur[j] / kDegToRad);
    }

    /// 软件限位检查（关节端度）。
    bool withinLimits(int joint, double joint_deg)
    {
        return joint_deg >= kJointLimitDeg[joint][0] - 1e-9 &&
               joint_deg <= kJointLimitDeg[joint][1] + 1e-9;
    }

    /// 把关节端角度钳位到该关节的软限位区间内（关节端度），返回钳位后的值。
    /// 用于上位机越限目标：不再丢弃，而是钳位到限位值下发，让关节走到限位
    /// 并停住（随后由实际位置判定触发报警）。
    double clampToLimits(int joint, double joint_deg)
    {
        if (joint_deg < kJointLimitDeg[joint][0])
            return kJointLimitDeg[joint][0];
        if (joint_deg > kJointLimitDeg[joint][1])
            return kJointLimitDeg[joint][1];
        return joint_deg;
    }

    /// 判定实际关节角是否已到达软限位（关节端度）。
    /// 到达返回 kLimitSideMax / kLimitSideMin；未到达返回 0。
    /// 阈值 kLimitTriggerDeg 略向内收缩，避免电机 PD 稳态误差导致永远不触发。
    uint8_t detectLimitSide(int joint, double joint_deg)
    {
        if (joint_deg >= kJointLimitDeg[joint][1] - kLimitTriggerDeg)
            return kLimitSideMax;
        if (joint_deg <= kJointLimitDeg[joint][0] + kLimitTriggerDeg)
            return kLimitSideMin;
        return 0;
    }

    /// 触发软限位报警：整机锁定 + 记录报警源（供 host 线程上报）。
    /// 只在首次触发时打印日志；已在锁定中则忽略（保留首个报警源）。
    /// 由 busThread 在实际位置到达限位时调用。
    void triggerLimitAlarm(int leg_idx, int joint_idx, uint8_t side, double joint_deg)
    {
        if (g_limit_locked.load(std::memory_order_relaxed))
            return; // 已锁定：保留首次报警源

        const uint16_t id = jointToId(leg_idx, joint_idx);
        g_limit_alarm_id.store(id, std::memory_order_relaxed);
        g_limit_alarm_side.store(side, std::memory_order_relaxed);
        g_limit_alarm_ms.store(nowMs(), std::memory_order_relaxed);
        g_limit_alarm_count.fetch_add(1, std::memory_order_relaxed);
        g_limit_locked.store(true, std::memory_order_relaxed);

        const char *side_txt = (side == kLimitSideMax) ? "上限" : "下限";
        const int lim_idx = (side == kLimitSideMax) ? 1 : 0;
        std::printf("\n[软限位][报警] 腿%d %s（全局 ID %u）到达%s %.1f°"
                    "（实际 %.2f°）→ 整机锁定，等待上位机 [0x03] 清除\n",
                    leg_idx + 1, kJointNames[joint_idx], id, side_txt,
                    kJointLimitDeg[joint_idx][lim_idx], joint_deg);
    }

    /// 清除软限位报警锁定（上位机 [0x03] 或键盘 clear 调用）。
    /// 只清锁定标志与报警源；各关节保持当前目标（不抱闸、不突变）。
    void clearLimitAlarm()
    {
        if (!g_limit_locked.exchange(false, std::memory_order_relaxed))
            return;
        const uint16_t id = g_limit_alarm_id.load(std::memory_order_relaxed);
        g_limit_alarm_id.store(0, std::memory_order_relaxed);
        g_limit_alarm_side.store(0, std::memory_order_relaxed);
        std::printf("[软限位] 报警锁定已清除（原报警关节 ID %u），"
                    "恢复接收目标（各关节保持当前目标）\n",
                    id);
    }

    /// 执行一条运动命令（Move / Goto）：五次多项式轨迹逐点下发 + 到位确认。
    /// 在本腿 busThread 内执行；执行期间 g_estop 可随时中止，g_stream_armed
    /// 置位（上位机流开始）时立即让位（轨迹中止，控制权交还流模式）。
    /// @return 成功到位返回 true；拒绝/中止/失败返回 false。
    bool executeMove(Leg &leg, const Command &cmd)
    {
        const int j = cmd.joint;

        // 1. 读取当前四个关节角度（同时校验电机在线）
        double cur[kJointCount];
        if (!readLegJointRad(leg, cur))
        {
            std::printf("[腿%d][%s][错误] 读取状态失败（电机无响应？），命令放弃\n",
                        leg.index, kJointNames[j]);
            return false;
        }
        for (int k = 0; k < kJointCount; ++k)
            leg.fb_rad[k].store(cur[k]);

        // 2. 计算目标关节角（弧度）
        const double cur_deg = cur[j] / kDegToRad;
        double target_deg;
        if (cmd.relative)
            target_deg = cur_deg + cmd.angle_deg;
        else
            target_deg = cmd.angle_deg;

        // 3. 软件限位检查：越界直接拒绝，不下发任何运动指令
        if (!withinLimits(j, target_deg))
        {
            std::printf("[腿%d][%s][拒绝] 目标 %.1f° 超出限位 [%.0f°, %.0f°]（当前 %.1f°）\n",
                        leg.index, kJointNames[j], target_deg,
                        kJointLimitDeg[j][0], kJointLimitDeg[j][1], cur_deg);
            return false;
        }

        // 4. 轨迹：指定关节 从当前 → 目标，其余关节保持当前位置
        const double target_rad = target_deg * kDegToRad;
        const double q0[kJointCount] = {cur[0], cur[1], cur[2], cur[3]};

        std::printf("[腿%d][%s] %s %.1f° → 目标 %.1f°（限位 [%.0f°, %.0f°]），"
                    "轨迹 %.1fs，其余关节保持\n",
                    leg.index, kJointNames[j],
                    cmd.relative ? "相对转动" : "绝对转角",
                    cmd.angle_deg, target_deg,
                    kJointLimitDeg[j][0], kJointLimitDeg[j][1],
                    cmd.duration_s);

        // 5. 五次多项式轨迹逐点下发（kLoopPeriodMs 周期）
        const auto t0 = std::chrono::steady_clock::now();
        bool aborted = false;
        while (g_running.load())
        {
            const double t = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - t0)
                                 .count();
            const double s = quinticSmoothstep(std::min(t / cmd.duration_s, 1.0));

            double q[kJointCount];
            for (int k = 0; k < kJointCount; ++k)
                q[k] = q0[k] + (k == j ? (target_rad - q0[j]) * s : 0.0);
            sendLegTargets(leg, q);
            for (int k = 0; k < kJointCount; ++k)
                leg.target_rad[k].store(q[k]);

            if (t >= cmd.duration_s)
                break;

            // 急停 / 上位机流开始：立即让位（抱闸在 busThread 主循环统一处理）
            if (g_estop.load() || g_estop_locked.load())
            {
                std::printf("[腿%d][%s][中止] 收到急停，轨迹中止\n",
                            leg.index, kJointNames[j]);
                return false;
            }
            if (g_stream_armed.load())
            {
                std::printf("[腿%d][%s][让位] 上位机流开始，手动轨迹中止\n",
                            leg.index, kJointNames[j]);
                aborted = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
        }
        if (aborted)
            return false;

        // 6. 到位确认：轮询 0x41，误差 < kSettleRad（关节端）即成功；
        //    轮询间隙同样检查急停/流让位（读本身也刷新反馈缓存）
        const auto settle_start = std::chrono::steady_clock::now();
        while (g_running.load() && !g_estop.load() && !g_estop_locked.load() && !g_stream_armed.load())
        {
            double now_q[kJointCount];
            readLegJointRad(leg, now_q);
            for (int k = 0; k < kJointCount; ++k)
                leg.fb_rad[k].store(now_q[k]);
            if (std::fabs(now_q[j] - target_rad) < kSettleRad)
            {
                std::printf("[腿%d][%s] 到位确认：目标 %.3f rad，实际 %.3f rad，"
                            "误差 %.3f rad ✓\n",
                            leg.index, kJointNames[j], target_rad, now_q[j],
                            std::fabs(now_q[j] - target_rad));
                leg.have_target.store(true);
                return true;
            }
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now() - settle_start)
                                        .count();
            if (elapsed_ms > kSettleTimeoutMs)
            {
                std::printf("[腿%d][%s][警告] 到位超时（目标 %.3f rad），电机保持位置环\n",
                            leg.index, kJointNames[j], target_rad);
                leg.have_target.store(true);
                return false;
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(kPollPeriodMs)));
        }
        leg.have_target.store(true);
        return false;
    }

    /// 总线线程：独占一条电机总线（can0~can3），负责该腿 4 个电机的全部收发。
    ///
    /// 循环体（5ms 周期）：
    ///   1. 急停请求 → 4 电机 0x02 抱闸，置急停锁定，清请求标志；
    ///   2. 流模式/保持：把目标缓存（host 线程写入的最新样点，或上次手动轨迹
    ///      终点）经 关节端→电机端 换算后 0x1E 重发——电机内部位置环 PD 平滑
    ///      追踪 10Hz 阶梯目标（纯 PD，无软件插值）；
    ///   3. 非流模式且无手动命令时：50ms 超时等待本腿命令队列（move/goto）；
    ///   4. 每 100ms（10Hz）读一轮 0x41 刷新反馈缓存 g_fb_rad（host 回传与
    ///      pos 命令的数据源）。读是阻塞往返，会占用本 tick 的时间片——10Hz
    ///      下对 200Hz 下发节拍影响可忽略（读一轮 4 电机在线时约几 ms）。
    void busThread(Leg &leg)
    {
        int64_t last_read_ms = 0;
        int offline_streak[kJointCount] = {0};                 ///< 各关节连续读失败次数（退避用）
        int64_t backoff_until_ms[kJointCount] = {0};           ///< 退避截止时刻（跳过读取）
        bool rearm_ok[kJointCount] = {true, true, true, true}; ///< 各关节已退离限位
                                                               ///< （防空转复发）

        while (g_running.load())
        {
            // 1. 急停（优先级最高）：抱闸 + 锁定（锁定期间一切目标不下发）
            if (g_estop.load())
            {
                for (int j = 0; j < kJointCount; ++j)
                    leg.motors[j]->stop();
                leg.armed.store(false, std::memory_order_relaxed); // 0x02 = 去使能（抱闸）
                g_estop_locked.store(true);
                g_estop.store(false);
                std::printf("[腿%d][急停] 4 电机已抱闸（0x02），锁定中（resume 解除）\n",
                            leg.index);
                // 清空积压的键盘命令（锁定期间一律拒绝）
                std::lock_guard<std::mutex> lock(leg.cmd_mutex);
                while (!leg.cmd_queue.empty())
                    leg.cmd_queue.pop();
            }
            else if (g_estop_locked.load())
            {
                // 锁定期间：不下发任何目标（电机已抱闸），只维持反馈读取与
                // 命令拒绝，等待 resume
                std::lock_guard<std::mutex> lock(leg.cmd_mutex);
                while (!leg.cmd_queue.empty())
                {
                    leg.cmd_queue.pop();
                    std::printf("[腿%d][拒绝] 急停锁定中，move/goto 无效（resume 解除）\n",
                                leg.index);
                }
            }
            // 2. 使能请求（解抱闸；优先级次于急停，高于流模式/move）
            else if (leg.arm_req.exchange(false))
            {
                enableLeg(leg);
            }
            // 3. 流模式：5ms 周期重发最新目标（电机内部位置环 PD 追踪）
            else if (g_stream_armed.load())
            {
                if (leg.have_target.load(std::memory_order_relaxed))
                {
                    double q[kJointCount];
                    for (int j = 0; j < kJointCount; ++j)
                        q[j] = leg.target_rad[j].load(std::memory_order_relaxed);
                    sendLegTargets(leg, q);
                }
                // 流模式中 move/goto 暂缓：出队提示但不执行（pos/stat 由主线程
                // 直接读缓存，不经过本队列）
                {
                    std::lock_guard<std::mutex> lock(leg.cmd_mutex);
                    while (!leg.cmd_queue.empty())
                    {
                        leg.cmd_queue.pop();
                        std::printf("[腿%d][流] 流模式进行中，move/goto 暂缓"
                                    "（上位机停发 1s 或发 0x02 退出流模式）\n",
                                    leg.index);
                    }
                }
            }
            else
            {
                // 3. 非流模式：先重发保持目标（腿不塌），再处理手动命令
                if (leg.have_target.load(std::memory_order_relaxed))
                {
                    double q[kJointCount];
                    for (int j = 0; j < kJointCount; ++j)
                        q[j] = leg.target_rad[j].load(std::memory_order_relaxed);
                    sendLegTargets(leg, q);
                }
                Command cmd;
                bool got = false;
                {
                    std::unique_lock<std::mutex> lock(leg.cmd_mutex);
                    if (leg.cmd_cv.wait_for(lock, std::chrono::milliseconds(kIdlePollMs),
                                            [&]
                                            { return !leg.cmd_queue.empty() || g_estop.load() || g_stream_armed.load(); }))
                    {
                        if (!leg.cmd_queue.empty())
                        {
                            cmd = leg.cmd_queue.front();
                            leg.cmd_queue.pop();
                            got = true;
                        }
                    }
                }
                if (got)
                    executeMove(leg, cmd);
            }

            // 4. 10Hz 读实际位置刷新反馈缓存（host 回传 / pos / stat 的数据源）。
            //    离线电机读取退避：连续失败 ≥3 次后，该关节 1s 内不再尝试
            //    （0x41 读是阻塞往返，超时 200ms/帧；不退避会拖垮 5ms 下发节拍）
            const int64_t now_ms = nowMs();
            if (now_ms - last_read_ms >= kReadPeriodMs)
            {
                last_read_ms = now_ms;
                for (int j = 0; j < kJointCount; ++j)
                {
                    if (now_ms < backoff_until_ms[j])
                    {
                        leg.motor_online[j].store(false, std::memory_order_relaxed);
                        continue;
                    }
                    int32_t cur_ma = 0;
                    double vel = 0.0, motor_pos = 0.0;
                    if (leg.motors[j]->readCurrentVelocityPosition(cur_ma, vel, motor_pos))
                    {
                        const double joint_rad = motorToJointRad(j, motor_pos);
                        leg.fb_rad[j].store(joint_rad, std::memory_order_relaxed);
                        leg.motor_online[j].store(true, std::memory_order_relaxed);
                        offline_streak[j] = 0;

                        // 软限位检测：仅在上位机流模式下判定（手动 move/goto
                        // 的目标已被限位拒绝，不会走到限位，无需在此判定）。
                        // 按【实际位置】判定——关节真的运动到限位才报警。
                        const double deg = joint_rad / kDegToRad;

                        // 防抖：clear 之后关节若仍压在限位上，不立即复发报警，
                        // 必须先退离限位 kLimitRearmDeg 才重新武装（否则
                        // clear → 立刻报警 → clear 死循环）。
                        if (!rearm_ok[j])
                        {
                            if (deg <= kJointLimitDeg[j][1] - kLimitRearmDeg &&
                                deg >= kJointLimitDeg[j][0] + kLimitRearmDeg)
                                rearm_ok[j] = true; // 已退离，重新武装
                        }
                        else if (g_stream_armed.load(std::memory_order_relaxed) &&
                                 !g_limit_locked.load(std::memory_order_relaxed))
                        {
                            const uint8_t side = detectLimitSide(j, deg);
                            if (side != 0)
                            {
                                triggerLimitAlarm(leg.index - 1, j, side, deg);
                                rearm_ok[j] = false; // 触限后解除武装
                            }
                        }
                    }
                    else
                    {
                        leg.motor_online[j].store(false, std::memory_order_relaxed);
                        if (++offline_streak[j] >= 3)
                            backoff_until_ms[j] = nowMs() + 1000;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
        }

        // 退出前抱闸（quit 路径；estop 已在循环内处理）
        for (int j = 0; j < kJointCount; ++j)
            leg.motors[j]->stop();
    }

    // =====================================================================
    // 上位机命令接收线程（独立 CAN 接口，收 ID 1~16 目标帧 + 发心跳/回复）
    // =====================================================================

    /// 构造并发送一帧（host 线程专用 CAN 接口）。
    void hostSend(taihu::SocketCanInterface &can, uint16_t id, uint8_t dlc,
                  const uint8_t data[8])
    {
        taihu::CanFrame f;
        f.id = id;
        f.dlc = dlc;
        for (int i = 0; i < 8; ++i)
            f.data[i] = data[i];
        can.send(f);
    }

    /// 发送一帧 [0x41][int32 小端：关节端角 × 10000]（DLC=5）到上位机，
    /// 数据源 = 对应总线线程 10Hz 轮询电机 0x41 刷新的反馈缓存。
    void sendFeedbackFrame(taihu::SocketCanInterface &can, uint16_t id)
    {
        const Leg &leg = g_legs[idToLeg(id)];
        const double rad = leg.fb_rad[idToJoint(id)].load(std::memory_order_relaxed);
        uint8_t data[8] = {0};
        data[0] = kHostCmdGetSt;
        put_i32le(&data[1], radToProto(rad));
        hostSend(can, id, 5, data);
    }

    /// 回复单个 ID 的 0x41 状态请求（上位机点名查询）。
    void replyStatusRequest(taihu::SocketCanInterface &can, uint16_t id)
    {
        sendFeedbackFrame(can, id);
    }

    /// 发送一帧软限位报警（ID 0x162，DLC=8）到上位机。
    ///
    /// 帧格式：
    ///   data[0]    = 0x62           报警功能码
    ///   data[1]    = 关节 ID 1~16   触发报警的全局关节号
    ///   data[2]    = 侧别           0x01=上限，0x02=下限
    ///   data[3]    = 0              保留
    ///   data[4..7] = int32 小端     当前实际关节端角 × 10000（每次刷新）
    ///
    /// 实际角每次重发都重新读取（而非冻结在触发瞬间），上位机据此可判断
    /// 关节是否仍在运动。限位值不上送：上位机按关节号本地查表即得。
    void sendLimitAlarmFrame(taihu::SocketCanInterface &can)
    {
        const uint16_t jid = g_limit_alarm_id.load(std::memory_order_relaxed);
        const uint8_t side = g_limit_alarm_side.load(std::memory_order_relaxed);
        if (!isJointId(jid) || side == 0)
            return; // 未报警/报警源异常：不发

        const int leg_idx = idToLeg(jid);
        const int joint_idx = idToJoint(jid);
        const double rad =
            g_legs[leg_idx].fb_rad[joint_idx].load(std::memory_order_relaxed);

        uint8_t data[8] = {0};
        data[0] = kLimitAlarmCode;
        data[1] = static_cast<uint8_t>(jid);
        data[2] = side;
        data[3] = 0;
        put_i32le(&data[4], radToProto(rad));
        hostSend(can, kLimitAlarmId, 8, data);
    }

    /// host 线程主函数：独占上位机 CAN 接口，解析 ID 1~16 目标帧并拆分到
    /// 四条腿的目标缓存；10Hz 回传实际关节角；200ms 心跳；断流 1s 保护。
    ///
    /// 帧格式（与钛虎写命令帧同构，详见 docs/step_protocol.pdf）：
    ///   上位机 → 本机（标准帧，ID = 1~16 = 全局关节号 (腿-1)×4+关节号）：
    ///     [0x1E][int32 小端：目标关节端角(rad) × 10000]  DLC=5，10Hz
    ///     [0x02]   DLC=1：退出流模式（电机保持当前目标）
    ///     [0x03]   DLC=1：清除软限位报警锁定
    ///     [0x41]   DLC=1：状态请求 → 本机立即回该 ID 实际关节角
    ///   本机 → 上位机：
    ///     ID 1~16（DLC=5）[0x41][int32 小端：实际关节端角 × 10000]，10Hz
    ///     ID 0x161（DLC=1）200ms 心跳：bit0=流模式，bit1=样点在线，
    ///                                   bit2=软限位报警锁定中
    ///     ID 0x162（DLC=8）200ms 软限位报警（锁定期间持续重发）
    ///
    /// 拆分与保护：
    ///   - ID → (腿, 关节) 由 idToLeg/idToJoint 完成，写入 g_legs[腿].target_rad；
    ///   - 目标越限（kJointLimitDeg，关节端）→ 钳位到限位值后下发并计数
    ///     （关节走到限位即触发报警）；急停锁定 / 软限位锁定期间一律丢弃；
    ///   - 断流 > 1s → 自动退出流模式（电机保持，不抱闸）。
    void hostThread(const char *ifname)
    {
        taihu::SocketCanInterface can;
        if (!can.open(ifname, 0))
        {
            std::printf("[协议][错误] 打开 %s 失败，上位机链路不可用\n", ifname);
            return;
        }
        std::printf("[协议] 上位机链路就绪：%s（监听关节 ID %u~%u，"
                    "目标 [0x1E][int32=rad×%.0f]，退出流模式 [0x02]，"
                    "清除软限位 [0x03]，状态请求 [0x41]，"
                    "报警上报 [0x%X]/心跳 [0x%X]）\n",
                    ifname, kJointIdBase, kJointIdBase + kTotalJoints - 1, kAngleScale,
                    kLimitAlarmId, kHeartbeatId);

        int64_t last_heartbeat_ms = nowMs();
        int64_t last_feedback_ms = nowMs();
        int64_t last_alarm_ms = 0; ///< 上次发报警帧时刻（0 = 尚未发过）

        while (g_running.load())
        {
            taihu::CanFrame rx;
            if (!can.receive(rx, /*timeout_ms=*/20))
            {
                // —— 无帧（20ms 超时）：处理周期任务 ——
                const int64_t now = nowMs();

                // 10Hz 周期回传：16 个 ID 各发一帧 [0x41][实际关节角×10000]
                // （数据源 = 各总线线程 10Hz 轮询电机刷新的反馈缓存，无总线流量）
                if (now - last_feedback_ms >= kFeedbackMs)
                {
                    last_feedback_ms = now;
                    for (int i = 0; i < kLegCount; ++i)
                    {
                        if (!g_legs[i].enabled.load(std::memory_order_relaxed))
                            continue;
                        for (int j = 0; j < kJointCount; ++j)
                            sendFeedbackFrame(can, jointToId(i, j));
                    }
                }

                // 心跳发送（200ms 周期，ID 0x161，1 字节状态）
                if (now - last_heartbeat_ms >= kHeartbeatMs)
                {
                    uint8_t d[1];
                    d[0] = static_cast<uint8_t>(
                        (g_stream_armed.load() ? 0x01 : 0x00) |  // bit0：流模式中
                        (g_stream_online.load() ? 0x02 : 0x00) | // bit1：样点在线
                        (g_limit_locked.load() ? 0x04 : 0x00));  // bit2：软限位锁定
                    hostSend(can, kHeartbeatId, 1, d);
                    last_heartbeat_ms = now;
                }

                // 软限位报警重发（锁定期间 200ms 周期；首次触发立即发一帧）
                if (g_limit_locked.load())
                {
                    if (last_alarm_ms == 0 || now - last_alarm_ms >= kLimitAlarmMs)
                    {
                        sendLimitAlarmFrame(can);
                        last_alarm_ms = now;
                    }
                }
                else
                {
                    last_alarm_ms = 0; // 解除后重置，下次触发立即上报
                }

                // 断流退出：流模式中 1s 无新样点（电机保持当前目标，不抱闸）
                if (g_stream_armed.load() &&
                    now - g_stream_last_rx_ms.load() > kStreamTimeoutMs)
                {
                    g_stream_armed.store(false);
                    g_stream_online.store(false);
                    std::printf("[流][警告] 超过 %dms 未收到新样点，自动退出流模式"
                                "（电机保持当前目标）\n",
                                kStreamTimeoutMs);
                }
                continue;
            }

            // —— 收到帧：按 ID 分流 ——
            const uint16_t id = static_cast<uint16_t>(rx.id);
            if (rx.is_extended)
                continue; // 只接受标准帧

            if (isJointId(id))
            {
                // ID 1~16：单关节命令帧
                const int leg_idx = idToLeg(id);
                const int joint_idx = idToJoint(id);
                Leg &leg = g_legs[leg_idx];

                if (rx.dlc == 1 && rx.data[0] == kHostCmdStop)
                {
                    // 退出流模式（电机保持当前角，不抱闸）
                    if (g_stream_armed.load())
                    {
                        g_stream_armed.store(false);
                        g_stream_online.store(false);
                        std::printf("[流] 收到 0x02（ID %u）：退出流模式，"
                                    "电机保持当前目标\n",
                                    id);
                    }
                    continue;
                }

                if (rx.dlc == 1 && rx.data[0] == kHostCmdClearLimit)
                {
                    // 清除软限位报警锁定（整机恢复接收目标；电机不抱闸）
                    clearLimitAlarm();
                    continue;
                }

                if (rx.dlc == 1 && rx.data[0] == kHostCmdGetSt)
                {
                    replyStatusRequest(can, id);
                    continue;
                }

                if (rx.dlc == 5 && rx.data[0] == kHostCmdSetPos)
                {
                    // 目标帧：int32 小端 = 关节端角(rad) × 10000 → 拆分写入
                    // 对应腿的目标缓存（该腿 busThread 5ms 周期下发真电机）
                    const double rad = protoToRad(get_i32le(&rx.data[1]));

                    // 急停锁定期间一律丢弃（电机保持抱闸）
                    if (g_estop_locked.load())
                    {
                        leg.rx_frames.fetch_add(1, std::memory_order_relaxed);
                        leg.dropped.fetch_add(1, std::memory_order_relaxed);
                        continue;
                    }
                    // 软限位锁定期间一律丢弃（目标已冻结在锁定瞬间的值）
                    if (g_limit_locked.load())
                    {
                        leg.rx_frames.fetch_add(1, std::memory_order_relaxed);
                        leg.dropped.fetch_add(1, std::memory_order_relaxed);
                        continue;
                    }
                    // 软件限位检查：越限不再丢弃，而是钳位到限位值后下发
                    // （关节会走到限位并停住，由实际位置判定触发报警）
                    {
                        const double deg = rad / kDegToRad;
                        if (!withinLimits(joint_idx, deg))
                        {
                            const double clamped_deg = clampToLimits(joint_idx, deg);
                            leg.rx_frames.fetch_add(1, std::memory_order_relaxed);
                            if (leg.dropped.fetch_add(1, std::memory_order_relaxed) == 0)
                                std::printf("[流][腿%d][%s][钳位] 目标 %.2f° 越限"
                                            "（限位 [%.0f°, %.0f°]）→ 钳位到 %.2f°\n",
                                            leg.index, kJointNames[joint_idx], deg,
                                            kJointLimitDeg[joint_idx][0],
                                            kJointLimitDeg[joint_idx][1], clamped_deg);
                            leg.target_rad[joint_idx].store(
                                clamped_deg * kDegToRad, std::memory_order_relaxed);
                            leg.have_target.store(true, std::memory_order_relaxed);
                            g_stream_online.store(true);
                            g_stream_last_rx_ms.store(nowMs());
                            if (!g_stream_armed.load())
                            {
                                g_stream_armed.store(true);
                                std::printf("[流] 收到首帧 0x1E（ID %u = 腿%d %s）："
                                            "进入流模式\n",
                                            id, leg_idx + 1, kJointNames[joint_idx]);
                            }
                            continue;
                        }
                    }
                    leg.target_rad[joint_idx].store(rad, std::memory_order_relaxed);
                    leg.have_target.store(true, std::memory_order_relaxed);
                    leg.rx_frames.fetch_add(1, std::memory_order_relaxed);

                    if (!g_stream_armed.load())
                    {
                        g_stream_armed.store(true);
                        std::printf("[流] 收到首帧 0x1E（ID %u = 腿%d %s）：进入流模式\n",
                                    id, leg_idx + 1, kJointNames[joint_idx]);
                    }
                    g_stream_online.store(true);
                    g_stream_last_rx_ms.store(nowMs());
                }
                continue;
            }
            // 其它 ID（如误接电机总线的回帧）：忽略
        }
        can.close();
    }

    // =====================================================================
    // 命令解析（主线程）
    // =====================================================================

    /// 去掉行尾换行与回车。
    void chomp(char *s)
    {
        size_t n = std::strlen(s);
        while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r'))
            s[--n] = '\0';
    }

    /// 解析腿号（1~4），成功返回 true。
    bool parseLeg(const char *tok, int &leg)
    {
        if (tok == nullptr)
            return false;
        char *end = nullptr;
        const long v = std::strtol(tok, &end, 10);
        if (end == tok || *end != '\0' || v < 1 || v > kLegCount)
            return false;
        leg = static_cast<int>(v);
        return true;
    }

    /// 解析关节名（J1~J4，不分大小写），返回下标 0~3；失败返回 -1。
    int parseJoint(const char *tok)
    {
        if (tok == nullptr || std::strlen(tok) != 2)
            return -1;
        if (tok[0] != 'J' && tok[0] != 'j')
            return -1;
        const int v = tok[1] - '1';
        return (v >= 0 && v < kJointCount) ? v : -1;
    }

    /// 解析浮点数（附带范围检查），失败返回 false。
    bool parseDouble(const char *tok, double lo, double hi, double &out)
    {
        if (tok == nullptr)
            return false;
        char *end = nullptr;
        const double v = std::strtod(tok, &end);
        if (end == tok || *end != '\0')
            return false;
        if (v < lo || v > hi)
            return false;
        out = v;
        return true;
    }

    // =====================================================================
    // 启动参数解析（命令行 → 运行时可变参数）
    //
    // 设计原则：**不带任何参数运行时，行为与历史版本完全一致**——仍然
    // 逐项交互式询问「启用的腿」与「上位机 CAN 接口」。带参数时跳过对应
    // 提问，便于脚本化启动（systemd/ROS2 launch）与现场快速试参。
    // =====================================================================

    /// 启动参数（只记录「是否显式指定」，取值直接写入运行时可变参数）
    struct StartupArgs
    {
        bool show_help = false;  ///< --help：打印参数说明后退出
        bool dry_run = false;    ///< --dry-run：只校验参数，不打开 CAN
        bool legs_given = false; ///< 给了 --legs → 不再提问腿号
        bool host_given = false; ///< 给了 --host/--no-host → 不再提问
        bool want[kLegCount + 1] = {false, false, false, false, false};
        std::string host_ifname; ///< 上位机 CAN 接口名（空 = 跳过链路）
    };

    /// 解析逗号/分号/空格分隔的 N 个浮点值到 dst（N = kJointCount）。
    bool parseFourDoubles(const char *tok, double dst[kJointCount],
                          double lo, double hi)
    {
        if (tok == nullptr)
            return false;
        std::string s(tok);
        if (s.empty())
            return false;
        char *save = nullptr;
        int i = 0;
        for (char *t = strtok_r(&s[0], ",;: \t", &save); t != nullptr;
             t = strtok_r(nullptr, ",;: \t", &save), ++i)
        {
            if (i >= kJointCount || !parseDouble(t, lo, hi, dst[i]))
                return false;
        }
        return i == kJointCount;
    }

    /// 解析 --limits："-120,120;-120,120;-30,180;-90,90"（4 组关节端限位，度）。
    bool parseLimits(const char *tok)
    {
        if (tok == nullptr)
            return false;
        std::string s(tok);
        if (s.empty())
            return false;
        char *save = nullptr;
        int joint = 0;
        for (char *grp = strtok_r(&s[0], ";", &save); grp != nullptr;
             grp = strtok_r(nullptr, ";", &save), ++joint)
        {
            if (joint >= kJointCount)
                return false;
            char *save2 = nullptr;
            const char *lo_tok = strtok_r(grp, ", \t", &save2);
            const char *hi_tok = strtok_r(nullptr, ", \t", &save2);
            double lo = 0.0, hi = 0.0;
            if (!parseDouble(lo_tok, -360.0, 360.0, lo) ||
                !parseDouble(hi_tok, -360.0, 360.0, hi) || lo >= hi)
                return false;
            kJointLimitDeg[joint][0] = lo;
            kJointLimitDeg[joint][1] = hi;
        }
        return joint == kJointCount;
    }

    /// 解析 --legs："1 2 3" / "1,2,3" / "all"。
    bool parseLegList(const char *tok, StartupArgs &out)
    {
        if (tok == nullptr)
            return false;
        if (std::strcmp(tok, "all") == 0)
        {
            for (int i = 1; i <= kLegCount; ++i)
                out.want[i] = true;
            return true;
        }
        std::string s(tok);
        if (s.empty())
            return false;
        char *save = nullptr;
        int n = 0;
        for (char *t = strtok_r(&s[0], ",;: \t", &save); t != nullptr;
             t = strtok_r(nullptr, ",;: \t", &save))
        {
            int leg = 0;
            if (!parseLeg(t, leg))
                return false;
            out.want[leg] = true;
            ++n;
        }
        return n > 0;
    }

    /// 打印命令行参数说明（--help）。
    void printCliHelp()
    {
        std::printf(
            "用法：walk_control_V1 [选项]\n"
            "\n"
            "不带任何选项运行 = 历史交互流程（依次询问启用的腿、上位机接口名）。\n"
            "带选项时跳过对应提问；所有选项的默认值等于历史硬编码值。\n"
            "\n"
            "构型与接入：\n"
            "  --legs \"1 2 3 4\"       启用的腿（空格/逗号分隔，all=全部）\n"
            "  --host <接口名>          上位机 CAN 接口（如 can4；不可用 can0~can3）\n"
            "  --no-host                不启用上位机链路（仅键盘命令）\n"
            "  --interactive            强制回到交互式提问（覆盖上面的跳过行为）\n"
            "  --dry-run                只解析参数并打印生效配置后退出（不碰 CAN）\n"
            "\n"
            "限位与传动（均为**关节端**语义）：\n"
            "  --limits \"a,b;c,d;e,f;g,h\"   J1~J4 的 {min,max}（度），默认 "
            "-120,120;-120,120;-30,180;-90,90\n"
            "  --trigger-deg <度>        限位触发阈值（默认 0.5，需大于 PD 稳态误差）\n"
            "  --rearm-deg <度>          解除锁定后重新武装的回差（默认 2.0）\n"
            "  --dir-sign \"+1,+1,+1,+1\" 关节方向符号（仅 ±1；锥齿轮装反时改 -1）\n"
            "  --gear \"101,162,162,101\" 关节端总减速比（仅影响速度换算）\n"
            "  --motor-scale \"1,2,2,1\"  电机输出端角 / 关节角（锥齿轮 1:2 填 2）\n"
            "\n"
            "电机位置环与通讯节奏：\n"
            "  --kp <int>                位置环 KP（默认 4000，写入电机固件）\n"
            "  --kd <int>                位置环 KD（默认 60）\n"
            "  --stream-timeout-ms <ms>  断流超时，超时自动退出流模式（默认 1000）\n"
            "  --heartbeat-ms <ms>       心跳 0x161 周期（默认 200）\n"
            "  --alarm-ms <ms>           软限位报警 0x162 重发周期（默认 200）\n"
            "  --feedback-hz <hz>        反馈回传频率（默认 10，范围 1~100）\n"
            "\n"
            "示例：\n"
            "  walk_control_V1                                     # 历史交互流程\n"
            "  walk_control_V1 --legs 1 --host can4                # 单腿 + 上位机链路\n"
            "  walk_control_V1 --legs all --no-host --kp 3000      # 全部腿，仅键盘\n"
            "  walk_control_V1 --limits \"-45,45;-60,60;-15,90;-45,45\"  # 缩小限位试限\n"
            "  walk_control_V1 --dry-run --legs 1 --host can4      # 只校验参数\n");
    }

    /// 解析全部命令行参数。
    /// @return false 表示参数非法（原因已打印）；true 表示可继续启动。
    bool parseArgs(int argc, char **argv, StartupArgs &out)
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string key = argv[i];
            const char *val = (i + 1 < argc) ? argv[i + 1] : nullptr;

            // 取一个取值（成功则把 i 推进到该取值）
            auto take = [&](const char *what) -> const char *
            {
                if (val == nullptr)
                {
                    std::printf("[参数][错误] %s 缺少取值\n", what);
                    return nullptr;
                }
                ++i;
                return val;
            };

            if (key == "--help" || key == "-h")
            {
                out.show_help = true;
                return true;
            }
            else if (key == "--legs")
            {
                const char *v = take("--legs");
                if (v == nullptr)
                    return false;
                for (int k = 0; k <= kLegCount; ++k)
                    out.want[k] = false;
                if (!parseLegList(v, out))
                {
                    std::printf("[参数][错误] --legs 取值非法：\"%s\""
                                "（示例：\"1 2 3\" 或 \"all\"）\n",
                                v);
                    return false;
                }
                out.legs_given = true;
            }
            else if (key == "--host")
            {
                const char *v = take("--host");
                if (v == nullptr)
                    return false;
                out.host_ifname = v;
                out.host_given = true;
            }
            else if (key == "--no-host" || key == "--no-host-link")
            {
                out.host_ifname.clear();
                out.host_given = true;
            }
            else if (key == "--interactive")
            {
                out.legs_given = false;
                out.host_given = false;
            }
            else if (key == "--dry-run")
            {
                out.dry_run = true;
            }
            else if (key == "--limits")
            {
                const char *v = take("--limits");
                if (v == nullptr)
                    return false;
                if (!parseLimits(v))
                {
                    std::printf("[参数][错误] --limits 取值非法：\"%s\""
                                "（示例：\"-120,120;-120,120;-30,180;-90,90\"）\n",
                                v);
                    return false;
                }
            }
            else if (key == "--trigger-deg")
            {
                const char *v = take("--trigger-deg");
                if (v == nullptr)
                    return false;
                if (!parseDouble(v, 0.0, 45.0, kLimitTriggerDeg))
                {
                    std::printf("[参数][错误] --trigger-deg 需为 0~45 度\n");
                    return false;
                }
            }
            else if (key == "--rearm-deg")
            {
                const char *v = take("--rearm-deg");
                if (v == nullptr)
                    return false;
                if (!parseDouble(v, 0.0, 90.0, kLimitRearmDeg))
                {
                    std::printf("[参数][错误] --rearm-deg 需为 0~90 度\n");
                    return false;
                }
            }
            else if (key == "--dir-sign")
            {
                const char *v = take("--dir-sign");
                if (v == nullptr)
                    return false;
                double tmp[kJointCount];
                if (!parseFourDoubles(v, tmp, -1.0, 1.0))
                {
                    std::printf("[参数][错误] --dir-sign 需为 4 个 +1/-1\n");
                    return false;
                }
                for (int j = 0; j < kJointCount; ++j)
                {
                    if (std::fabs(tmp[j]) != 1.0)
                    {
                        std::printf("[参数][错误] --dir-sign 每项只能为 +1 或 -1\n");
                        return false;
                    }
                    kJointDirSign[j] = tmp[j];
                }
            }
            else if (key == "--gear")
            {
                const char *v = take("--gear");
                if (v == nullptr)
                    return false;
                if (!parseFourDoubles(v, kJointGearRatios, 1.0, 10000.0))
                {
                    std::printf("[参数][错误] --gear 需为 4 个正数"
                                "（示例：\"101,162,162,101\"）\n");
                    return false;
                }
            }
            else if (key == "--motor-scale")
            {
                const char *v = take("--motor-scale");
                if (v == nullptr)
                    return false;
                if (!parseFourDoubles(v, kMotorRadPerJointRad, 0.01, 100.0))
                {
                    std::printf("[参数][错误] --motor-scale 需为 4 个正数"
                                "（示例：\"1,2,2,1\"）\n");
                    return false;
                }
            }
            else if (key == "--kp")
            {
                const char *v = take("--kp");
                if (v == nullptr)
                    return false;
                char *end = nullptr;
                const long n = std::strtol(v, &end, 10);
                if (end == v || *end != '\0' || n < 0 || n > 100000)
                {
                    std::printf("[参数][错误] --kp 需为 0~100000 的整数\n");
                    return false;
                }
                kPositionKp = static_cast<int32_t>(n);
            }
            else if (key == "--kd")
            {
                const char *v = take("--kd");
                if (v == nullptr)
                    return false;
                char *end = nullptr;
                const long n = std::strtol(v, &end, 10);
                if (end == v || *end != '\0' || n < 0 || n > 100000)
                {
                    std::printf("[参数][错误] --kd 需为 0~100000 的整数\n");
                    return false;
                }
                kPositionKd = static_cast<int32_t>(n);
            }
            else if (key == "--stream-timeout-ms")
            {
                const char *v = take("--stream-timeout-ms");
                if (v == nullptr)
                    return false;
                char *end = nullptr;
                const long n = std::strtol(v, &end, 10);
                if (end == v || *end != '\0' || n < 20 || n > 60000)
                {
                    std::printf("[参数][错误] --stream-timeout-ms 需为 20~60000\n");
                    return false;
                }
                kStreamTimeoutMs = static_cast<int>(n);
            }
            else if (key == "--heartbeat-ms")
            {
                const char *v = take("--heartbeat-ms");
                if (v == nullptr)
                    return false;
                char *end = nullptr;
                const long n = std::strtol(v, &end, 10);
                if (end == v || *end != '\0' || n < 10 || n > 5000)
                {
                    std::printf("[参数][错误] --heartbeat-ms 需为 10~5000\n");
                    return false;
                }
                kHeartbeatMs = static_cast<int>(n);
            }
            else if (key == "--alarm-ms")
            {
                const char *v = take("--alarm-ms");
                if (v == nullptr)
                    return false;
                char *end = nullptr;
                const long n = std::strtol(v, &end, 10);
                if (end == v || *end != '\0' || n < 10 || n > 5000)
                {
                    std::printf("[参数][错误] --alarm-ms 需为 10~5000\n");
                    return false;
                }
                kLimitAlarmMs = static_cast<int>(n);
            }
            else if (key == "--feedback-hz")
            {
                const char *v = take("--feedback-hz");
                if (v == nullptr)
                    return false;
                char *end = nullptr;
                const long n = std::strtol(v, &end, 10);
                if (end == v || *end != '\0' || n < 1 || n > 100)
                {
                    std::printf("[参数][错误] --feedback-hz 需为 1~100\n");
                    return false;
                }
                kFeedbackMs = 1000 / static_cast<int>(n);
                // 反馈周期不会快于总线线程读电机的周期，否则只是重复缓存
                if (kFeedbackMs < kReadPeriodMs)
                {
                    std::printf("[参数][提示] --feedback-hz %ld 快于总线读取周期"
                                "（%dms / %dHz），回传周期按 %dms 生效\n",
                                n, kReadPeriodMs, 1000 / kReadPeriodMs,
                                kReadPeriodMs);
                    kFeedbackMs = kReadPeriodMs;
                }
            }
            else
            {
                std::printf("[参数][错误] 未知选项 \"%s\"（--help 查看全部选项）\n",
                            key.c_str());
                return false;
            }
        }
        return true;
    }

    /// 打印当前生效的可调参数（启动时一行摘要；--dry-run 时逐项展开）。
    void printEffectiveConfig(bool verbose)
    {
        std::printf("[参数] 限位(关节端°) J1 [%.0f,%.0f] J2 [%.0f,%.0f] "
                    "J3 [%.0f,%.0f] J4 [%.0f,%.0f]\n",
                    kJointLimitDeg[0][0], kJointLimitDeg[0][1],
                    kJointLimitDeg[1][0], kJointLimitDeg[1][1],
                    kJointLimitDeg[2][0], kJointLimitDeg[2][1],
                    kJointLimitDeg[3][0], kJointLimitDeg[3][1]);
        std::printf("[参数] KP=%d KD=%d 触发=%.2f° 回差=%.1f° "
                    "断流=%dms 心跳=%dms 报警=%dms 反馈=%dms\n",
                    kPositionKp, kPositionKd, kLimitTriggerDeg, kLimitRearmDeg,
                    kStreamTimeoutMs, kHeartbeatMs, kLimitAlarmMs, kFeedbackMs);
        std::printf("[参数] 方向符号 %+.0f %+.0f %+.0f %+.0f；"
                    "电机端/关节端 %g %g %g %g；减速比 %g %g %g %g\n",
                    kJointDirSign[0], kJointDirSign[1],
                    kJointDirSign[2], kJointDirSign[3],
                    kMotorRadPerJointRad[0], kMotorRadPerJointRad[1],
                    kMotorRadPerJointRad[2], kMotorRadPerJointRad[3],
                    kJointGearRatios[0], kJointGearRatios[1],
                    kJointGearRatios[2], kJointGearRatios[3]);
        if (!verbose)
            return;
        std::printf("[参数] 关节端限位对应的电机端等效范围：\n");
        for (int j = 0; j < kJointCount; ++j)
            std::printf("         %s：关节端 [%+7.1f°, %+7.1f°]"
                        " ↔ 电机端 [%+8.1f°, %+8.1f°]\n",
                        kJointNames[j],
                        kJointLimitDeg[j][0], kJointLimitDeg[j][1],
                        kJointLimitDeg[j][0] * kMotorRadPerJointRad[j],
                        kJointLimitDeg[j][1] * kMotorRadPerJointRad[j]);
    }

    /// 解析 pos 命令：pos 不入队，主线程直接读 10Hz 反馈缓存打印（零总线流量）。
    /// @return true 表示该行确实是 pos 命令（leg_out：1~4，0 = 全部腿）。
    bool tryParsePos(char *line, int &leg_out)
    {
        char *save = nullptr;
        const char *verb = strtok_r(line, " \t", &save);
        if (verb == nullptr || std::strcmp(verb, "pos") != 0)
            return false;
        leg_out = 0;
        const char *leg_tok = strtok_r(nullptr, " \t", &save);
        if (leg_tok != nullptr)
        {
            int leg = 0;
            if (!parseLeg(leg_tok, leg))
            {
                std::printf("[格式] 腿号需为 1~4（缺省=全部腿）\n");
                return true; // 已识别为 pos，只是参数有误
            }
            leg_out = leg;
            if (strtok_r(nullptr, " \t", &save) != nullptr)
                std::printf("[格式] pos 多余参数（已忽略）\n");
        }
        return true;
    }

    /// 打印各腿关节当前位置（数据源 = 总线线程 10Hz 刷新的反馈缓存，
    /// 因此显示精度取决于 100ms 轮询周期，且不占用任何 CAN 带宽）。
    void printPos(int leg)
    {
        const int leg_lo = (leg == 0) ? 1 : leg;
        const int leg_hi = (leg == 0) ? kLegCount : leg;
        for (int i = leg_lo; i <= leg_hi; ++i)
        {
            const Leg &l = g_legs[i - 1];
            if (!l.enabled.load(std::memory_order_relaxed))
            {
                std::printf("[腿%d] 未启用（CAN %s 打开失败）\n", i, l.ifname.c_str());
                continue;
            }
            std::printf("[腿%d] %s：", i, l.ifname.c_str());
            for (int j = 0; j < kJointCount; ++j)
            {
                const double rad = l.fb_rad[j].load(std::memory_order_relaxed);
                const bool online = l.motor_online[j].load(std::memory_order_relaxed);
                std::printf("%s %8.2f°%s", kJointNames[j], rad / kDegToRad,
                            online ? "" : "(离线)");
            }
            std::printf("\n");
        }
    }

    /// 解析一行运动命令（move / goto），成功返回 true（cmd 为输出）。
    bool parseCommand(char *line, Command &cmd)
    {
        // strtok 线程不安全，但只在主线程使用（demo 级别可接受）
        char *save = nullptr;
        const char *verb = strtok_r(line, " \t", &save);
        if (verb == nullptr)
            return false;

        if (std::strcmp(verb, "move") == 0 || std::strcmp(verb, "goto") == 0)
        {
            const bool relative = (std::strcmp(verb, "move") == 0);

            int leg = 0;
            if (!parseLeg(strtok_r(nullptr, " \t", &save), leg))
            {
                std::printf("[格式] 腿号需为 1~4\n");
                return false;
            }
            const int joint = parseJoint(strtok_r(nullptr, " \t", &save));
            if (joint < 0)
            {
                std::printf("[格式] 关节需为 J1~J4\n");
                return false;
            }

            // move 带 cw/ccw 方向词；goto 直接跟带符号角度
            double angle_deg = 0.0;
            if (relative)
            {
                const char *dir = strtok_r(nullptr, " \t", &save);
                if (dir == nullptr ||
                    (std::strcmp(dir, "cw") != 0 && std::strcmp(dir, "ccw") != 0))
                {
                    std::printf("[格式] 方向需为 cw（顺时针）或 ccw（逆时针）\n");
                    return false;
                }
                if (!parseDouble(strtok_r(nullptr, " \t", &save), 0.1, kMaxAngleDeg,
                                 angle_deg))
                {
                    std::printf("[格式] 角度需为 0.1~%.0f°\n", kMaxAngleDeg);
                    return false;
                }
                if (std::strcmp(dir, "cw") == 0)
                    angle_deg = -angle_deg; // cw = 负
            }
            else
            {
                if (!parseDouble(strtok_r(nullptr, " \t", &save),
                                 -kMaxAngleDeg, kMaxAngleDeg, angle_deg))
                {
                    std::printf("[格式] 角度需为 ±%.0f° 以内\n", kMaxAngleDeg);
                    return false;
                }
            }

            // 可选：轨迹时长
            double duration = kDefaultMoveSec;
            const char *dur_tok = strtok_r(nullptr, " \t", &save);
            if (dur_tok != nullptr &&
                !parseDouble(dur_tok, kMinMoveSec, kMaxMoveSec, duration))
            {
                std::printf("[格式] 时长需为 %.1f~%.0fs\n", kMinMoveSec, kMaxMoveSec);
                return false;
            }
            if (strtok_r(nullptr, " \t", &save) != nullptr)
            {
                std::printf("[格式] 多余参数\n");
                return false;
            }

            cmd.type = relative ? Command::Type::Move : Command::Type::Goto;
            cmd.leg = leg;
            cmd.joint = joint;
            cmd.relative = relative;
            cmd.angle_deg = angle_deg;
            cmd.duration_s = duration;
            return true;
        }

        std::printf("[格式] 未知命令（help 查看用法）\n");
        return false;
    }

    /// 打印帮助。
    void printHelp()
    {
        std::printf(
            "命令格式（角度均按【关节端】计算）：\n"
            "  move <腿1-4> <J1-J4> <cw|ccw> <角度°> [时长s]  相对转动，例：move 1 J1 cw 90\n"
            "  goto <腿1-4> <J1-J4> <角度°> [时长s]           绝对关节角，例：goto 1 J2 45\n"
            "  pos  [腿1-4]                                    查询当前位置（读 10Hz 缓存，缺省=全部腿）\n"
            "  stat                                            流模式统计（各腿收帧/钳位/电机在线/使能）\n"
            "  arm                                             使能：解除全部腿的抱闸（启动时已自动执行）\n"
            "  disarm                                          去使能：全部电机抱闸并锁定\n"
            "  estop                                           急停（全部电机抱闸并锁定）\n"
            "  resume                                          解除急停锁定并重新使能（解抱闸）\n"
            "  clear                                           清除软限位报警锁定（同上位机 [0x03]）\n"
            "  help                                            帮助\n"
            "  quit                                            退出\n"
            "说明：cw=顺时针（负方向），ccw=逆时针（正方向）；默认轨迹时长 %.1fs。\n"
            "      ★ 使能 = 解抱闸（协议无独立使能命令，setTargetCurrent(0) 即「使能+零转矩」）；\n"
            "        上电默认去使能，未使能时 0x1E 位置帧不会被执行。启动流程已自动使能。\n"
            "      上位机流：在上位机 CAN 接口上发 ID 1~16 帧 [0x1E][int32=关节端 rad×%.0f]，\n"
            "      ID=(腿-1)×4+关节号，10Hz（协议见 docs/step_protocol.pdf）。\n"
            "      流模式中 move/goto 暂缓；急停锁定期间上位机目标一律丢弃。\n"
            "      ★ 软限位保护：越限目标钳位到限位值下发；实际角到达限位时整机锁定并\n"
            "        以 ID 0x162 上报报警（200ms 重发），需 [0x03] 或 clear 解除。\n"
            "      启动参数（限位/KP/KD/超时/接口名等）见命令行 --help。\n",
            kDefaultMoveSec, kAngleScale);
    }

    // =====================================================================
    // 初始化与主循环
    // =====================================================================

    /// 打开一条腿的 CAN 总线并初始化四个电机。
    /// @param probe 是否读一次状态探测电机在线（打印结果）
    /// @return 初始化成功返回 true（个别电机离线仅警告，不判失败）
    bool initLeg(int leg_index, bool probe)
    {
        Leg &leg = g_legs[leg_index - 1];
        leg.index = leg_index;
        char ifbuf[16];
        std::snprintf(ifbuf, sizeof ifbuf, "can%d", leg_index - 1);
        leg.ifname = ifbuf;

        if (!leg.can.open(leg.ifname.c_str(), 0))
        {
            std::printf("[腿%d][错误] 打开 %s 失败（接口需先 up："
                        "sudo ./build/kcanctl up %s 1000000）\n",
                        leg_index, leg.ifname.c_str(), leg.ifname.c_str());
            return false;
        }

        // 初始化四个电机：减速比（关节端总减速比，速度换算用）+ 位置环增益
        for (int j = 0; j < kJointCount; ++j)
        {
            taihu::MotorConfig cfg{"leg" + std::to_string(leg_index) + "_" + kJointNames[j],
                                   kMotorIds[j], kJointGearRatios[j], kPositionKp, kPositionKd};
            leg.motors[j] = taihu::initMotor(leg.can, cfg);
            leg.motors[j]->clearError();
        }

        // 探测：逐电机读一次 0x41，确认在线（离线仅警告）
        if (probe)
        {
            std::printf("[腿%d] %s 电机在线性：", leg_index, leg.ifname.c_str());
            for (int j = 0; j < kJointCount; ++j)
            {
                int32_t cur_ma = 0;
                double vel = 0.0, pos = 0.0;
                const bool ok =
                    leg.motors[j]->readCurrentVelocityPosition(cur_ma, vel, pos);
                std::printf("%s%s", j > 0 ? " " : "", ok ? kJointNames[j] : "×");
            }
            std::printf("（× = 无响应）\n");
        }

        // 启动即使能（解抱闸）：上电默认去使能(抱闸)，不使能则 0x1E 位置帧
        // 不会被执行——表现为「上位机发目标、关节角度一直是 0」。
        // 这里在起 busThread 之前调用，天然独占该腿 CAN，无并发问题。
        enableLeg(leg);

        leg.enabled.store(true);
        return true;
    }

} // namespace

int main(int argc, char **argv)
{
    // 0. 解析命令行参数（--help / 未知选项 / 非法取值都在此退出）。
    //    不带任何参数时 legs_given / host_given 均为 false，后续仍走
    //    交互式提问 —— 与历史版本行为完全一致。
    StartupArgs args;
    if (!parseArgs(argc, argv, args))
        return 2;
    if (args.show_help)
    {
        printCliHelp();
        return 0;
    }

    std::printf("============================================================\n");
    std::printf("  四腿机器人【下位机】walk_control_V1（上位机轨迹流拆分 + 关节级命令）\n");
    std::printf("============================================================\n");
    std::printf("  构型：4 腿 × 4 关节 YPPY（腿 i ↔ can%d~can%d，电机 ID 1~4）\n", 0, kLegCount - 1);
    std::printf("        J1/J4 = Y 直驱；J2/J3 = P 锥齿轮 1:2（减速比见下方 [参数] 行）\n");
    std::printf("  上位机：独立 CAN 接口，ID 1~16 = (腿-1)×4+关节号，\n");
    std::printf("          [0x1E][int32 小端 = 关节端 rad × %.0f]，10Hz；\n", kAngleScale);
    std::printf("          本机拆分后按钛虎原生协议下发真电机（电机内部 PD 执行）\n");
    std::printf("  角度：按【关节端】计算；ccw（逆时针）= 正，与 xCAN 电机方向一致\n");
    std::printf("  软保护：越限目标钳位到限位值下发；实际角到达限位 → 整机锁定，\n");
    std::printf("          以 ID 0x162 上报报警（200ms 重发），需 [0x03] 或 clear 解除\n");
    std::printf("  使能：启动时自动解除抱闸（arm 重新使能 / disarm 抱闸）\n");
    std::printf("  例：move 1 J1 cw 90  →  腿1 的 J1 顺时针转 90°（平滑轨迹）\n");
    std::printf("  提示：walk_control_V1 --help 查看全部可调参数\n");
    std::printf("------------------------------------------------------------\n");
    if (!args.dry_run)
        printEffectiveConfig(/*verbose=*/false);
    //    交互式：直接回车 = 全部；现场只接部分腿时可只输入如 1）
    bool want[kLegCount + 1] = {false, false, false, false, false};
    char line[256] = {0};
    if (args.legs_given)
    {
        for (int i = 0; i <= kLegCount; ++i)
            want[i] = args.want[i];
        std::printf("[参数] 启用腿：");
        for (int i = 1; i <= kLegCount; ++i)
            if (want[i])
                std::printf("%d ", i);
        std::printf("（来自 --legs）\n");
    }
    else
    {
        std::printf("请输入要启用的腿号（1~4，空格分隔，直接回车=全部）：");
        if (std::fgets(line, sizeof line, stdin) == nullptr)
            return -1;
        chomp(line);

        if (line[0] == '\0')
        {
            for (int i = 1; i <= kLegCount; ++i)
                want[i] = true;
        }
        else
        {
            char *save = nullptr;
            for (char *tok = strtok_r(line, " \t", &save); tok != nullptr;
                 tok = strtok_r(nullptr, " \t", &save))
            {
                int leg = 0;
                if (!parseLeg(tok, leg))
                {
                    std::printf("[格式] 腿号需为 1~4，收到 \"%s\"\n", tok);
                    return -1;
                }
                want[leg] = true;
            }
        }
    }

    // 2. 输入上位机 CAN 接口名（必须是独立接口，不能与电机总线 can0~can3
    //    共用——否则上位机发的 ID 1~4 会被腿 1 真电机直接执行）
    std::string host_ifname;
    if (args.host_given)
    {
        host_ifname = args.host_ifname;
        std::printf("[参数] 上位机接口：%s（来自命令行）\n",
                    host_ifname.empty() ? "（不启用）" : host_ifname.c_str());
    }
    else
    {
        std::printf("请输入上位机 CAN 接口名（如 can4/can5，直接回车=跳过上位机链路）：");
        if (std::fgets(line, sizeof line, stdin) == nullptr)
            return -1;
        chomp(line);
        host_ifname = line;
    }
    if (!host_ifname.empty())
    {
        for (int i = 0; i < kLegCount; ++i)
        {
            char ifbuf[16];
            std::snprintf(ifbuf, sizeof ifbuf, "can%d", i);
            if (host_ifname == ifbuf)
            {
                std::printf("[错误] 上位机接口不能与电机总线 %s 共用"
                            "（ID 1~4 会被腿1 真电机直接执行）\n",
                            ifbuf);
                return -1;
            }
        }
    }

    // 2.5 --dry-run：只校验参数，不打开任何 CAN 设备
    if (args.dry_run)
    {
        std::printf("[dry-run] 参数校验通过，未打开任何 CAN 接口。\n");
        std::printf("[dry-run] 启用腿：");
        for (int i = 1; i <= kLegCount; ++i)
            if (want[i])
                std::printf("%d ", i);
        std::printf("；上位机接口：%s\n",
                    host_ifname.empty() ? "（不启用）" : host_ifname.c_str());
        printEffectiveConfig(/*verbose=*/true);
        return 0;
    }

    // 3. 初始化各腿（打开 CAN + 电机清错/增益 + 在线探测）
    int enabled_count = 0;
    for (int i = 1; i <= kLegCount; ++i)
    {
        if (!want[i])
            continue;
        if (initLeg(i, /*probe=*/true))
            ++enabled_count;
    }
    if (enabled_count == 0)
    {
        std::printf("[错误] 没有任何腿初始化成功，退出\n");
        return -1;
    }
    std::printf("[信息] %d 条腿就绪（KP=%d KD=%d，下发周期 %dms，反馈读取 %dms）\n",
                enabled_count, kPositionKp, kPositionKd,
                taihu::kLoopPeriodMs, kReadPeriodMs);
    printHelp();
    std::printf("------------------------------------------------------------\n");

    // 4. 启动线程：4 个总线线程（每腿独占 can0~can3）+ 1 个上位机线程。
    //    SocketCanInterface 无内部锁 ⇒ 每条 CAN 接口只允许一个线程访问；
    //    主线程只做键盘路由与原子缓存读写，绝不直接碰 CAN。
    std::vector<std::thread> bus_threads;
    for (int i = 0; i < kLegCount; ++i)
    {
        if (g_legs[i].enabled.load())
            bus_threads.emplace_back(busThread, std::ref(g_legs[i]));
    }
    std::thread host;
    if (!host_ifname.empty())
        host = std::thread(hostThread, host_ifname.c_str());
    else
        std::printf("[协议] 未配置上位机接口：仅键盘命令可用\n");

    // 5. 主循环：读命令 → 解析 → 路由（estop/resume 置标志；pos/stat 读缓存）
    while (true)
    {
        std::printf("> ");
        std::fflush(stdout);
        if (std::fgets(line, sizeof line, stdin) == nullptr)
            break; // EOF
        chomp(line);
        if (line[0] == '\0')
            continue;

        if (std::strcmp(line, "quit") == 0 || std::strcmp(line, "q") == 0)
            break;
        if (std::strcmp(line, "estop") == 0)
        {
            g_estop.store(true);         // 各总线线程下一 tick 抱闸并置锁定
            g_stream_armed.store(false); // 流模式一并退出
            g_stream_online.store(false);
            for (auto &l : g_legs)
                l.cmd_cv.notify_all(); // 唤醒可能在等命令的总线线程
            std::printf("[急停] 已请求：全部电机将抱闸并锁定（resume 解除）\n");
            continue;
        }
        if (std::strcmp(line, "resume") == 0)
        {
            if (!g_estop_locked.load())
                std::printf("[resume] 当前未处于急停锁定，无需解除\n");
            else
            {
                g_estop_locked.store(false);
                // 急停踩下时已抱闸（去使能），解除锁定后必须重新使能，
                // 否则电机仍抱闸、任何目标都不会被执行
                for (auto &l : g_legs)
                    if (l.enabled.load(std::memory_order_relaxed))
                        l.arm_req.store(true);
                for (auto &l : g_legs)
                    l.cmd_cv.notify_all();
                std::printf("[resume] 急停锁定已解除，正在重新使能（解抱闸）…\n");
            }
            continue;
        }
        if (std::strcmp(line, "arm") == 0)
        {
            // arm 兼作解除锁定：锁定期间 busThread 不走使能分支，先清锁
            if (g_estop_locked.exchange(false))
                std::printf("[使能] 顺带解除急停锁定\n");
            int n = 0;
            for (auto &l : g_legs)
                if (l.enabled.load(std::memory_order_relaxed))
                {
                    l.arm_req.store(true);
                    ++n;
                }
            for (auto &l : g_legs)
                l.cmd_cv.notify_all();
            std::printf("[使能] 已请求解除 %d 条腿的抱闸（各腿总线线程执行）\n", n);
            continue;
        }
        if (std::strcmp(line, "disarm") == 0)
        {
            // 去使能 = 抱闸。置急停请求复用同一路径（抱闸 + 锁定 + 清命令）。
            g_estop.store(true);
            g_stream_armed.store(false);
            g_stream_online.store(false);
            for (auto &l : g_legs)
                l.cmd_cv.notify_all();
            std::printf("[去使能] 已请求：全部电机抱闸并锁定（arm 或 resume 解除）\n");
            continue;
        }
        if (std::strcmp(line, "help") == 0 || std::strcmp(line, "h") == 0)
        {
            printHelp();
            continue;
        }
        if (std::strcmp(line, "clear") == 0)
        {
            // 清除软限位报警锁定（与上位机 [0x03] 等价）
            if (!g_limit_locked.load())
                std::printf("[软限位] 当前未处于报警锁定，无需清除\n");
            else
                clearLimitAlarm();
            continue;
        }
        if (std::strcmp(line, "stat") == 0)
        {
            std::printf("[流状态] 模式=%s 在线=%s 急停锁定=%s 软限位锁定=%s\n",
                        g_stream_armed.load() ? "流模式" : "空闲",
                        g_stream_online.load() ? "是" : "否",
                        g_estop_locked.load() ? "是" : "否",
                        g_limit_locked.load() ? "是" : "否");
            if (g_limit_locked.load())
            {
                const uint16_t aid = g_limit_alarm_id.load();
                const uint8_t aside = g_limit_alarm_side.load();
                if (isJointId(aid))
                    std::printf("         报警关节 ID %u（腿%d %s，撞%s），"
                                "累计报警 %u 次\n",
                                aid, idToLeg(aid) + 1, kJointNames[idToJoint(aid)],
                                (aside == kLimitSideMax) ? "上限" : "下限",
                                g_limit_alarm_count.load());
            }
            for (int i = 0; i < kLegCount; ++i)
            {
                const Leg &l = g_legs[i];
                if (!l.enabled.load(std::memory_order_relaxed))
                {
                    std::printf("  腿%d（%s）：未启用\n", i + 1, l.ifname.c_str());
                    continue;
                }
                std::printf("  腿%d（%s）：收帧 %u，钳位/丢弃 %u，使能=%s，电机在线",
                            i + 1, l.ifname.c_str(),
                            l.rx_frames.load(), l.dropped.load(),
                            l.armed.load(std::memory_order_relaxed) ? "是" : "否");
                for (int j = 0; j < kJointCount; ++j)
                    std::printf(" %s%s", kJointNames[j],
                                l.motor_online[j].load(std::memory_order_relaxed) ? "✓" : "×");
                std::printf("\n");
            }
            continue;
        }

        // pos：主线程直接读 10Hz 反馈缓存（零总线流量，流模式中也随时可用）
        {
            char pos_line[256];
            std::strncpy(pos_line, line, sizeof pos_line - 1);
            pos_line[sizeof pos_line - 1] = '\0';
            int pos_leg = 0;
            if (tryParsePos(pos_line, pos_leg))
            {
                printPos(pos_leg);
                continue;
            }
        }

        Command cmd;
        if (!parseCommand(line, cmd))
            continue;

        // 未启用的腿：直接提示（不入队）
        if (!g_legs[cmd.leg - 1].enabled.load())
        {
            std::printf("[腿%d] 未启用（%s）\n", cmd.leg,
                        g_legs[cmd.leg - 1].ifname.c_str());
            continue;
        }
        // 急停锁定 / 软限位锁定 / 流模式：主线程即时提示（总线线程同样会拒绝，双保险）
        if (g_estop_locked.load())
        {
            std::printf("[腿%d][拒绝] 急停锁定中，move/goto 无效（resume 解除）\n",
                        cmd.leg);
            continue;
        }
        if (g_limit_locked.load())
        {
            std::printf("[腿%d][拒绝] 软限位报警锁定中，move/goto 无效"
                        "（clear 或上位机 [0x03] 解除）\n",
                        cmd.leg);
            continue;
        }
        if (g_stream_armed.load())
        {
            std::printf("[腿%d][暂缓] 流模式进行中，move/goto 暂缓"
                        "（上位机停发 1s 或发 0x02 退出流模式）\n",
                        cmd.leg);
            continue;
        }

        // 路由到该腿的命令队列（对应总线线程执行）
        {
            Leg &leg = g_legs[cmd.leg - 1];
            std::lock_guard<std::mutex> lock(leg.cmd_mutex);
            leg.cmd_queue.push(cmd);
        }
        g_legs[cmd.leg - 1].cmd_cv.notify_all();
    }

    // 6. 退出清理：通知全部线程退出 → 总线线程各自抱闸（退出前 stop）→
    //    join → 关闭总线（每条总线仍由其总线线程独占，无并发访问）
    g_running.store(false);
    for (auto &l : g_legs)
        l.cmd_cv.notify_all();
    for (auto &t : bus_threads)
        t.join();
    if (host.joinable())
        host.join();
    for (int i = 0; i < kLegCount; ++i)
    {
        if (g_legs[i].enabled.load())
            g_legs[i].can.close();
    }
    std::printf("[信息] 已退出：全部电机停止抱闸，总线关闭\n");
    return 0;
}
