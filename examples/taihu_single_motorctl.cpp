/**
 * @file taihu_single_motorctl.cpp
 * @brief 单电机控制示例：交互式输入电机参数与控制模式。
 *
 * 运行时交互输入：
 *   - 电机 CAN ID 与减速比；
 *   - 控制模式：1 位置控制 / 2 正弦轨迹测试；
 *   - 模式 1 还需输入目标角度（度）与完成时间（秒），
 *     用五次多项式轨迹在指定时间内平滑到达目标角度。
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "taihu/config.h"
#include "taihu/damiao_usb2can.h"
#include "taihu/joint_module.h"

namespace {

constexpr const char* kSerialDevice = "/dev/ttyACM0"; // 串口设备（每条总线独立配置）

// —— 位置环增益（需按实际负载调试） ——
constexpr int32_t kPositionKp = 4000;
constexpr int32_t kPositionKd = 60;

// —— 模式 2：正弦轨迹参数 ——
constexpr double kAmplitude        = 0.5;   ///< 正弦幅度 (rad)
constexpr double kFrequency        = 0.5;   ///< 频率 (Hz)
constexpr double kTotalDurationSec = 10.0;  ///< 总运行时长 (s)

/// 五次多项式平滑函数（quintic smoothstep）。
/// 满足 s(0)=0、s(1)=1，且一阶、二阶导数在两端均为 0，
/// 用于位置轨迹规划，保证位置、速度、加速度连续、轨迹平滑。
double quinticSmoothstep(double t) {
    return 6.0 * t * t * t * t * t - 15.0 * t * t * t * t + 10.0 * t * t * t;
}

// 模式 1：位置控制（五次多项式轨迹）
bool runPositionMode(taihu::CanInterface& can, uint32_t can_id, double gear_ratio) {
    using namespace taihu;

    JointModule joint(can, can_id, can_id);
    joint.setGearRatio(gear_ratio);

    joint.clearError();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    joint.setPositionKp(kPositionKp);
    joint.setPositionKd(kPositionKd);

    // 输入目标角度（度）与完成时间（秒）
    double angle_deg = 0.0;
    double duration_sec = 0.0;
    std::printf("请输入目标角度（度，逆时针为正）：");
    std::scanf("%lf", &angle_deg);
    std::printf("请输入完成时间（秒）：");
    std::scanf("%lf", &duration_sec);
    if (duration_sec <= 0.0) {
        std::printf("[错误] 完成时间必须为正数\n");
        return false;
    }

    const double target_rad = angle_deg * kDegToRad;

    // 读取当前位置作为轨迹起点
    double pos_start = 0.0;
    if (!joint.readPosition(pos_start)) {
        std::printf("[错误] 读取当前位置失败\n");
        return false;
    }

    std::printf("[信息] 位置控制：从 %.3f rad 到 %.3f rad，时长 %.1f 秒（五次多项式轨迹）\n",
                pos_start, target_rad, duration_sec);

    // 五次多项式轨迹：在 duration_sec 内从 pos_start 平滑过渡到 target_rad
    const auto start = std::chrono::steady_clock::now();
    while (true) {
        const double t = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start).count();
        if (t >= duration_sec) break;

        const double s      = quinticSmoothstep(t / duration_sec);
        const double target = pos_start + (target_rad - pos_start) * s;
        if (!joint.setTargetPosition(target)) {
            std::printf("[错误] 发送位置指令失败\n");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
    }

    // 最终精确到位
    joint.setTargetPosition(target_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    joint.stop();
    std::printf("[信息] 位置控制结束\n");
    return true;
}

// 模式 2：正弦轨迹测试
bool runSinusoidal(taihu::CanInterface& can, uint32_t can_id, double gear_ratio) {
    using namespace taihu;

    JointModule joint(can, can_id, can_id);
    joint.setGearRatio(gear_ratio);

    joint.clearError();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    joint.setPositionKp(kPositionKp);
    joint.setPositionKd(kPositionKd);

    std::printf("[信息] 开始正弦轨迹测试...\n");
    const auto start = std::chrono::steady_clock::now();

    while (true) {
        const double t = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start).count();
        if (t >= kTotalDurationSec) break;   // 运行满指定时长后退出

        const double target = kAmplitude * std::sin(2.0 * M_PI * kFrequency * t);
        if (!joint.setTargetPosition(target)) {
            std::printf("[错误] 发送位置指令失败\n");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
    }

    joint.stop();
    std::printf("[信息] 正弦轨迹测试结束\n");
    return true;
}

} // namespace

int main() {
    using namespace taihu;

    // 1. 打开 CAN 设备
    DamiaoUsb2CanInterface can;
    if (!can.open(kSerialDevice, 0)) {
        std::printf("[错误] 打开串口 %s 失败\n", kSerialDevice);
        return -1;
    }

    // 2. 输入电机参数
    uint32_t can_id = 1;
    double gear_ratio = 81.0;

    std::printf("========================================\n");
    std::printf("  单电机控制 demo\n");
    std::printf("========================================\n");
    std::printf("请输入电机 CAN ID（默认 1）：");
    if (std::scanf("%u", &can_id) != 1) can_id = 1;
    std::printf("请输入减速比（默认 81）：");
    if (std::scanf("%lf", &gear_ratio) != 1) gear_ratio = 81.0;

    std::printf("----------------------------------------\n");
    std::printf("  1 —— 位置控制（五次多项式轨迹）\n");
    std::printf("  2 —— 正弦轨迹测试\n");
    std::printf("请输入 1 或 2 选择控制模式：");

    int choice = 0;
    std::scanf("%d", &choice);

    // 3. 按选择执行
    bool ok = false;
    if (choice == 1) {
        ok = runPositionMode(can, can_id, gear_ratio);
    } else if (choice == 2) {
        ok = runSinusoidal(can, can_id, gear_ratio);
    } else {
        std::printf("[错误] 无效输入（只能输入 1 或 2）\n");
    }

    can.close();
    return ok ? 0 : -1;
}
