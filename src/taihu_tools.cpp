#include "taihu/taihu_tools.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "taihu/config.h"
#include "taihu/joint_module.h"

namespace taihu {

namespace {

/// 下发目标位置并等待电机到位。
bool moveAndWait(JointModule& joint, double target_rad) {
    if (!joint.setTargetPosition(target_rad)) {
        std::fprintf(stderr, "[错误] 发送目标位置失败\n");
        return false;
    }

    const auto start = std::chrono::steady_clock::now();
    while (true) {
        double position = 0.0;
        if (!joint.readPosition(position)) {
            std::fprintf(stderr, "[错误] 读取当前位置失败\n");
            return false;
        }

        if (std::fabs(position - target_rad) < kSettleThresholdRad) {
            return true;
        }

        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - start).count();
        if (elapsed_ms > kSettleTimeoutMs) {
            std::fprintf(stderr, "[错误] 到位超时（当前位置 %.3f rad）\n", position);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollPeriodMs));
    }
}

} // namespace

std::unique_ptr<JointModule> initMotor(CanInterface& can, const MotorConfig& cfg) {
    auto motor = std::make_unique<JointModule>(can, cfg.can_id, cfg.can_id);
    motor->setGearRatio(cfg.gear_ratio);
    motor->setPositionKp(cfg.kp);
    motor->setPositionKd(cfg.kd);
    return motor;
}

bool readMotorStatus(CanInterface& can, uint32_t can_id, double gear_ratio,
                     MotorStatus& status) {
    JointModule joint(can, can_id, can_id);
    joint.setGearRatio(gear_ratio);   // 减速比用于速度换算

    status.can_id = can_id;

    // 0x41 一次性读电流+速度+位置，减少读操作次数（高频采样关键）
    int32_t current_ma = 0;
    double  velocity_rad_s = 0.0;
    double  position_rad = 0.0;
    if (!joint.readCurrentVelocityPosition(current_ma, velocity_rad_s, position_rad)) {
        std::fprintf(stderr, "[错误] 读电流/速度/位置失败\n");
        return false;
    }
    status.current_ma     = current_ma;
    status.velocity_rad_s = velocity_rad_s;
    status.position_rad   = position_rad;

    // 电压不再记录（绘图已移除电压图）
    status.bus_voltage_v = 0;

    uint32_t error = 0;
    if (!joint.readErrorState(error)) { std::fprintf(stderr, "[错误] 读错误状态失败\n"); return false; }
    status.error_bits = error;

    return true;
}

void diagnose(CanInterface& can, uint32_t can_id) {
    JointModule joint(can, can_id, can_id);

    int32_t pos_cnt = 0;
    int32_t offset  = 0;
    uint32_t error  = 0;

    if (joint.readPositionCnt(pos_cnt)) {
        std::printf("[信息] 当前位置 = %d cnt (%.3f rad)\n",
                    pos_cnt, static_cast<double>(pos_cnt) * 2.0 * M_PI / 262144.0);
    } else {
        std::printf("[错误] 读取当前位置失败\n");
    }

    if (joint.readPositionOffset(offset)) {
        std::printf("[信息] 位置偏移 = %d cnt\n", offset);
    } else {
        std::printf("[错误] 读取位置偏移失败\n");
    }

    if (joint.readErrorState(error)) {
        std::printf("[信息] 错误状态 = 0x%X%s\n", error, (error == 0 ? "（无错误）" : ""));
    } else {
        std::printf("[错误] 读取错误状态失败\n");
    }
}

bool changeCanId(CanInterface& can, uint32_t cur_id, uint32_t new_id) {
    JointModule joint_old(can, cur_id, cur_id);

    joint_old.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (!joint_old.setMotorId(new_id)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    JointModule joint_new(can, new_id, new_id);
    joint_new.saveParams();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    return true;
}

bool resetFactory(CanInterface& can, uint32_t can_id) {
    JointModule joint(can, can_id, can_id);

    joint.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    joint.resetFactory();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 恢复出厂后 CAN ID 回到默认 1，用 ID 1 保存
    JointModule joint_default(can, 1, 1);
    joint_default.saveParams();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    return true;
}

bool setZeroPosition(CanInterface& can, uint32_t can_id) {
    JointModule joint(can, can_id, can_id);

    joint.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    int32_t pos_cnt = 0;
    if (!joint.readPositionCnt(pos_cnt)) return false;

    int32_t offset = 0;
    if (!joint.readPositionOffset(offset)) return false;

    const int32_t abs_pos = pos_cnt + offset;   // 编码器绝对位置 = 相对位置 + 偏移

    if (!joint.setPositionOffset(abs_pos)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    joint.saveParams();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::printf("[信息] 已标零：当前位置 %d cnt -> 0\n", pos_cnt);
    return true;
}

bool setLowVoltageThreshold(CanInterface& can, uint32_t can_id, int32_t voltage_v) {
    JointModule joint(can, can_id, can_id);

    // 1. 读母线电压
    int32_t bus_v = 0;
    if (!joint.readBusVoltage(bus_v)) {
        std::printf("[错误] 读取母线电压失败\n");
        return false;
    }
    std::printf("[信息] 当前母线电压 = %d V\n", bus_v);

    // 2. 读旧低压阈值
    int32_t old_threshold = 0;
    if (!joint.readLowVoltageThreshold(old_threshold)) {
        std::printf("[错误] 读取低压阈值失败\n");
        return false;
    }
    std::printf("[信息] 当前低压阈值 = %d V\n", old_threshold);

    // 3. 设置新低压阈值
    if (!joint.setLowVoltageThreshold(voltage_v)) {
        std::printf("[错误] 设置低压阈值失败\n");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 4. 保存到 Flash
    joint.saveParams();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 5. 读回确认
    int32_t new_threshold = 0;
    if (!joint.readLowVoltageThreshold(new_threshold)) {
        std::printf("[错误] 确认低压阈值失败\n");
        return false;
    }
    std::printf("[信息] 已设置低压阈值 = %d V（重启生效）\n", new_threshold);

    return new_threshold == voltage_v;
}

bool rotateFromZero(CanInterface& can, uint32_t can_id, double angle_deg,
                    int32_t kp, int32_t kd) {
    JointModule joint(can, can_id, can_id);

    // 1. 清除锁存错误并配置位置环增益
    joint.clearError();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    joint.setPositionKp(kp);
    joint.setPositionKd(kd);

    // 2. 回到零位
    std::printf("[信息] 回到零位...\n");
    if (!moveAndWait(joint, 0.0)) {
        joint.stop();
        return false;
    }

    // 3. 旋转指定角度（逆时针为正）
    const double target_rad = angle_deg * kDegToRad;
    std::printf("[信息] 旋转 %.1f° ...\n", angle_deg);
    if (!moveAndWait(joint, target_rad)) {
        joint.stop();
        return false;
    }

    // 4. 停止电机
    joint.stop();
    std::printf("[信息] 完成\n");
    return true;
}

} // namespace taihu
