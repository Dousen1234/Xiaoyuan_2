/**
 * @file taihu_motor_tools_openbreak.cpp
 * @brief 打开电机刹车（释放抱闸）并重新标零的工具。
 *
 * 流程：
 *   1. 按 CAN ID 打开电机刹车（使能 + 目标电流 0，电机可自由旋转）；
 *   2. 保持刹车打开，等待用户手动将电机旋转到目标位置；
 *   3. 用户按回车确认后，将当前位置保存为新的零位并写入 Flash；
 *   4. 关闭刹车（去使能，重新抱闸）。
 *
 * 用法: ./taihu_motor_tools_openbreak [CAN ID 默认1]
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "taihu/damiao_usb2can.h"
#include "taihu/joint_module.h"
#include "taihu/taihu_tools.h"

namespace {
constexpr const char* kSerialDevice = "/dev/ttyACM0"; // 串口设备（每条总线独立配置）
}

int main(int argc, char* argv[]) {
    using namespace taihu;

    uint32_t can_id = 1;
    if (argc >= 2) {
        can_id = static_cast<uint32_t>(std::strtoul(argv[1], nullptr, 0));
    }

    DamiaoUsb2CanInterface can;
    if (!can.open(kSerialDevice, 0)) {
        std::printf("[错误] 打开串口 %s 失败\n", kSerialDevice);
        return -1;
    }

    JointModule joint(can, can_id, can_id);

    // 1. 清除锁存错误（如欠压），确保可正常使能
    joint.clearError();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 2. 打开刹车：设置目标电流 0，进入电流模式（使能 + 零转矩），解除抱闸
    if (!joint.setTargetCurrent(0)) {
        std::printf("[错误] 打开刹车（使能）失败\n");
        can.close();
        return -1;
    }
    std::printf("[信息] 刹车已打开，电机可自由旋转，请手动旋转到目标位置\n");

    // 3. 等待用户确认
    std::printf("[提示] 旋转完成后按回车键确认保存零位...\n");
    std::getchar();

    // 4. 标零并保存 Flash（setZeroPosition 内部会先 stop 去使能，即关闭刹车抱闸）
    const bool ok = setZeroPosition(can, can_id);

    std::printf("[信息] 刹车已关闭（重新抱闸）\n");
    can.close();
    return ok ? 0 : -1;
}
