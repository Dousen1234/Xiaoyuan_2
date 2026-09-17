/**
 * @file set_low_voltage.cpp
 * @brief 调整低压阈值工具入口：调用 taihu::setLowVoltageThreshold。
 *
 * 参考官方文档《低压阈值调整》，流程：读母线电压 → 读旧阈值 →
 * 设置新阈值 → 保存到 Flash → 读回确认。设置后需断电重新上电生效。
 *
 * 用法: ./taihu_motor_tools_set_low_voltage <低压阈值V> [CAN ID 默认1]
 * 示例: ./taihu_motor_tools_set_low_voltage 24 2   # 把 ID=2 电机低压阈值设为 24V
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "taihu/config.h"
#include "taihu/damiao_usb2can.h"
#include "taihu/taihu_tools.h"

namespace {


} // namespace

constexpr const char* kSerialDevice = "/dev/ttyACM0"; // 串口设备（每条总线独立配置）

int main(int argc, char* argv[]) {
    using namespace taihu;

    if (argc < 2) {
        std::printf("用法: %s <低压阈值V> [CAN ID 默认1]\n", argv[0]);
        std::printf("示例: %s 24 2   # 把 ID=2 电机低压阈值设为 24V\n", argv[0]);
        return -1;
    }

    const int32_t voltage_v = static_cast<int32_t>(std::strtol(argv[1], nullptr, 0));
    uint32_t can_id = 1;
    if (argc >= 3) {
        can_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 0));
    }

    DamiaoUsb2CanInterface can;
    if (!can.open(kSerialDevice, 0)) {
        std::printf("[错误] 打开串口 %s 失败\n", kSerialDevice);
        return -1;
    }

    const bool ok = setLowVoltageThreshold(can, can_id, voltage_v);
    can.close();

    if (ok) {
        std::printf("[提示] 请断电重新上电使设置生效\n");
    }
    return ok ? 0 : -1;
}
