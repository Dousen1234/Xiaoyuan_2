/**
 * @file reset_factory.cpp
 * @brief 恢复出厂设置工具入口：调用 taihu::resetFactory。
 * 用法: ./taihu_motor_tools_reset_factory [当前ID 默认1]
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "taihu/config.h"
#include "taihu/damiao_usb2can.h"
#include "taihu/taihu_tools.h"

namespace {
}

constexpr const char* kSerialDevice = "/dev/ttyACM0"; // 串口设备（每条总线独立配置）

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

    const bool ok = resetFactory(can, can_id);
    can.close();
    return ok ? 0 : -1;
}
