/**
 * @file change_id.cpp
 * @brief 修改 CAN ID 工具入口：调用 taihu::changeCanId。
 * 用法: ./taihu_motor_tools_change_id <新ID 1~127> [当前ID 默认1]
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

    if (argc < 2) {
        std::printf("用法: %s <新ID 1~127> [当前ID 默认1]\n", argv[0]);
        return -1;
    }

    const uint32_t new_id = static_cast<uint32_t>(std::strtoul(argv[1], nullptr, 0));
    uint32_t cur_id = 1;
    if (argc >= 3) {
        cur_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 0));
    }

    if (new_id < 1 || new_id > 127) {
        std::printf("[错误] 新 ID 必须在 1~127 之间\n");
        return -1;
    }

    DamiaoUsb2CanInterface can;
    if (!can.open(kSerialDevice, 0)) {
        std::printf("[错误] 打开串口 %s 失败\n", kSerialDevice);
        return -1;
    }

    const bool ok = changeCanId(can, cur_id, new_id);
    can.close();
    return ok ? 0 : -1;
}
