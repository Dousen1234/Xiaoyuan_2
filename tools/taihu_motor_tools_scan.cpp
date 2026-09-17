/**
 * @file taihu_motor_tools_scan.cpp
 * @brief 扫描 CAN 总线上所有电机 ID。
 *
 * 遍历 ID 1~127，向每个 ID 发送「获取当前位置」读命令，
 * 打印所有有响应的设备，用于确认总线上各电机的实际 CAN ID。
 *
 * 用法: ./taihu_motor_tools_scan
 */

#include <cstdint>
#include <cstdio>

#include "taihu/damiao_usb2can.h"
#include "taihu/joint_module.h"

namespace {

constexpr const char* kSerialDevice = "/dev/ttyACM0";

} // namespace

int main() {
    using namespace taihu;

    DamiaoUsb2CanInterface can;
    if (!can.open(kSerialDevice, 0)) {
        std::printf("[错误] 打开串口 %s 失败\n", kSerialDevice);
        return -1;
    }

    std::printf("[信息] 开始扫描 CAN ID 1~127 ...\n");

    int found = 0;
    for (uint32_t id = 1; id <= 127; ++id) {
        JointModule joint(can, id, id);
        int32_t cnt = 0;
        if (joint.readPositionCnt(cnt)) {
            std::printf("[发现] ID=%u，位置=%d cnt (%.3f rad)\n",
                        id, cnt, static_cast<double>(cnt) * 2.0 * 3.14159265358979323846 / 262144.0);
            ++found;
        }
    }

    std::printf("[信息] 扫描结束，共发现 %d 个设备\n", found);
    can.close();
    return 0;
}
