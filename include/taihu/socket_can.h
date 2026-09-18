#ifndef TAIHU_SOCKET_CAN_H
#define TAIHU_SOCKET_CAN_H

#include "taihu/can_interface.h"

namespace taihu {

/**
 * @brief 基于 Linux SocketCAN 的 CAN 接口实现。
 *
 * 适用于鲲弘 KH-UCANFDX6-Mini 等 USB-CANFD 设备：其 Linux 驱动 (kcan)
 * 会把设备注册为 can0~can5 共 6 个标准 SocketCAN 网络接口，每个通道
 * 对应一路独立 CAN 总线，直接用本类打开即可。
 *
 * CAN 波特率由 `ip link set can0 type can bitrate 1000000` 配置，
 * 打开设备前需确保接口已 up（`ip link set can0 up`）。
 */
class SocketCanInterface : public CanInterface {
public:
    ~SocketCanInterface() override;

    bool open(const char* device, int bitrate) override;
    void close() override;
    bool send(const CanFrame& frame) override;
    bool receive(CanFrame& frame, int timeout_ms = 0) override;
    void drain() override;

private:
    int sock_ = -1;
};

} // namespace taihu

#endif // TAIHU_SOCKET_CAN_H
