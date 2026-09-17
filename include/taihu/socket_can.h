#ifndef TAIHU_SOCKET_CAN_H
#define TAIHU_SOCKET_CAN_H

#include "taihu/can_interface.h"

namespace taihu {

/**
 * @brief 基于 Linux SocketCAN 的 CAN 接口实现。
 *
 * 适用于达妙 USB2CAN 在 Linux 下以 SocketCAN (如 can0) 形式工作的场景。
 * 若改用达妙官方 SDK，则另写一个实现类继承 CanInterface 并替换使用。
 */
class SocketCanInterface : public CanInterface {
public:
    ~SocketCanInterface() override;

    bool open(const char* device, int bitrate) override;
    void close() override;
    bool send(const CanFrame& frame) override;
    bool receive(CanFrame& frame, int timeout_ms = 0) override;

private:
    int sock_ = -1;
};

} // namespace taihu

#endif // TAIHU_SOCKET_CAN_H
