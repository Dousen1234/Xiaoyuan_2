#ifndef TAIHU_CAN_INTERFACE_H
#define TAIHU_CAN_INTERFACE_H

#include <cstdint>
#include <cstddef>

namespace taihu {

/**
 * @brief CAN 帧结构体
 */
struct CanFrame {
    uint32_t id = 0;                 ///< CAN 标识符（标准 11 位或扩展 29 位）
    uint8_t  data[8] = {0};          ///< 数据域，最多 8 字节
    uint8_t  dlc = 0;                ///< 数据长度（0~8）
    bool     is_extended = false;    ///< 是否为扩展帧
};

/**
 * @brief CAN 通信抽象接口。
 *
 * 上层关节模组控制逻辑只依赖此接口，具体的达妙 USB2CAN / SocketCAN
 * 驱动通过继承并实现本接口完成，实现与协议解耦。
 */
class CanInterface {
public:
    virtual ~CanInterface() = default;

    /**
     * @brief 打开并初始化 CAN 设备
     * @param device 设备名（如 SocketCAN 的 "can0"，或达妙 SDK 的设备句柄标识）
     * @param bitrate 波特率，单位 bit/s，本项目为 1000000 (1M)
     * @return 成功返回 true，失败返回 false
     */
    virtual bool open(const char* device, int bitrate) = 0;

    /// 关闭设备
    virtual void close() = 0;

    /**
     * @brief 发送一帧 CAN 数据
     * @return 成功返回 true
     */
    virtual bool send(const CanFrame& frame) = 0;

    /**
     * @brief 接收一帧 CAN 数据
     * @param frame 输出参数，存放接收到的帧
     * @param timeout_ms 阻塞等待超时，0 表示非阻塞，<0 表示无限阻塞
     * @return 成功接收返回 true，超时或失败返回 false
     */
    virtual bool receive(CanFrame& frame, int timeout_ms = 0) = 0;

    /**
     * @brief 清空接收缓冲区（丢弃残留的响应帧）。
     *
     * 写命令（如清除错误）发送后，电机可能返回响应帧残留在缓冲区，
     * 干扰后续读命令；读取前调用本方法可丢弃这些残留。
     */
    virtual void drain() {}
};

} // namespace taihu

#endif // TAIHU_CAN_INTERFACE_H
