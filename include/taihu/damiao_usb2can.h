#ifndef TAIHU_DAMIAO_USB2CAN_H
#define TAIHU_DAMIAO_USB2CAN_H

#include <cstdint>
#include <mutex>
#include <queue>
#include "taihu/can_interface.h"

namespace taihu {

#pragma pack(push, 1)
/**
 * @brief 达妙 USB2CAN 发送帧结构（30 字节）。
 * 通过虚拟串口（如 /dev/ttyACM0，波特率 921600）下发。
 */
struct DmSendFrame {
    uint8_t  header[2];       // 帧头 0x55 0xAA
    uint8_t  frame_len;       // 帧长 0x1E (30)
    uint8_t  cmd;             // 0x03: 非反馈 CAN 转发
    uint32_t send_times;      // 发送次数 = 1
    uint32_t time_interval;   // 时间间隔 = 10
    uint8_t  id_type;         // 0 标准帧 / 1 扩展帧
    uint32_t can_id;          // CAN ID（小端）
    uint8_t  frame_type;      // 0 数据帧 / 1 远程帧
    uint8_t  len;             // 数据长度 dlc
    uint8_t  id_acc;          // 0
    uint8_t  data_acc;        // 0
    uint8_t  data[8];         // CAN 数据域
    uint8_t  crc;             // 0（未校验）
};

/**
 * @brief 达妙 USB2CAN 接收帧结构（16 字节）。
 */
struct DmRecvFrame {
    uint8_t  header;          // 帧头 0xAA
    uint8_t  cmd;             // 0x11: 接收成功
    uint8_t  can_dlc : 6;     // 数据长度
    uint8_t  can_ide : 1;     // 0 标准帧 / 1 扩展帧
    uint8_t  can_rtr : 1;     // 0 数据帧 / 1 远程帧
    uint32_t can_id;          // CAN ID（小端）
    uint8_t  data[8];         // CAN 数据域
    uint8_t  frame_end;       // 帧尾 0x55
};
#pragma pack(pop)

/**
 * @brief 达妙 USB2CAN 的 CanInterface 实现。
 *
 * 设备以虚拟串口形式接入 Linux，默认 /dev/ttyACM0，波特率 921600。
 * CAN 总线波特率（本项目 1M）由 USB2CAN 设备内部配置。
 */
class DamiaoUsb2CanInterface : public CanInterface {
public:
    ~DamiaoUsb2CanInterface() override;

    /**
     * @param device  串口设备路径（如 "/dev/ttyACM0"）
     * @param bitrate 忽略（串口波特率固定 921600；CAN 波特率由设备配置）
     */
    bool open(const char* device, int bitrate) override;
    void close() override;
    bool send(const CanFrame& frame) override;
    bool receive(CanFrame& frame, int timeout_ms = 0) override;
    void drain() override;

    /**
     * @brief 诊断用：读取并打印串口原始字节（含帧头/CMD/canId），
     *        用于排查 CAN 通信层问题。
     * @param timeout_ms 等待超时
     * @return 实际读到的字节数
     */
    int dumpRaw(int timeout_ms = 300);

private:
    int fd_ = -1;

    // 接收字节缓冲队列，用于帧同步（按 0xAA 帧头对齐 16 字节）
    std::queue<uint8_t> rx_queue_;

    // 串口收发互斥锁（多线程共享同一设备时保证帧不交错）
    mutable std::mutex mutex_;

    bool readBytes(uint8_t* buf, size_t len, int timeout_ms);
};

} // namespace taihu

#endif // TAIHU_DAMIAO_USB2CAN_H
