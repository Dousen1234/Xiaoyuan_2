/**
 * @file mpt_encoder.h
 * @brief 金钢 MPT 磁编码器 RS485 驱动（手册 v0.6，接口 R / D 两种协议自动适配）。
 *
 * 支持的协议形态（由构造参数 protocol 决定，或用 probe() 自动探测）：
 *   - Protocol::RS485  ：接口代号 R。请求 1 字节（0x31 / 0x64 / 0x30），无地址字段，
 *                        总线上只能挂 1 台编码器。
 *   - Protocol::BUS    ：接口代号 D（高速总线）。请求 1 字节 = 奇校验(b7) | 操作类型(b6~b5)
 *                        | 设备地址(b4~b0，出厂 0x1F)；两台可挂同一对 A/B（需先改不同 ID）。
 *
 * UART 格式（两种协议相同）：8 数据位、无奇偶校验、1 停止位，LSB 优先。
 * 波特率：型号代号 M=2,500,000（默认）/ V=5,000,000 / A=115,200。
 *
 * 数据解析（以 24 位单圈、无多圈型号为例）：
 *   - RS485 款 0x31 返回 5 字节：A2 A1 A0 S CRC（CRC-8：x8+x7+x4+x2+x+1，查表）
 *   - RS485 款 0x64 返回 6 字节：A2 A1 A0 S?（手册 24 位行为 5 字节 + S 共 6 字节）
 *   - BUS 款“获取信息”返回：S A0 A1 A2（24 位）+ CRC
 *   角度（°）= raw / 2^24 * 360
 *
 * 用法：独立于 CAN 电机总线（走 /dev/ttyUSB* 串口），与任意主控制程序互不影响。
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace taihu {

/// RS485 串口封装（8N1，供 MPT 编码器使用）
class Rs485Port {
public:
    Rs485Port() = default;
    ~Rs485Port();

    /// 打开串口设备并配置波特率（8N1 无流控）。失败返回 false 并写 err。
    bool open(const std::string& device, int baudrate, std::string* err = nullptr);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    /// 发送一字节请求
    bool writeByte(uint8_t b);

    /// 带超时读 n 字节（返回实际读到的字节数，超时返回 < n）
    int readBytes(uint8_t* buf, int n, int timeout_ms);

    /// 清空输入缓冲（丢弃残留帧）
    void drain();

private:
    int fd_ = -1;
};

/// MPT 磁编码器驱动
class MptEncoder {
public:
    /// 接口协议类型（型号字符串末位）
    enum class Protocol { RS485, BUS };

    /// 输出参数（型号中的“输出参数”段）
    enum class Output {
        Single24,     ///< 24  位单圈（24）
        Multi24,      ///< 24  位单圈 + 普通多圈（24M）
        Battery24,    ///< 24  位单圈 + 电池多圈（24BM）
        Flash24,      ///< 24  位单圈 + 闪存多圈（24FM）
    };

    struct Sample {
        uint32_t raw = 0;              ///< 单圈原始值（24 位）
        double angle_deg = 0.0;        ///< 单圈角度（度，0~360）
        int32_t turns = 0;             ///< 多圈圈数（多圈型号有效）
        bool error = false;            ///< 错误位（置位时数据不可信）
        bool warning = false;          ///< 警告位（置位时数据仍可用）
        uint8_t status = 0;            ///< 状态位字节
        std::chrono::steady_clock::time_point stamp{};
    };

    /// @param proto   协议形态；传 RS485 时 addr 忽略
    /// @param addr    BUS 款设备地址（出厂 0x1F，0~31）
    /// @param output  输出参数型号（决定返回帧长度与多圈解析）
    MptEncoder(Rs485Port& port, Protocol proto, uint8_t addr = 0x1F,
               Output output = Output::Single24);

    // —— 配置 ——
    void setAddress(uint8_t addr) { addr_ = addr & 0x1F; }
    uint8_t address() const { return addr_; }
    Protocol protocol() const { return proto_; }

    /// 总线上逐个探测：先按 BUS 全地址 0x1F 问询，再退回 RS485 0x31。
    /// 探测期间会尝试读取，找到则返回协议类型并存 addr。
    static Protocol probe(Rs485Port& port, uint8_t* found_addr, Output out,
                          int tries = 3);

    // —— 读取 ——
    /// 读一次位置（0x31 / BUS 获取信息）。成功返回 true 并填充 s。
    bool readPosition(Sample& s);

    /// 读一次位置+状态（0x64；BUS 款即“获取信息”，含状态字节）。
    bool readPositionStatus(Sample& s);

    // —— 设置 ——
    /// 设置零位：RS485 款需 0x31 / 0x30 交替 10 次；BUS 款操作类型 00/01 交替 10 组。
    /// 约 50ms 生效期间编码器不响应任何命令，函数内部已等待。
    bool setZero();

    /// BUS 款改设备地址（发送地址序列 X,2,4,...,16,Y；成功后 addr_ 更新）。
    /// RS485 款无地址概念，返回 false。
    bool setBusAddress(uint8_t new_addr);

    // —— 工具 ——
    static double rawToDeg(uint32_t raw) {
        return static_cast<double>(raw) / 16777216.0 * 360.0;  // 2^24
    }
    static constexpr uint8_t kFactoryAddr = 0x1F;

private:
    /// 发送请求并读回一帧（返回帧长，0=失败）
    int transact(uint8_t request, uint8_t* reply, int reply_len, int timeout_ms = 20);

    /// 解析“获取信息”类回复
    bool parseReply(const uint8_t* reply, int len, Sample& s);

    static uint8_t crc8(const uint8_t* buf, int len);   ///< x8+x7+x4+x2+x+1 查表

    Rs485Port& port_;
    Protocol proto_;
    uint8_t addr_ = kFactoryAddr;
    Output output_ = Output::Single24;
};

} // namespace taihu
