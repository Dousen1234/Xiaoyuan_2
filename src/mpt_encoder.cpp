#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

/**
 * @file mpt_encoder.cpp
 * @brief 金钢 MPT 磁编码器 RS485 驱动实现（手册 v0.6）。
 *
 * 串口配置使用内核结构 termios2（<asm/termbits.h>）而非 glibc <termios.h>：
 * 唯有前者支持 BOTHER 任意波特率（2.5M / 5M）。两者互斥，本文件不得再含 glibc termios.h。
 */

#include "taihu/mpt_encoder.h"

#include <fcntl.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>   // struct termios / termios2 / BOTHER / B115200 / CS8 ...
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>

// glibc <termios.h> 才有的清空常量（内核侧等价取值）
constexpr int kTciflush = 0;   // TCIFUSH: 清输入队列
constexpr int kTcioflush = 2;  // TCIOFLUSH: 清收发队列

namespace taihu {

// =====================================================================
// CRC-8 查表（多项式 x8+x7+x4+x2+x+1，来自手册附录）
// =====================================================================
static const std::array<uint8_t, 256> kCrc8Table = {
    0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
    0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C,
    0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
    0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 0x58, 0xCF,
    0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7,
    0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
    0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24,
    0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E,
    0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
    0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E,
    0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7,
    0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
    0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65,
    0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF,
    0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
    0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C,
};

uint8_t MptEncoder::crc8(const uint8_t* buf, int len) {
    uint8_t temp = *buf++;
    while (--len) {
        temp = *buf++ ^ kCrc8Table[temp];
    }
    return kCrc8Table[temp];
}

// =====================================================================
// Rs485Port：串口封装
// =====================================================================
Rs485Port::~Rs485Port() { close(); }

// 把波特率写入 termios2：标准值用 B 常量，其它走 BOTHER 自定义。成功返回 true。
static bool baudToSpeed(int baud, termios2& t2) {
    speed_t sp;
    switch (baud) {
        case 115200:  sp = B115200; break;
        case 57600:   sp = B57600;  break;
        case 38400:   sp = B38400;  break;
        case 19200:   sp = B19200;  break;
        case 9600:    sp = B9600;   break;
        default: {  // 2.5M / 5M 等：BOTHER 任意波特率
            t2.c_cflag &= ~CBAUD;
            t2.c_cflag |= BOTHER;
            t2.c_ispeed = baud;
            t2.c_ospeed = baud;
            return true;
        }
    }
    t2.c_cflag &= ~CBAUD;
    t2.c_cflag |= (sp & CBAUD);
    t2.c_ispeed = 0;
    t2.c_ospeed = 0;
    return true;
}

// 串口配置全走 termios2 + ioctl(TCSETS2)，支持 BOTHER 任意波特率（2.5M/5M）。
bool Rs485Port::open(const std::string& device, int baudrate, std::string* err) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        if (err) *err = std::strerror(errno);
        return false;
    }

    // 非阻塞打开后切回阻塞模式（配 select 超时）
    int flags = ::fcntl(fd_, F_GETFL);
    if (flags >= 0) ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    termios2 t2{};
    if (::ioctl(fd_, TCGETS2, &t2) != 0) {
        if (err) *err = std::strerror(errno);
        close();
        return false;
    }
    // raw 8N1
    t2.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    t2.c_oflag &= ~OPOST;
    t2.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    t2.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    t2.c_cflag |= (CS8 | CLOCAL | CREAD);
    t2.c_cc[VMIN]  = 0;               // read 立即返回（配 select 超时）
    t2.c_cc[VTIME] = 0;

    if (baudToSpeed(baudrate, t2)) {  // 标准或 BOTHER 均已写入 t2
        if (::ioctl(fd_, TCSETS2, &t2) != 0) {
            if (err) *err = std::strerror(errno);
            close();
            return false;
        }
    } else {
        if (err) *err = "不支持的标准波特率";
        close();
        return false;
    }
    ::ioctl(fd_, TCFLSH, kTcioflush);
    return true;
}

void Rs485Port::close() {
    if (fd_ >= 0) {
        ::ioctl(fd_, TCFLSH, kTcioflush);
        ::close(fd_);
        fd_ = -1;
    }
}

bool Rs485Port::writeByte(uint8_t b) {
    return ::write(fd_, &b, 1) == 1;
}

int Rs485Port::readBytes(uint8_t* buf, int n, int timeout_ms) {
    if (fd_ < 0 || n <= 0) return 0;
    int total = 0;
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);
    while (total < n) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);
        auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                        deadline - std::chrono::steady_clock::now()).count();
        if (left <= 0) break;
        timeval tv{};
        tv.tv_sec  = static_cast<long>(left) / 1000;
        tv.tv_usec = static_cast<long>(left) % 1000 * 1000;
        int r = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (r <= 0) break;  // 超时或错误
        int got = ::read(fd_, buf + total, n - total);
        if (got <= 0) break;
        total += got;
    }
    return total;
}

void Rs485Port::drain() {
    if (fd_ < 0) return;
    ::ioctl(fd_, TCFLSH, kTciflush);
    uint8_t junk[64];
    while (::read(fd_, junk, sizeof(junk)) > 0) {}
}

// =====================================================================
// MptEncoder
// =====================================================================
MptEncoder::MptEncoder(Rs485Port& port, Protocol proto, uint8_t addr, Output output)
    : port_(port), proto_(proto), addr_(addr & 0x1F), output_(output) {}

int MptEncoder::transact(uint8_t request, uint8_t* reply, int reply_len, int timeout_ms) {
    port_.drain();
    if (!port_.writeByte(request)) return 0;
    return port_.readBytes(reply, reply_len, timeout_ms);
}

// —— 请求字节构造 ——
// RS485 款：直接命令字节 0x31 / 0x64 / 0x30
// BUS 款：b7=奇校验位（手册未给出校验算法，置 0），b6~b5=操作类型，b4~b0=地址
static uint8_t makeBusRequest(uint8_t op_type, uint8_t addr) {
    return static_cast<uint8_t>((op_type << 5) | (addr & 0x1F));
}

// —— BUS 款“获取信息”返回（24 位单圈）长度：1（回显请求）+ 1（S）+ 3（A0A1A2）+ 1（CRC）= 6
// —— RS485 款 0x31（24）返回：A2 A1 A0 S? … 按手册表格为 5 字节（A2 A1 A0 S CRC）
//    实际上手册 24 位、0x31 行：N=5，字节为 A2 A1 A0 S CRC
//    （0x31 无状态字节；0x64 N=5 为 A2 A1 A0 S CRC —— 同长但含 S；为稳妥两种都按 5 字节解析）

MptEncoder::Protocol MptEncoder::probe(Rs485Port& port, uint8_t* found_addr, Output out, int tries) {
    // 1) 先试 BUS：全 0x1F 地址“获取信息”，期待 6 字节且 CRC 通过
    for (int i = 0; i < tries; ++i) {
        MptEncoder e(port, Protocol::BUS, kFactoryAddr, out);
        uint8_t reply[16] = {0};
        int n = e.transact(makeBusRequest(0b00, kFactoryAddr), reply, 6);
        if (n >= 6 && reply[0] == makeBusRequest(0b00, kFactoryAddr)) {
            // BUS 回显请求字节；CRC 覆盖 B1..B(n-1)（S A0 A1 A2）
            if (crc8(reply + 1, 4) == reply[5]) {
                if (found_addr) *found_addr = kFactoryAddr;
                return Protocol::BUS;
            }
        }
        // 尝试地址 0x01/0x02（用户可能已改过）
        for (uint8_t a = 1; a < 0x1F; ++a) {
            MptEncoder e2(port, Protocol::BUS, a, out);
            uint8_t r2[16] = {0};
            int m = e2.transact(makeBusRequest(0b00, a), r2, 6);
            if (m >= 6 && r2[0] == makeBusRequest(0b00, a) && crc8(r2 + 1, 4) == r2[5]) {
                if (found_addr) *found_addr = a;
                return Protocol::BUS;
            }
        }
    }
    // 2) 再试 RS485：发 0x31，期待 5 字节且 CRC 通过
    for (int i = 0; i < tries; ++i) {
        uint8_t reply[16] = {0};
        if (port.drain(), port.writeByte(0x31), true) {
            int n = port.readBytes(reply, 5, 20);
            if (n >= 5 && crc8(reply, 4) == reply[4]) {
                if (found_addr) *found_addr = 0;
                return Protocol::RS485;
            }
        }
    }
    return Protocol::RS485;  // 默认按 RS485 处理（调用方看 readPosition 是否成功）
}

bool MptEncoder::readPosition(Sample& s) {
    uint8_t reply[16] = {0};
    int n = 0;
    if (proto_ == Protocol::BUS) {
        n = transact(makeBusRequest(0b00, addr_), reply, 6);
        if (n < 6) return false;
        if (reply[0] != makeBusRequest(0b00, addr_)) return false;
        if (crc8(reply + 1, 4) != reply[5]) return false;
        // B1=S, B2=A0, B3=A1, B4=A2（手册 BUS 表：S A0 A1 A2）
        s.raw = (static_cast<uint32_t>(reply[4]) << 16) |
                (static_cast<uint32_t>(reply[3]) << 8) |
                static_cast<uint32_t>(reply[2]);
        s.status   = reply[1];
        s.error    = (reply[1] & 0x80) != 0;
        s.warning  = (reply[1] & 0x40) != 0;
    } else {
        n = transact(0x31, reply, 5);
        if (n < 5) return false;
        if (crc8(reply, 4) != reply[4]) return false;
        s.raw = (static_cast<uint32_t>(reply[0]) << 16) |
                (static_cast<uint32_t>(reply[1]) << 8) |
                static_cast<uint32_t>(reply[2]);
        s.status   = reply[3];
        s.error    = (reply[3] & 0x80) != 0;
        s.warning  = (reply[3] & 0x40) != 0;
    }
    s.angle_deg = rawToDeg(s.raw);
    s.stamp = std::chrono::steady_clock::now();
    return true;
}

bool MptEncoder::readPositionStatus(Sample& s) {
    if (proto_ == Protocol::BUS) return readPosition(s);  // BUS“获取信息”已含状态
    uint8_t reply[16] = {0};
    int n = transact(0x64, reply, 5);
    if (n < 5) return false;
    if (crc8(reply, 4) != reply[4]) return false;
    s.raw = (static_cast<uint32_t>(reply[0]) << 16) |
            (static_cast<uint32_t>(reply[1]) << 8) |
            static_cast<uint32_t>(reply[2]);
    s.status   = reply[3];
    s.error    = (reply[3] & 0x80) != 0;
    s.warning  = (reply[3] & 0x40) != 0;
    s.angle_deg = rawToDeg(s.raw);
    s.stamp = std::chrono::steady_clock::now();
    return true;
}

bool MptEncoder::setZero() {
    // —— RS485 款：0x31 与 0x30 交替共 10 次；当 0x30 返回计数值为 10 时生效 ——
    // —— BUS 款：操作类型 00 与 01 交替共 10 组，每组发完等回复 ——
    const int kRounds = 12;  // 略多几次确保 C 到 10
    uint8_t reply[16] = {0};
    if (proto_ == Protocol::RS485) {
        for (int i = 0; i < kRounds; ++i) {
            int n = transact(0x31, reply, 5);
            if (n < 5) continue;
            n = transact(0x30, reply, 2);
            if (n >= 2) {
                // 返回 2 字节：C CRC（24 位单圈）
                // C 为连续计数，到 10 时触发设置；期间约 50ms 不响应
                if (reply[0] >= 10) {
                    ::usleep(60000);  // 等 50ms 生效窗口过去
                    return true;
                }
            }
        }
        return false;
    }
    // BUS 款
    for (int i = 0; i < kRounds; ++i) {
        uint8_t q0 = makeBusRequest(0b00, addr_);
        int n = transact(q0, reply, 6);
        if (n < 6) continue;
        uint8_t q1 = makeBusRequest(0b01, addr_);
        n = transact(q1, reply, 3);   // 设置零位返回：S C CRC → 3 字节
        if (n >= 3 && crc8(reply + 1, 1) == reply[2]) {
            if (reply[1] >= 10) {
                ::usleep(60000);
                return true;
            }
        }
    }
    return false;
}

bool MptEncoder::setBusAddress(uint8_t new_addr) {
    if (proto_ != Protocol::BUS) return false;
    new_addr &= 0x1F;
    // 手册序列：X（任意）, 2, 4, 6, 8, 10, 12, 14, 16, Y（目标地址）
    // 其中“设置地址”的地址字段值为上次返回 C 值的 2 倍：
    //   X(无响应) → 2(无响应) → 4(C=4) → 6(C=5) → 8(C=6) → 10(C=7) → 12(C=8) → 14(C=9) → 16(C=10) → Y
    static const uint8_t seq[] = {2, 4, 6, 8, 10, 12, 14, 16};
    uint8_t reply[16] = {0};

    // 先发任意 X（无响应）
    transact(makeBusRequest(0b10, 0x00), reply, 2, 8);
    for (uint8_t v : seq) {
        int n = transact(makeBusRequest(0b10, v), reply, 3, 20);
        (void)n;  // 序列中前三组可能无响应；后续返回 C 校验可逐步核对
    }
    // 最后一发目标地址
    int n = transact(makeBusRequest(0b10, new_addr), reply, 3, 20);
    if (n >= 2) {
        addr_ = new_addr;
        return true;
    }
    return false;
}

} // namespace taihu
