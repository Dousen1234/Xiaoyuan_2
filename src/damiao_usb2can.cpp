#include "taihu/damiao_usb2can.h"

#include <chrono>
#include <cstring>
#include <cstdio>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

namespace taihu {

DamiaoUsb2CanInterface::~DamiaoUsb2CanInterface() {
    close();
}

bool DamiaoUsb2CanInterface::open(const char* device, int bitrate) {
    (void)bitrate; // 串口波特率固定 921600，CAN 波特率由设备配置

    fd_ = ::open(device, O_RDWR | O_NOCTTY);
    if (fd_ < 0) return false;

    struct termios opt;
    std::memset(&opt, 0, sizeof(opt));
    if (::tcgetattr(fd_, &opt) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    opt.c_oflag = 0;
    opt.c_lflag = 0;
    opt.c_iflag = 0;

    // 921600 波特率，8 数据位，无校验，1 停止位
    cfsetispeed(&opt, B921600);
    cfsetospeed(&opt, B921600);

    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_cflag &= ~PARENB;
    opt.c_iflag &= ~INPCK;
    opt.c_cflag &= ~CSTOPB;

    opt.c_cc[VTIME] = 0;
    opt.c_cc[VMIN] = 0;
    opt.c_lflag |= CBAUDEX;

    ::tcflush(fd_, TCIFLUSH);
    if (::tcsetattr(fd_, TCSANOW, &opt) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    return true;
}

void DamiaoUsb2CanInterface::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    while (!rx_queue_.empty()) rx_queue_.pop();
}

bool DamiaoUsb2CanInterface::send(const CanFrame& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) return false;

    DmSendFrame s;
    std::memset(&s, 0, sizeof(s));

    s.header[0]    = 0x55;
    s.header[1]    = 0xAA;
    s.frame_len    = sizeof(DmSendFrame); // 0x1E = 30
    s.cmd          = 0x03;                // 非反馈 CAN 转发
    s.send_times   = 1;
    s.time_interval = 10;
    s.id_type      = frame.is_extended ? 1 : 0;
    s.can_id       = frame.id;
    s.frame_type   = 0;                   // 数据帧
    s.len          = frame.dlc;
    s.id_acc       = 0;
    s.data_acc     = 0;
    std::memcpy(s.data, frame.data, 8);
    s.crc          = 0;

    ssize_t n = ::write(fd_, &s, sizeof(s));
    return n == static_cast<ssize_t>(sizeof(s));
}

bool DamiaoUsb2CanInterface::readBytes(uint8_t* buf, size_t len, int timeout_ms) {
    if (fd_ < 0) return false;

    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(fd_, &rset);

    struct timeval tv;
    struct timeval* tvp = nullptr;
    if (timeout_ms >= 0) {
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        tvp = &tv;
    }

    int ret = ::select(fd_ + 1, &rset, nullptr, nullptr, tvp);
    if (ret <= 0) return false;

    ssize_t n = ::read(fd_, buf, len);
    if (n <= 0) return false;

    // 追加到接收队列
    for (ssize_t i = 0; i < n; ++i) rx_queue_.push(buf[i]);
    return true;
}

bool DamiaoUsb2CanInterface::receive(CanFrame& frame, int timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) return false;

    const auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::milliseconds(timeout_ms);

    while (true) {
        // 丢弃非帧头字节，寻找 0xAA
        while (!rx_queue_.empty() && rx_queue_.front() != 0xAA) {
            rx_queue_.pop();
        }

        // 若已有完整帧，尝试解析
        if (rx_queue_.size() >= sizeof(DmRecvFrame)) {
            DmRecvFrame r;
            for (size_t i = 0; i < sizeof(DmRecvFrame); ++i) {
                reinterpret_cast<uint8_t*>(&r)[i] = rx_queue_.front();
                rx_queue_.pop();
            }

            // 仅有效数据帧（接收成功 + 帧尾正确）才返回；
            // 否则丢弃该帧，继续寻找下一帧
            if (r.cmd == 0x11 && r.frame_end == 0x55) {
                frame.id          = r.can_id;
                frame.dlc         = r.can_dlc;
                frame.is_extended = (r.can_ide != 0);
                std::memcpy(frame.data, r.data, 8);
                return true;
            }
            continue;
        }

        // 计算剩余超时
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return false;
        const int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());

        // 读更多字节
        uint8_t buf[512];
        if (!readBytes(buf, sizeof(buf), remaining_ms)) return false;
    }
}

void DamiaoUsb2CanInterface::drain() {
    std::lock_guard<std::mutex> lock(mutex_);

    // 清空接收队列
    while (!rx_queue_.empty()) rx_queue_.pop();

    // 持续读空串口缓冲（1ms 内无新数据即停止，降低每次读命令的等待开销）
    uint8_t buf[512];
    while (readBytes(buf, sizeof(buf), /*timeout_ms=*/1)) {
    }
    while (!rx_queue_.empty()) rx_queue_.pop();
}

int DamiaoUsb2CanInterface::dumpRaw(int timeout_ms) {
    if (fd_ < 0) return -1;

    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(fd_, &rset);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = ::select(fd_ + 1, &rset, nullptr, nullptr, &tv);
    if (ret <= 0) return 0;

    uint8_t buf[512];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) return 0;

    std::printf("[raw %zd bytes] ", n);
    for (ssize_t i = 0; i < n; ++i) {
        std::printf("%02X ", buf[i]);
    }
    std::printf("\n");
    return static_cast<int>(n);
}

} // namespace taihu
