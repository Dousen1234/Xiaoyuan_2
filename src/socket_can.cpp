#include "taihu/socket_can.h"

#include <cstring>

#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace taihu {

SocketCanInterface::~SocketCanInterface() {
    close();
}

bool SocketCanInterface::open(const char* device, int bitrate) {
    (void)bitrate; // 波特率通常由 `ip link set can0 ... bitrate 1000000` 配置

    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) return false;

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, device, IFNAMSIZ - 1);
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(sock_, reinterpret_cast<struct sockaddr*>(&addr),
               sizeof(addr)) < 0) {
        ::close(sock_);
        sock_ = -1;
        return false;
    }
    return true;
}

void SocketCanInterface::close() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

bool SocketCanInterface::send(const CanFrame& frame) {
    if (sock_ < 0) return false;

    struct can_frame f;
    std::memset(&f, 0, sizeof(f));
    f.can_id = frame.id;
    if (frame.is_extended) f.can_id |= CAN_EFF_FLAG;
    f.can_dlc = frame.dlc;
    std::memcpy(f.data, frame.data, frame.dlc);

    return ::write(sock_, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f));
}

bool SocketCanInterface::receive(CanFrame& frame, int timeout_ms) {
    if (sock_ < 0) return false;

    // 循环接收：跳过内核上报的 CAN 错误帧（如 BUS-OFF / 错误计数），
    // 只把正常数据帧返回给上层
    while (true) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(sock_, &fds);

        struct timeval tv;
        struct timeval* tvp = nullptr;
        if (timeout_ms >= 0) {
            tv.tv_sec  = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;
            tvp = &tv;
        }

        int ret = ::select(sock_ + 1, &fds, nullptr, nullptr, tvp);
        if (ret <= 0) return false;

        struct can_frame f;
        ssize_t n = ::read(sock_, &f, sizeof(f));
        if (n < 0) return false;

        // 跳过内核错误帧（带 CAN_ERR_FLAG 标志），继续等待数据帧
        if (f.can_id & CAN_ERR_FLAG) {
            continue;
        }

        frame.id = f.can_id & CAN_EFF_MASK;
        frame.is_extended = (f.can_id & CAN_EFF_FLAG) != 0;
        frame.dlc = f.can_dlc;
        std::memcpy(frame.data, f.data, f.can_dlc);
        return true;
    }
}

void SocketCanInterface::drain() {
    // 非阻塞读取，清空内核接收缓冲区中的残留帧（含错误帧），直到无数据可读。
    // 注意：socket 为阻塞模式，必须用 MSG_DONTWAIT，否则缓冲区为空时会永久阻塞。
    if (sock_ < 0) return;

    struct can_frame f;
    while (::recv(sock_, &f, sizeof(f), MSG_DONTWAIT) > 0) {
    }
}

} // namespace taihu
