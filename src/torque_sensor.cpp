#include "taihu/torque_sensor.h"

#include <cstring>

namespace taihu {

namespace {

/// 协议固定字节
constexpr uint8_t kTail0 = 0x0D;
constexpr uint8_t kTail1 = 0x0A;
constexpr uint8_t kFixedAddr = 0xAA;

/// 从 4 字节小端数据读 float（协议：低字节在前）
float readFloatLE(const uint8_t* p) {
    uint32_t u = static_cast<uint32_t>(p[0])
               | (static_cast<uint32_t>(p[1]) << 8)
               | (static_cast<uint32_t>(p[2]) << 16)
               | (static_cast<uint32_t>(p[3]) << 24);
    float f;
    std::memcpy(&f, &u, sizeof f);
    return f;
}

} // namespace

TorqueSensor::TorqueSensor(CanInterface& can, uint32_t rx_id, uint32_t tx_id)
    : can_(can), rx_id_(rx_id), tx_id_(tx_id) {}

TorqueSensor::~TorqueSensor() {
    stopRxThread();
}

// ===== 命令发送 =====

bool TorqueSensor::requestSingle() {
    CanFrame f;
    f.id = rx_id_;
    f.dlc = 4;
    f.data[0] = 0x49;
    f.data[1] = kFixedAddr;
    f.data[2] = kTail0;
    f.data[3] = kTail1;
    return can_.send(f);
}

bool TorqueSensor::startContinuous(OutputRate rate) {
    CanFrame f;
    f.id = rx_id_;
    f.dlc = 5;
    f.data[0] = 0x48;
    f.data[1] = kFixedAddr;
    f.data[2] = static_cast<uint8_t>(rate);
    f.data[3] = kTail0;
    f.data[4] = kTail1;
    return can_.send(f);
}

bool TorqueSensor::stopOutput() {
    CanFrame f;
    f.id = rx_id_;
    f.dlc = 4;
    f.data[0] = 0x43;
    f.data[1] = kFixedAddr;
    f.data[2] = kTail0;
    f.data[3] = kTail1;
    return can_.send(f);
}

bool TorqueSensor::setBaudRate(BaudRate rate) {
    CanFrame f;
    f.id = rx_id_;
    f.dlc = 5;
    f.data[0] = 0x50;
    f.data[1] = kFixedAddr;
    f.data[2] = static_cast<uint8_t>(rate);
    f.data[3] = kTail0;
    f.data[4] = kTail1;
    return can_.send(f);
}

bool TorqueSensor::setId(uint16_t new_rx_id, uint16_t new_tx_id) {
    CanFrame f;
    f.id = rx_id_;
    // 文档示例：DE AA 04 10 04 30 0D 0A（命令 + 固定地址 + Rx 2B 大端 + Tx 2B 大端 + 尾）
    f.dlc = 8;
    f.data[0] = 0xDE;
    f.data[1] = kFixedAddr;
    f.data[2] = static_cast<uint8_t>(new_rx_id >> 8);   // Rx_ID 高字节（大端）
    f.data[3] = static_cast<uint8_t>(new_rx_id & 0xFF); // Rx_ID 低字节
    f.data[4] = static_cast<uint8_t>(new_tx_id >> 8);   // Tx_ID 高字节（大端）
    f.data[5] = static_cast<uint8_t>(new_tx_id & 0xFF); // Tx_ID 低字节
    f.data[6] = kTail0;
    f.data[7] = kTail1;
    return can_.send(f);
}

// ===== 数据获取 =====

bool TorqueSensor::latest(Sample& out, int max_age_ms) const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (latest_.stamp.time_since_epoch().count() == 0) return false; // 尚无数据
    auto age = std::chrono::steady_clock::now() - latest_.stamp;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(age).count() > max_age_ms)
        return false; // 样本过期
    out = latest_;
    return true;
}

// ===== 接收线程 =====

void TorqueSensor::startRxThread() {
    if (rx_running_.load()) return;
    rx_running_.store(true);
    rx_thread_ = std::thread([this] { rxLoop(); });
}

void TorqueSensor::stopRxThread() {
    if (!rx_running_.load()) return;
    rx_running_.store(false);
    if (rx_thread_.joinable()) rx_thread_.join();
}

void TorqueSensor::rxLoop() {
    // 命令调用线程（requestSingle 等）与接收线程共用一个 socket；
    // receive() 用 select 超时轮询，不与发送冲突。
    while (rx_running_.load()) {
        CanFrame f;
        if (!can_.receive(f, 50)) continue;   // 50ms 超时后重查退出标志
        // 帧甄别：只认本传感器的 7 字节数据帧，电机（ID 1~4）等其它节点一律忽略
        if (f.id != tx_id_ || f.dlc != 7) continue;
        handleDataFrame(f);
    }
}

bool TorqueSensor::handleDataFrame(const CanFrame& f) {
    const uint8_t sync = f.data[0];
    const uint8_t cmd  = f.data[1];
    const uint8_t key  = f.data[2];

    if (cmd != 0x48 && cmd != 0x49) return false; // 只接受数据回复

    const bool is_aa = (key == 0xAA);
    const bool is_bb = (key == 0xBB);
    if (!is_aa && !is_bb) return false;

    // —— 同步数配对：收到 0xAA 帧先暂存；等同一同步数的 0xBB 帧到齐 ——
    if (is_aa) {
        // 新的 0xAA：重置配对窗口（若上一组 0xBB 丢失则放弃旧 0xAA）
        pending_sync_ = sync;
        pending_has_aa_ = true;
        pending_torque_nm_ = static_cast<double>(readFloatLE(&f.data[3]));
        pending_stamp_ = std::chrono::steady_clock::now();
        return false; // 还差 0xBB
    }

    // 0xBB 帧：必须有同同步数的 0xAA 在前
    if (!pending_has_aa_ || sync != pending_sync_) return false;
    const double bridge = static_cast<double>(readFloatLE(&f.data[3]));

    // —— 配对完成，组装样本 ——
    Sample s;
    s.sync = sync;
    s.torque_raw_nm = pending_torque_nm_;
    s.bridge_mv_per_v = bridge;
    s.stamp = pending_stamp_;
    pending_has_aa_ = false; // 清配对状态

    // —— 零偏校准 ——
    bool calib_finished = false;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (calib_active_) {
            calib_sum_ += s.torque_raw_nm;
            if (--calib_remaining_ <= 0) {
                zero_offset_nm_ = calib_sum_ / static_cast<double>(calib_total_);
                calib_active_ = false;
                calib_finished = true;
            }
        }
        s.torque_nm = s.torque_raw_nm - zero_offset_nm_;
        latest_ = s;
    }
    (void)calib_finished; //（如需回调可扩展）

    // —— 每秒统计 ——
    ++sps_window_count_;
    auto now = std::chrono::steady_clock::now();
    if (now - sps_window_start_ >= std::chrono::seconds(1)) {
        sps_atomic_.store(sps_window_count_);
        sps_window_count_ = 0;
        sps_window_start_ = now;
    }
    return true;
}

// ===== 零偏校准 =====

void TorqueSensor::startZeroCalib(int n) {
    std::lock_guard<std::mutex> lk(mtx_);
    calib_total_ = (n > 0 ? n : 1);
    calib_remaining_ = calib_total_;
    calib_sum_ = 0.0;
    calib_active_ = true;
}

} // namespace taihu
