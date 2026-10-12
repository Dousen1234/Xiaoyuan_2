#ifndef TAIHU_TORQUE_SENSOR_H
#define TAIHU_TORQUE_SENSOR_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "taihu/can_interface.h"

namespace taihu {

/**
 * @brief 坤维 HIT 系列扭矩传感器（CAN-2Frames-Torque and Voltage-HIT-V2.0 协议）。
 *
 * 协议要点（均为标准帧）：
 *   - 上位机命令发往 Rx_ID（默认 0x10，即电机总线上的 ID 16，与电机 ID 1~4 不冲突）；
 *   - 传感器回复带 Tx_ID（默认 0x15）；
 *   - `49 AA 0D 0A`              单次回复；
 *   - `48 AA <频率位> 0D 0A`     连续输出（0x00=100Hz 0x01=200Hz 0x02=250Hz 0x03=400Hz）；
 *   - `43 AA 0D 0A`              停止转换与输出；
 *   - `50 AA <波特率位> 0D 0A`   修改波特率（0x00=250k 0x01=500k 0x02=1000k，掉电保持需确认）；
 *   - `DE AA <Rx2B 大端> <Tx2B 大端> 0D 0A`  修改 ID（总线只能接一个传感器）；
 *   - 回复数据：每次 2 帧 × 7 字节，帧 ID = Tx_ID；
 *       第 1 字节同步数（0x00~0xFF 循环），同一同步数的两帧是同一次采样；
 *       第 2 字节命令码（0x48 连续 / 0x49 单次）；
 *       第 3 字节 0xAA → 后 4 字节扭矩 float 小端 Nm；0xBB → 后 4 字节桥压 float 小端 mV/V。
 *
 * 采样率内部 1kHz，连续输出最高 400Hz。上电有初值漂移，建议取前几次平均值
 * 作零偏（本类提供 startZeroCalib(n) 自动完成）。
 *
 * @note 该类与电机控制完全独立：内部只甄别 Tx_ID 的 7 字节数据帧，
 *       总线上其它节点（电机 1~4 等）的帧一律忽略；同样，JointModule
 *       的帧甄别也会忽略本传感器的帧。可与任意主控制程序共用一条 CAN 总线。
 */
class TorqueSensor {
public:
    /// 输出频率选项（0x48 命令的频率选择位）
    enum class OutputRate : uint8_t {
        Hz100 = 0x00,
        Hz200 = 0x01,
        Hz250 = 0x02,
        Hz400 = 0x03,
    };

    /// 波特率选项（0x50 命令的波特率选择位，掉电保持需与厂家确认）
    enum class BaudRate : uint8_t {
        Bps250k  = 0x00,
        Bps500k  = 0x01,
        Bps1000k = 0x02,
    };

    /// 一组配对完成的测量（同一同步数的 0xAA + 0xBB 两帧）
    struct Sample {
        uint8_t  sync = 0;      ///< 同步数（0x00~0xFF 循环）
        double   torque_nm = 0.0;  ///< 扭矩（Nm，已减零偏）
        double   torque_raw_nm = 0.0; ///< 扭矩原始值（未减零偏）
        double   bridge_mv_per_v = 0.0; ///< 桥压（mV/V）
        std::chrono::steady_clock::time_point stamp; ///< 收到 0xAA 帧的时刻
    };

    /**
     * @brief 构造传感器对象（不打开 CAN）。
     * @param can     已打开的 CAN 接口（外部管理生命周期）
     * @param rx_id   传感器命令 ID（默认 0x10）
     * @param tx_id   传感器回复 ID（默认 0x15）
     */
    explicit TorqueSensor(CanInterface& can,
                          uint32_t rx_id = 0x10,
                          uint32_t tx_id = 0x15);

    ~TorqueSensor();

    TorqueSensor(const TorqueSensor&) = delete;
    TorqueSensor& operator=(const TorqueSensor&) = delete;

    // —— 命令（发一条 7 字节命令帧）——

    /// 命令传感器单次转换并回复（`49 AA 0D 0A`）
    bool requestSingle();

    /// 命令传感器连续输出（`48 AA <rate> 0D 0A`），随后可轮询/线程自动收
    bool startContinuous(OutputRate rate = OutputRate::Hz100);

    /// 命令传感器停止转换与输出（`43 AA 0D 0A`）
    bool stopOutput();

    /// 修改传感器波特率（`50 AA <code> 0D 0A`）。改后需重启 CAN 通道再 open。
    bool setBaudRate(BaudRate rate);

    /// 修改传感器 ID（`DE AA <rx 大端 2B> <tx 大端 2B> 0D 0A`）。
    /// @note 发送时总线上只能接一个传感器；执行后本对象仍用旧 ID 记录，
    ///       建议改完重新构造对象（用新 ID）。
    bool setId(uint16_t new_rx_id, uint16_t new_tx_id);

    // —— 数据获取 ——

    /**
     * @brief 取最新测量（线程安全）。
     * @param out 输出样本
     * @param max_age_ms 样本的最大允许年龄（毫秒），超龄返回 false
     * @return 有新鲜样本返回 true
     */
    bool latest(Sample& out, int max_age_ms = 200) const;

    /// 最近 1 秒实际收到的样本数（配对完成计一次），用于判断链路健康
    int samplesPerSecond() const { return sps_atomic_.load(); }

    // —— 后台接收线程（推荐） ——

    /// 启动内部接收线程：自动收帧、配对、更新 latest。可重复调用（先停旧的）。
    void startRxThread();

    /// 停止接收线程
    void stopRxThread();

    /// 接收线程是否在跑
    bool rxThreadRunning() const { return rx_running_.load(); }

    // —— 零偏校准 ——

    /**
     * @brief 启动零偏校准：接下来 n 个配对样本取平均作为零偏。
     * @param n 样本数（建议 50~200）
     * @note 校准期间请保持传感器空载、静止。
     */
    void startZeroCalib(int n = 100);

    /// 清除零偏（偏置归零）
    void clearZero() { std::lock_guard<std::mutex> lk(mtx_); zero_offset_nm_ = 0.0; }

    /// 当前零偏值（Nm）
    double zeroOffsetNm() const { std::lock_guard<std::mutex> lk(mtx_); return zero_offset_nm_; }

private:
    /// 接收线程主循环：阻塞收帧 → 按 ID+长度甄别 → 配对 → 更新 latest
    void rxLoop();

    /// 处理一帧数据（已确认 ID == tx_id_ 且 dlc == 7）；返回是否配对完成
    bool handleDataFrame(const CanFrame& f);

    CanInterface& can_;
    uint32_t rx_id_;
    uint32_t tx_id_;

    mutable std::mutex mtx_;             ///< 保护下面的共享数据
    Sample     latest_;                  ///< 最新配对样本
    double     zero_offset_nm_ = 0.0;    ///< 零偏（原始扭矩 - 零偏 = 输出扭矩）

    // 同步数配对状态（只在接收线程 / 命令调用线程访问）
    uint8_t    pending_sync_ = 0;
    bool       pending_has_aa_ = false;
    double     pending_torque_nm_ = 0.0;
    std::chrono::steady_clock::time_point pending_stamp_;

    // 零偏校准状态
    int        calib_total_ = 0;         ///< 本次校准的目标样本数
    int        calib_remaining_ = 0;     ///< 还需采的样本数
    double     calib_sum_ = 0.0;         ///< 原始扭矩累加
    bool       calib_active_ = false;

    // 统计
    std::atomic<int>  sps_atomic_{0};    ///< 最近 1s 配对样本数（原子读）
    std::chrono::steady_clock::time_point sps_window_start_;
    int               sps_window_count_ = 0;

    std::atomic<bool> rx_running_{false};
    std::thread       rx_thread_;
};

} // namespace taihu

#endif // TAIHU_TORQUE_SENSOR_H
