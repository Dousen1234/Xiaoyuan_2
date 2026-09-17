#ifndef TAIHU_CONFIG_H
#define TAIHU_CONFIG_H

namespace taihu {

// =====================================================================
// 全局公共配置（集中管理，供示例与工具统一引用）
//
// 注意：串口设备、减速比等属于「每条总线 / 每个电机」的特有参数，
//       不在此全局配置中，应由各程序/各电机单独指定。
// =====================================================================

// —— 控制参数 ——
inline constexpr int kLoopPeriodMs = 5;      ///< 位置控制周期 (ms)

// 日志采样周期：固定 100ms（0.1s）。
// CAN 总线半双工且为单串口，采样与控制无法物理独立；
// 拉长采样周期可大幅减少采样线程对总线的占用，避免争抢控制周期。
inline constexpr int    kLogSamplePeriodMs  = 100;    ///< 日志采样周期 (ms)
inline constexpr double kLogSamplePeriodSec = kLogSamplePeriodMs / 1000.0; ///< 日志采样周期 (s) = 0.1

// —— 到位判定参数（用于 rotateFromZero 等） ——
inline constexpr double kSettleThresholdRad = 0.02; ///< 位置误差阈值 (rad)
inline constexpr int    kSettleTimeoutMs     = 3000; ///< 到位超时 (ms)
inline constexpr int    kPollPeriodMs        = 10;   ///< 到位轮询周期 (ms)

// —— 角度换算 ——
inline constexpr double kDegToRad = 3.14159265358979323846 / 180.0; ///< 度 -> 弧度

} // namespace taihu

#endif // TAIHU_CONFIG_H
