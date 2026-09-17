#ifndef TAIHU_MOTOR_LOGGER_H
#define TAIHU_MOTOR_LOGGER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "taihu/taihu_tools.h"  // MotorStatus

namespace taihu {

/**
 * @brief 电机运行日志记录器（支持多电机）。
 *
 * 以 CSV 格式把多个电机的状态写入文件，每一行包含：
 *   时间戳 + 各电机的 ID/位置/速度/电流/电压/错误
 *
 * - 每次打开（构造）都以覆盖模式写入，即新运行会覆盖旧日志；
 * - 时间戳由调用者显式传入（单位 秒）。
 */
class MotorLogger {
public:
    /**
     * @brief 打开日志文件（覆盖模式）。
     * @param filename 日志文件路径（如 "motor_log.csv"）
     */
    explicit MotorLogger(const std::string& filename);

    ~MotorLogger();

    MotorLogger(const MotorLogger&) = delete;
    MotorLogger& operator=(const MotorLogger&) = delete;

    /// 是否成功打开文件。
    bool isOpen() const { return file_.is_open(); }

    /**
     * @brief 写入 CSV 表头（按电机 ID 顺序）。
     * @param motor_ids 电机 ID 列表，决定列顺序
     */
    void writeHeader(const std::vector<uint32_t>& motor_ids);

    /**
     * @brief 记录一行多电机状态。
     * @param timestamp_s 采样时刻（秒）
     * @param motors      各电机状态（顺序与 writeHeader 的 motor_ids 一致）
     */
    void log(double timestamp_s, const std::vector<MotorStatus>& motors);

    /// 关闭文件。
    void close();

private:
    std::ofstream file_;
};

} // namespace taihu

#endif // TAIHU_MOTOR_LOGGER_H
