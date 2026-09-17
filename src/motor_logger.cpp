#include "taihu/motor_logger.h"

namespace taihu {

MotorLogger::MotorLogger(const std::string& filename) {
    // 覆盖模式打开（std::ios::trunc 确保新运行覆盖旧日志）
    file_.open(filename, std::ios::out | std::ios::trunc);
}

MotorLogger::~MotorLogger() {
    close();
}

void MotorLogger::writeHeader(const std::vector<uint32_t>& motor_ids) {
    if (!file_.is_open()) return;

    file_ << "timestamp_s";
    for (uint32_t id : motor_ids) {
        file_ << ",id" << id << "_pos"
              << ",id" << id << "_vel"
              << ",id" << id << "_cur"
              << ",id" << id << "_vol"
              << ",id" << id << "_err";
    }
    file_ << '\n';
}

void MotorLogger::log(double timestamp_s, const std::vector<MotorStatus>& motors) {
    if (!file_.is_open()) return;

    file_ << timestamp_s;
    for (const auto& m : motors) {
        file_ << ',' << m.can_id
              << ',' << m.position_rad
              << ',' << m.velocity_rad_s
              << ',' << m.current_ma
              << ',' << m.bus_voltage_v
              << ',' << m.error_bits;
    }
    file_ << '\n';
}

void MotorLogger::close() {
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

} // namespace taihu
