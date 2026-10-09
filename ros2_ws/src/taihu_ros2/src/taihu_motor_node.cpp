/**
 * @file taihu_motor_node.cpp
 * @brief 钛虎电机 ROS2 驱动节点：一条 CAN 总线 + 四个电机（一条腿）。
 *
 * 节点行为：
 *   - 10ms 实时控制循环（独立线程，不阻塞 ROS 执行器）；
 *   - 200Hz 发布 sensor_msgs/JointState（位置/速度/电流估算）；
 *   - 订阅 ~/command（std_msgs 暂不用，直接内置接口见下）：
 *       * 服务 ~/enable  ：使能并开启 10ms 位置控制循环；
 *       * 服务 ~/stop    ：停止全部电机（抱闸）；
 *       * 服务 ~/set_gains：设置位置环 KP/KD；
 *   - 订阅 ~/command_position（sensor_msgs/JointState 风格，四电机目标位置数组）。
 *
 * 与 demo 的对应关系：
 *   - 控制方式与 taihu_singleleg_step_xCAN 的发送线程一致（10ms 周期 setTargetPosition）；
 *   - 读取用 0x41 三合一命令（一次读回电流+速度+位置，减少总线占用）。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "taihu/config.h"
#include "taihu/joint_module.h"
#include "taihu/socket_can.h"

namespace {

using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;

// 一条腿的四个电机：ID 与减速比（与 taihu_singleleg_step 一致）
const std::vector<uint32_t> kMotorIds    = {1, 2, 3, 4};
const std::vector<double>   kGearRatios  = {101.0, 81.0, 81.0, 101.0};
const std::vector<std::string> kJointNames = {"joint1", "joint2", "joint3", "joint4"};

} // namespace

namespace taihu_ros {

class MotorNode : public rclcpp::Node {
public:
    explicit MotorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("taihu_motor_node", options) {
        // —— 参数 ——
        can_ifname_ = declare_parameter<std::string>("can_ifname", "can0");
        pos_kp_     = declare_parameter<int>("position_kp", 4000);
        pos_kd_     = declare_parameter<int>("position_kd", 60);
        rate_hz_    = declare_parameter<int>("control_rate_hz", 200);

        // —— CAN 与电机初始化 ——
        if (!can_.open(can_ifname_.c_str(), 0)) {
            RCLCPP_FATAL(get_logger(),
                         "打开 CAN 接口 %s 失败（接口需要先 up：sudo ./build/kcanctl up %s 1000000）",
                         can_ifname_.c_str(), can_ifname_.c_str());
            throw std::runtime_error("CAN open failed: " + can_ifname_);
        }
        for (size_t i = 0; i < kMotorIds.size(); ++i) {
            auto j = std::make_unique<taihu::JointModule>(can_, kMotorIds[i], kMotorIds[i]);
            j->setGearRatio(kGearRatios[i]);
            j->clearError();
            joints_.push_back(std::move(j));
        }
        RCLCPP_INFO(get_logger(), "CAN %s 就绪，%zu 个电机（ID 1~4）已初始化",
                    can_ifname_.c_str(), joints_.size());

        // —— 发布 / 订阅 / 服务 ——
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
        cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "~/command_position", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                if (msg->position.size() < joints_.size()) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "command_position 需要至少 %zu 个位置，收到 %zu",
                                         joints_.size(), msg->position.size());
                    return;
                }
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                for (size_t i = 0; i < joints_.size(); ++i)
                    cmd_position_[i] = msg->position[i];
            });

        enable_srv_ = create_service<Trigger>(
            "~/enable", [this](const Trigger::Request::SharedPtr,
                               Trigger::Response::SharedPtr res) {
                applyGains();
                enable_ = true;
                res->success = true;
                res->message = "控制循环已使能";
            });
        stop_srv_ = create_service<Trigger>(
            "~/stop", [this](const Trigger::Request::SharedPtr,
                             Trigger::Response::SharedPtr res) {
                enable_ = false;
                for (auto& j : joints_) j->stop();
                res->success = true;
                res->message = "已停止全部电机（抱闸）";
            });
        set_gains_srv_ = create_service<Trigger>(
            "~/set_gains", [this](const Trigger::Request::SharedPtr,
                                  Trigger::Response::SharedPtr res) {
                applyGains();
                res->success = true;
                res->message = "增益已下发";
            });

        // —— 实时线程 ——
        ctrl_thread_ = std::thread(&MotorNode::controlLoop, this);
    }

    ~MotorNode() override {
        running_ = false;
        if (ctrl_thread_.joinable()) ctrl_thread_.join();
        for (auto& j : joints_) j->stop();
        can_.close();
    }

private:
    /// 把 KP/KD 下发到全部电机（enable / set_gains 服务调用时执行）
    void applyGains() {
        const int64_t kp = get_parameter("position_kp").as_int();
        const int64_t kd = get_parameter("position_kd").as_int();
        for (auto& j : joints_) {
            j->setPositionKp(static_cast<int32_t>(kp));
            j->setPositionKd(static_cast<int32_t>(kd));
        }
        RCLCPP_INFO(get_logger(), "已下发增益 KP=%lld KD=%lld",
                    static_cast<long long>(kp), static_cast<long long>(kd));
    }

    /// 10ms/200Hz 控制循环：发布状态 +（使能时）下发目标位置
    void controlLoop() {
        const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(rate_hz_));
        auto next_tick = std::chrono::steady_clock::now() + std::chrono::milliseconds(2);
        sensor_msgs::msg::JointState js;
        js.name = kJointNames;
        js.position.resize(joints_.size());
        js.velocity.resize(joints_.size());
        js.effort.resize(joints_.size());

        while (running_.load()) {
            // —— 状态读取 + 发布（0x41 三合一）——
            for (size_t i = 0; i < joints_.size(); ++i) {
                int32_t cur_ma = 0;
                double vel = 0.0, pos = 0.0;
                if (joints_[i]->readCurrentVelocityPosition(cur_ma, vel, pos)) {
                    js.position[i] = pos;
                    js.velocity[i] = vel;
                    // 电流估算力矩：T = Kt * I；CRA-RI50-60 的 Kt ≈ 0.107 N·m/A（电机端）
                    js.effort[i]   = static_cast<double>(cur_ma) * 0.001 * 0.107 * kGearRatios[i];
                }
            }
            js.header.stamp = this->now();
            joint_state_pub_->publish(js);

            // —— 控制下发 ——
            if (enable_.load()) {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                for (size_t i = 0; i < joints_.size(); ++i)
                    joints_[i]->setTargetPosition(cmd_position_[i]);
            }

            next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next_tick);
        }
    }

    // —— 成员 ——
    std::string can_ifname_;
    int pos_kp_ = 4000, pos_kd_ = 60, rate_hz_ = 200;

    taihu::SocketCanInterface can_;
    std::vector<std::unique_ptr<taihu::JointModule>> joints_;
    std::vector<double> cmd_position_ = {0.0, 0.0, 0.0, 0.0};
    std::mutex cmd_mutex_;

    std::atomic<bool> enable_{false};
    std::atomic<bool> running_{true};
    std::thread ctrl_thread_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
    rclcpp::Service<Trigger>::SharedPtr enable_srv_;
    rclcpp::Service<Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<Trigger>::SharedPtr set_gains_srv_;
};

} // namespace taihu_ros

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<taihu_ros::MotorNode>());
    rclcpp::shutdown();
    return 0;
}
