/**
 * @file taihu_gait_node.cpp
 * @brief 单腿步态播放节点：把步态轨迹发给电机驱动节点的 command_position。
 *
 * 与 taihu_singleleg_step_xCAN 的步态定义一致（一步 = 3 个等长阶段）：
 *   阶段 0（零位保持）：四电机零位保持不动；
 *   阶段 1（正摆）：ID 2/3 从 0° 到 +180°；
 *   阶段 2（回摆）：ID 2/3 从 +180° 回到 0°。
 *
 * 停顿（按需开启）：
 *   - 参数 pause_after_swing（默认 true）：阶段 1 结束后暂停，等待
 *     ~/resume 服务调用后才开始阶段 2（对应终端 demo 里「按回车开始回摆」）；
 *   - pause 后 target 冻结在当前值，电机保持通电锁止（与终端 demo 一致）。
 *
 * 观测：
 *   - ~/gait_state 话题发布当前步/阶段/是否暂停（std_msgs 风格自定义字符串，简单可读）。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "taihu/config.h"

namespace {

using std_srvs::srv::Trigger;

constexpr int    kPhasesPerStep = 3;   ///< 一步 = 3 阶段（0 零位 / 1 正摆 / 2 回摆）
constexpr double kDegToRad      = 3.14159265358979323846 / 180.0;

struct StepPose {
    double id[4];   ///< 四个电机的目标角度（rad）
};

/// 一步的关键帧序列：第 i 阶段在 keyframes[i] → keyframes[i+1] 之间五次多项式插值。
/// 与终端 demo 完全一致（ID1/4 不动，ID2/3 0° ↔ 180°）。
std::vector<StepPose> makeKeyframes() {
    const double z = 0.0, s = 180.0 * kDegToRad;
    return {
        {{z, z, z, z}},  ///< 阶段 0 起点
        {{z, z, z, z}},  ///< 阶段 0 终点 = 阶段 1 起点
        {{z, s, s, z}},  ///< 阶段 1 终点 = 阶段 2 起点
        {{z, z, z, z}},  ///< 阶段 2 终点（= 下一步起点）
    };
}

double quinticSmoothstep(double t) {
    return 6.0 * t * t * t * t * t - 15.0 * t * t * t * t + 10.0 * t * t * t;
}

} // namespace

namespace taihu_ros {

class GaitNode : public rclcpp::Node {
public:
    explicit GaitNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("taihu_gait_node", options), keyframes_(makeKeyframes()) {
        num_steps_    = declare_parameter<int>("num_steps", 1);
        phase_sec_    = declare_parameter<double>("phase_sec", 5.0);
        pause_after_swing_ = declare_parameter<bool>("pause_after_swing", true);

        cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "taihu_motor_node/command_position", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("~/gait_state", 10);

        start_srv_ = create_service<Trigger>(
            "~/start_gait", [this](const Trigger::Request::SharedPtr,
                                   Trigger::Response::SharedPtr res) {
                if (gait_thread_ && gait_thread_->joinable()) {
                    res->success = false;
                    res->message = "步态已在运行";
                    return;
                }
                gait_thread_ = std::make_unique<std::thread>(&GaitNode::runGait, this);
                res->success = true;
                res->message = "步态已启动";
            });

        stop_srv_ = create_service<Trigger>(
            "~/stop_gait", [this](const Trigger::Request::SharedPtr,
                                  Trigger::Response::SharedPtr res) {
                abort_ = true;
                resume_ = true;   // 解除暂停以防死等
                if (gait_thread_ && gait_thread_->joinable()) gait_thread_->join();
                gait_thread_.reset();
                res->success = true;
                res->message = "步态已停止，电机目标冻结";
            });

        resume_srv_ = create_service<Trigger>(
            "~/resume", [this](const Trigger::Request::SharedPtr,
                               Trigger::Response::SharedPtr res) {
                if (paused_.load()) {
                    resume_ = true;
                    res->success = true;
                    res->message = "已放行回摆";
                } else {
                    res->success = false;
                    res->message = "当前不在暂停状态";
                }
            });

        RCLCPP_INFO(get_logger(),
                    "步态节点就绪：N=%d 步 × 3 阶段 × %.1fs，正摆后%s停顿",
                    num_steps_, phase_sec_, pause_after_swing_ ? "会" : "不");
    }

    ~GaitNode() override {
        abort_ = true;
        resume_ = true;
        if (gait_thread_ && gait_thread_->joinable()) gait_thread_->join();
    }

private:
    void publishState(int step, int phase, bool paused) {
        std_msgs::msg::String msg;
        msg.data = "step " + std::to_string(step) + "/" + std::to_string(num_steps_) +
                   ", phase " + std::to_string(phase) +
                   (paused ? ", PAUSED (call ~/resume to continue)" : "");
        state_pub_->publish(msg);
    }

    void sendTarget(const StepPose& pose) {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.position = {pose.id[0], pose.id[1], pose.id[2], pose.id[3]};
        cmd_pub_->publish(js);
    }

    /// 步态线程：等价于终端 demo 的发送线程（含正摆后暂停等放行）
    void runGait() {
        const auto loop_period = std::chrono::milliseconds(5);
        int step = 1;
        for (; step <= num_steps_ && !abort_.load(); ++step) {
            for (int phase = 0; phase < kPhasesPerStep && !abort_.load(); ++phase) {
                // —— 执行本阶段 ——
                const auto phase_start = std::chrono::steady_clock::now();
                while (!abort_.load()) {
                    const double t = std::chrono::duration<double>(
                                         std::chrono::steady_clock::now() - phase_start).count();
                    if (t >= phase_sec_) break;

                    const StepPose& a = keyframes_[phase];
                    const StepPose& b = keyframes_[phase + 1];
                    const double s = quinticSmoothstep(t / phase_sec_);
                    StepPose p;
                    for (int m = 0; m < 4; ++m)
                        p.id[m] = a.id[m] + (b.id[m] - a.id[m]) * s;
                    sendTarget(p);

                    std::this_thread::sleep_for(loop_period);
                }

                // —— 阶段 1（正摆）结束：暂停等放行 ——
                if (phase == 1 && pause_after_swing_ && !abort_.load()) {
                    paused_ = true;
                    publishState(step, phase, true);
                    RCLCPP_INFO(get_logger(), "第 %d/%d 步正摆完成，等待 ~/resume 放行回摆",
                                step, num_steps_);
                    while (!resume_.load() && !abort_.load())
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    paused_ = false;
                    resume_ = false;
                }
                publishState(step, phase, false);
            }
        }
        RCLCPP_INFO(get_logger(), "步态结束（共 %d 步）", step - 1);
    }

    // —— 成员 ——
    int num_steps_ = 1;
    double phase_sec_ = 5.0;
    bool pause_after_swing_ = true;

    std::vector<StepPose> keyframes_;
    std::unique_ptr<std::thread> gait_thread_;

    std::atomic<bool> paused_{false};
    std::atomic<bool> resume_{false};
    std::atomic<bool> abort_{false};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Service<Trigger>::SharedPtr start_srv_;
    rclcpp::Service<Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<Trigger>::SharedPtr resume_srv_;
};

} // namespace taihu_ros

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<taihu_ros::GaitNode>());
    rclcpp::shutdown();
    return 0;
}
