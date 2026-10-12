/**
 * @file taihu_encoder_node.cpp
 * @brief 金钢 MPT 磁编码器 ROS2 驱动节点：一条 RS485 总线 + 最多 32 台编码器。
 *
 * 节点行为：
 *   - 独立于 CAN 电机总线（走 /dev/ttyUSB* 串口），与 taihu_motor_node 互不影响；
 *   - 200Hz 轮询总线上所有已配置的编码器（BUS 款按地址逐一问询）；
 *   - 发布：
 *       * ~/angle（std_msgs/Float64[]）：各编码器单圈角度（度，0~360），顺序与 enc_addrs 一致；
 *       * ~/samples（sensor_msgs/JointState 风格）：name=enc<addr>，position=角度，
 *         effort 不用，velocity 留 0（编码器单圈无速度输出，可由上游差分）；
 *   - 服务 ~/zero（std_srvs/Trigger）：对全部编码器设零位；
 *   - 服务 ~/zero_one（std_srvs/SetBool）：data=true 设第一台，false 设第二台（双编码器场景）。
 *
 * 典型用法（两台 D 款共总线，地址已改为 5 和 6）：
 *   ros2 run taihu_ros2 taihu_encoder_node --ros-args
 *       -p serial_port:=/dev/ttyUSB0 -p baud:=2500000
 *       -p enc_addrs:="[5, 6]"
 *
 * R 款（RS485）单台：enc_addrs 留空数组即自动探测（R 款只有一台）。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "taihu/mpt_encoder.h"

namespace {

using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;

} // namespace

namespace taihu_ros {

class EncoderNode : public rclcpp::Node {
public:
    explicit EncoderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("taihu_encoder_node", options) {
        // —— 参数 ——
        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        baud_        = declare_parameter<int>("baud", 2500000);
        rate_hz_     = declare_parameter<int>("rate_hz", 200);
        enc_addrs_   = declare_parameter<std::vector<int64_t>>("enc_addrs",
                                                                std::vector<int64_t>{});
        force_bus_   = declare_parameter<bool>("force_bus", false);

        // —— 打开串口 ——
        std::string err;
        if (!port_.open(serial_port_, baud_, &err)) {
            RCLCPP_FATAL(get_logger(),
                         "打开串口 %s 失败：%s（ls /dev/ttyUSB* 确认设备名；2.5Mbps 需 FT232H 类 USB 转 485）",
                         serial_port_.c_str(), err.c_str());
            throw std::runtime_error("serial open failed: " + serial_port_);
        }

        // —— 编码器发现 ——
        if (enc_addrs_.empty()) {
            // 自动探测：总线上第一台
            uint8_t found_addr = 0;
            auto proto = taihu::MptEncoder::probe(port_, &found_addr,
                                                  taihu::MptEncoder::Output::Single24);
            encoders_.push_back(std::make_unique<taihu::MptEncoder>(
                port_, proto, found_addr == 0 ? taihu::MptEncoder::kFactoryAddr : found_addr));
            RCLCPP_INFO(get_logger(), "自动探测：1 台编码器（协议 %s，地址 0x%02X）",
                        proto == taihu::MptEncoder::Protocol::BUS ? "BUS" : "RS485",
                        encoders_[0]->address());
        } else {
            // 用户指定地址列表（BUS 款）：逐台构造，地址来自参数
            for (int64_t a : enc_addrs_) {
                encoders_.push_back(std::make_unique<taihu::MptEncoder>(
                    port_, taihu::MptEncoder::Protocol::BUS, static_cast<uint8_t>(a)));
            }
            RCLCPP_INFO(get_logger(), "按参数配置 %zu 台编码器（BUS 地址: %s）",
                        encoders_.size(), addrListString().c_str());
        }

        // —— 发布 / 服务 ——
        angle_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/angle", 10);
        js_pub_    = create_publisher<sensor_msgs::msg::JointState>("~/samples", 10);

        zero_srv_ = create_service<Trigger>(
            "~/zero", [this](const Trigger::Request::SharedPtr,
                             Trigger::Response::SharedPtr res) {
                int ok = 0;
                for (auto& e : encoders_) if (e->setZero()) ++ok;
                res->success = (ok == static_cast<int>(encoders_.size()));
                res->message = "设零位完成 " + std::to_string(ok) + "/" +
                               std::to_string(encoders_.size()) + " 台";
            });
        zero_one_srv_ = create_service<SetBool>(
            "~/zero_one", [this](const SetBool::Request::SharedPtr req,
                                 SetBool::Response::SharedPtr res) {
                size_t idx = req->data ? 0 : (encoders_.size() > 1 ? 1 : 0);
                if (idx >= encoders_.size()) {
                    res->success = false;
                    res->message = "没有第 " + std::to_string(idx + 1) + " 台编码器";
                    return;
                }
                res->success = encoders_[idx]->setZero();
                res->message = res->success ? "已设零位" : "设零位失败";
            });

        // —— 轮询线程 ——
        poll_thread_ = std::thread(&EncoderNode::pollLoop, this);
        RCLCPP_INFO(get_logger(), "编码器节点就绪（%s @ %d bps，%d Hz，%zu 台）",
                    serial_port_.c_str(), baud_, rate_hz_, encoders_.size());
    }

    ~EncoderNode() override {
        running_ = false;
        if (poll_thread_.joinable()) poll_thread_.join();
        port_.close();
    }

private:
    std::string addrListString() const {
        std::string s;
        for (size_t i = 0; i < encoders_.size(); ++i) {
            if (i) s += ",";
            s += "0x" + std::to_string(encoders_[i]->address());
        }
        return s;
    }

    void pollLoop() {
        const auto period = std::chrono::duration<double>(
            1.0 / static_cast<double>(rate_hz_));
        auto next_tick = std::chrono::steady_clock::now();

        std_msgs::msg::Float64MultiArray angle_msg;
        angle_msg.data.resize(encoders_.size());
        sensor_msgs::msg::JointState js;
        js.name.resize(encoders_.size());
        for (size_t i = 0; i < encoders_.size(); ++i)
            js.name[i] = "enc" + std::to_string(encoders_[i]->address());
        js.position.resize(encoders_.size());
        js.velocity.resize(encoders_.size(), 0.0);

        while (running_.load()) {
            for (size_t i = 0; i < encoders_.size(); ++i) {
                taihu::MptEncoder::Sample smp;
                if (encoders_[i]->readPosition(smp)) {
                    angle_msg.data[i] = smp.angle_deg;
                    js.position[i]    = smp.angle_deg;
                    // 错误位统计（每秒打一条）
                    if (smp.error) ++err_cnt_;
                } else {
                    ++miss_cnt_;
                }
            }
            js.header.stamp = this->now();
            angle_pub_->publish(angle_msg);
            js_pub_->publish(js);

            // 每秒一次健康统计
            auto now = std::chrono::steady_clock::now();
            if (now - last_stat_ > std::chrono::seconds(1)) {
                if (miss_cnt_ > 0 || err_cnt_ > 0) {
                    RCLCPP_WARN(get_logger(), "轮询统计（1s）：读失败 %lld 次，错误位 %lld 次",
                                static_cast<long long>(miss_cnt_),
                                static_cast<long long>(err_cnt_));
                }
                miss_cnt_ = 0;
                err_cnt_  = 0;
                last_stat_ = now;
            }

            next_tick += std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next_tick);
        }
    }

    // —— 成员 ——
    std::string serial_port_;
    int baud_ = 2500000, rate_hz_ = 200;
    std::vector<int64_t> enc_addrs_;
    bool force_bus_ = false;

    taihu::Rs485Port port_;
    std::vector<std::unique_ptr<taihu::MptEncoder>> encoders_;

    std::atomic<bool> running_{true};
    std::thread poll_thread_;
    int64_t miss_cnt_ = 0, err_cnt_ = 0;
    std::chrono::steady_clock::time_point last_stat_{};

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    rclcpp::Service<Trigger>::SharedPtr zero_srv_;
    rclcpp::Service<SetBool>::SharedPtr zero_one_srv_;
};

} // namespace taihu_ros

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<taihu_ros::EncoderNode>());
    rclcpp::shutdown();
    return 0;
}
