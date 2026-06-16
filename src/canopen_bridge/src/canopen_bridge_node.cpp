#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include "canopen_interface.hpp"
#include "conversion_utils.hpp"

namespace canopen_bridge
{

namespace
{
constexpr uint8_t kSubVelocity = 0x00;
constexpr uint8_t kSubStatus = 0x01;
constexpr uint8_t kSubError = 0x02;

std::string status_to_string(int code)
{
  switch (code) {
    case 0:
      return "OK";
    case 1:
      return "WARNING";
    case 2:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

class CanopenBridgeNode : public rclcpp::Node
{
public:
  CanopenBridgeNode()
  : rclcpp::Node("canopen_bridge_node"),
    can_(std::make_unique<MockCanopenInterface>())
  {
    // ---- Parameters (overridable via YAML / --ros-args -p) ----------------
    velocity_scale_ = this->declare_parameter<double>("velocity_scale", 1000.0);
    rpdo_index_ = this->declare_parameter<int>("rpdo_index", 0x2000);
    tpdo_index_ = this->declare_parameter<int>("tpdo_index", 0x3000);
    const int feedback_rate_hz = this->declare_parameter<int>("feedback_rate_hz", 10);
    const bool fault_injection = this->declare_parameter<bool>("fault_injection", false);

    // setFaultInjection is mock-specific; only poke it if the backend is a mock.
    if (auto * mock = dynamic_cast<MockCanopenInterface *>(can_.get())) {
      mock->setFaultInjection(fault_injection);
    }

    // ---- Callback groups so the subscription and feedback timer can run on
    //      separate executor threads concurrently (bonus: multithreading) ----
    sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_cb_group_;

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_velocity", 10,
      std::bind(&CanopenBridgeNode::on_cmd_velocity, this, std::placeholders::_1),
      sub_options);

    state_pub_ = this->create_publisher<std_msgs::msg::String>("/device_state", 10);

    const auto period =
      std::chrono::milliseconds(feedback_rate_hz > 0 ? 1000 / feedback_rate_hz : 100);
    feedback_timer_ = this->create_wall_timer(
      period,
      std::bind(&CanopenBridgeNode::publish_device_state, this),
      timer_cb_group_);

    RCLCPP_INFO(
      get_logger(),
      "canopen_bridge_node started | scale=%.1f RPDO=0x%04X TPDO=0x%04X rate=%dHz faults=%s",
      velocity_scale_, rpdo_index_, tpdo_index_, feedback_rate_hz,
      fault_injection ? "on" : "off");
  }

private:
  // ---- ROS2 -> CANopen (command path) -----------------------------------
  void on_cmd_velocity(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double ros_vel = msg->linear.x;
    const int16_t can_vel = to_canopen_velocity(ros_vel, velocity_scale_);
    try {
      can_->sendRPDO(static_cast<uint16_t>(rpdo_index_), kSubVelocity, can_vel);
      RCLCPP_INFO(
        get_logger(), "ROS->CAN  linear.x=%.3f m/s -> RPDO[0x%04X:00]=%d",
        ros_vel, rpdo_index_, can_vel);
    } catch (const CanIoError & e) {
      RCLCPP_ERROR(get_logger(), "RPDO send failed: %s", e.what());
    }
  }

  // ---- CANopen -> ROS2 (feedback path) ----------------------------------
  void publish_device_state()
  {
    std_msgs::msg::String out;
    try {
      const auto idx = static_cast<uint16_t>(tpdo_index_);
      const int16_t raw_vel = static_cast<int16_t>(can_->readTPDO(idx, kSubVelocity));
      const int status = can_->readTPDO(idx, kSubStatus);
      const int error_code = can_->readTPDO(idx, kSubError);

      const double vel = from_canopen_velocity(raw_vel, velocity_scale_);

      std::ostringstream ss;
      ss << "{\"status\": \"" << status_to_string(status) << "\", "
         << "\"velocity\": " << std::fixed << std::setprecision(2) << vel << ", "
         << "\"error_code\": " << error_code << "}";
      out.data = ss.str();
      state_pub_->publish(out);
      RCLCPP_INFO(get_logger(), "CAN->ROS  %s", out.data.c_str());
    } catch (const CanIoError & e) {
      // Communication fault: log and publish a well-formed error state so
      // downstream consumers still receive a valid, parseable message.
      RCLCPP_ERROR(get_logger(), "TPDO read failed: %s", e.what());
      out.data = "{\"status\": \"COMM_ERROR\", \"velocity\": 0.00, \"error_code\": 255}";
      state_pub_->publish(out);
    }
  }

  std::unique_ptr<CanopenInterface> can_;

  double velocity_scale_ {1000.0};
  int rpdo_index_ {0x2000};
  int tpdo_index_ {0x3000};

  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

}  // namespace canopen_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<canopen_bridge::CanopenBridgeNode>();

  // Multithreaded executor: CAN polling (timer) and command handling
  // (subscription) live in separate callback groups and can run in parallel.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
