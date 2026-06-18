#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SafetyWatchdog : public rclcpp::Node {
public:
  SafetyWatchdog() : Node("safety_watchdog") {
    this->declare_parameter("battery_threshold_critical", 10.5);
    this->declare_parameter("battery_threshold_warning", 11.0);
    this->declare_parameter("cmd_vel_timeout_ms", 200);
    this->declare_parameter("heartbeat_timeout_ms", 500);
    this->declare_parameter("sensor_loss_timeout_s", 5.0);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&SafetyWatchdog::cmdVelCallback, this, std::placeholders::_1));
    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/battery_voltage", 10, std::bind(&SafetyWatchdog::batteryCallback, this, std::placeholders::_1));

    safety_pub_ = this->create_publisher<std_msgs::msg::String>("/safety_state", 10);
    safe_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&SafetyWatchdog::watchdogLoop, this));
    RCLCPP_INFO(this->get_logger(), "safety_watchdog initialized");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_ = *msg;
    last_cmd_time_ = this->now();
  }

  void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    battery_voltage_ = msg->data;
  }

  void watchdogLoop() {
    auto now = this->now();
    double cmd_age = (now - last_cmd_time_).seconds() * 1000.0;
    int level = 3; // NORMAL
    std::string detail = "ok";

    if (battery_voltage_ < 10.5) {
      level = 0; detail = "battery critical";
    } else if (cmd_age > 500.0) {
      level = 1; detail = "cmd_vel heartbeat lost";
    } else if (battery_voltage_ < 11.0) {
      level = 1; detail = "battery low";
    } else if (cmd_age > 200.0) {
      level = 2; detail = "cmd_vel stale";
    }

    auto safe_msg = geometry_msgs::msg::Twist();
    if (level <= 1) {
      safe_msg.linear.x = 0.0;
      safe_msg.angular.z = 0.0;
    } else {
      safe_msg.linear.x = last_cmd_.linear.x;
      safe_msg.angular.z = last_cmd_.angular.z;
    }
    safe_cmd_vel_pub_->publish(safe_msg);

    auto state = std_msgs::msg::String();
    state.data = detail;
    safety_pub_->publish(state);
  }

  double battery_voltage_{12.0};
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyWatchdog>());
  rclcpp::shutdown();
  return 0;
}
