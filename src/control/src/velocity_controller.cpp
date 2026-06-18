#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include "autocar_control/pid_controller.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class VelocityController : public rclcpp::Node {
public:
  VelocityController() : Node("velocity_controller") {
    this->declare_parameter("kp", 1.2);
    this->declare_parameter("ki", 0.3);
    this->declare_parameter("kd", 0.05);
    this->declare_parameter("max_linear_speed", 0.5);
    this->declare_parameter("max_linear_accel", 0.3);
    this->declare_parameter("control_rate", 50.0);

    double rate = this->get_parameter("control_rate").as_double();
    dt_ = 1.0 / rate;

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&VelocityController::cmdVelCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&VelocityController::odomCallback, this, std::placeholders::_1));

    motor_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor_left_speed", 10);

    pid_ = std::make_unique<autocar_control::PIDController>(
      this->get_parameter("kp").as_double(),
      this->get_parameter("ki").as_double(),
      this->get_parameter("kd").as_double(),
      -this->get_parameter("max_linear_speed").as_double(),
      this->get_parameter("max_linear_speed").as_double(),
      10.0);

    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
      std::bind(&VelocityController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "velocity_controller initialized");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_linear_ = msg->linear.x;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_linear_ = msg->twist.twist.linear.x;
  }

  void controlLoop() {
    double cmd = pid_->compute(target_linear_, current_linear_, dt_);

    double max_accel = this->get_parameter("max_linear_accel").as_double();
    double max_change = max_accel * dt_;
    cmd = std::clamp(cmd, prev_cmd_ - max_change, prev_cmd_ + max_change);
    prev_cmd_ = cmd;

    auto msg = std_msgs::msg::Float32();
    msg.data = static_cast<float>(cmd);
    motor_pub_->publish(msg);
  }

  double target_linear_{0.0};
  double current_linear_{0.0};
  double prev_cmd_{0.0};
  double dt_{0.02};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_pub_;
  std::unique_ptr<autocar_control::PIDController> pid_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityController>());
  rclcpp::shutdown();
  return 0;
}
