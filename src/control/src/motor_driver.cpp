#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class MotorDriver : public rclcpp::Node {
public:
  MotorDriver() : Node("motor_driver") {
    this->declare_parameter("wheel_radius", 0.032);
    this->declare_parameter("wheel_base", 0.178);
    this->declare_parameter("encoder_ppr", 48);
    this->declare_parameter("gear_ratio", 1.0);
    this->declare_parameter("pwm_pin", 12);
    this->declare_parameter("dir_pin", 13);
    this->declare_parameter("max_pwm", 255);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("/battery_voltage", 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MotorDriver::cmdVelCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    control_timer_ = this->create_wall_timer(
      20ms, std::bind(&MotorDriver::controlLoop, this));
    odom_timer_ = this->create_wall_timer(
      50ms, std::bind(&MotorDriver::publishOdometry, this));

    RCLCPP_INFO(this->get_logger(), "motor_driver initialized");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_ = *msg;
    double left = msg->linear.x - msg->angular.z * wheel_base_ / 2.0;
    double right = msg->linear.x + msg->angular.z * wheel_base_ / 2.0;
    target_left_speed_ = static_cast<int>(std::clamp(left / max_speed_ * max_pwm_, -max_pwm_, max_pwm_));
    target_right_speed_ = static_cast<int>(std::clamp(right / max_speed_ * max_pwm_, -max_pwm_, max_pwm_));
  }

  void controlLoop() {
    // Hardware PWM write would go here:
    // - GPIO write to pwm_pin with target_left_speed_/target_right_speed_
    // - GPIO write to dir_pin for direction
    // In simulation mode, just track encoder ticks from simulated motor response
    current_left_speed_ = target_left_speed_;
    current_right_speed_ = target_right_speed_;
  }

  void publishOdometry() {
    double wheel_radius = this->get_parameter("wheel_radius").as_double();
    double wheel_base = this->get_parameter("wheel_base").as_double();
    double max_pwm = this->get_parameter("max_pwm").as_double();

    double left_dist = (current_left_speed_ / max_pwm) * max_speed_ * 0.05;
    double right_dist = (current_right_speed_ / max_pwm) * max_speed_ * 0.05;

    double linear = (left_dist + right_dist) / 2.0;
    double angular = (right_dist - left_dist) / wheel_base;

    x_ += linear * std::cos(theta_);
    y_ += linear * std::sin(theta_);
    theta_ += angular;

    auto now = this->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = linear / 0.05;
    odom.twist.twist.angular.z = angular / 0.05;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    std_msgs::msg::Float32 battery;
    battery.data = 12.0;
    battery_pub_->publish(battery);
  }

  double x_{0.0}, y_{0.0}, theta_{0.0};
  double wheel_radius_{0.032}, wheel_base_{0.178};
  double max_speed_{0.5};
  int max_pwm_{255};
  int target_left_speed_{0}, target_right_speed_{0};
  int current_left_speed_{0}, current_right_speed_{0};
  geometry_msgs::msg::Twist last_cmd_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriver>());
  rclcpp::shutdown();
  return 0;
}
