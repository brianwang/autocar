#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "autocar_perception/kalman_filter.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class LocalizationFusion : public rclcpp::Node {
public:
  LocalizationFusion() : Node("localization_fusion"), ekf_(6, 3) {
    this->declare_parameter("gps_min_satellites", 6);
    this->declare_parameter("gps_max_hdop", 2.0);
    this->declare_parameter("gps_loss_timeout_s", 3.0);

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", 10, std::bind(&LocalizationFusion::gpsCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&LocalizationFusion::imuCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&LocalizationFusion::odomCallback, this, std::placeholders::_1));

    fused_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_fused", 10);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6);
    ekf_.setInitialState(x0, P0);

    timer_ = this->create_wall_timer(50ms,
      std::bind(&LocalizationFusion::fusionLoop, this));

    RCLCPP_INFO(this->get_logger(), "localization_fusion initialized");
  }

private:
  enum class Mode { OUTDOOR, INDOOR, TRANSITION };

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    last_gps_time_ = this->now();
    gps_lat_ = msg->latitude;
    gps_lon_ = msg->longitude;
    gps_alt_ = msg->altitude;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_yaw_rate_ = msg->angular_velocity.z;
    imu_ax_ = msg->linear_acceleration.x;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    has_odom_ = true;
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_linear_ = msg->twist.twist.linear.x;
  }

  void fusionLoop() {
    double dt = 0.05;
    auto now = this->now();
    double gps_age = (now - last_gps_time_).seconds();
    bool gps_valid = gps_age < 3.0;

    if (gps_valid && mode_ != Mode::OUTDOOR) {
      mode_ = Mode::OUTDOOR;
      RCLCPP_INFO(this->get_logger(), "switching to OUTDOOR mode");
    } else if (!gps_valid && mode_ != Mode::INDOOR) {
      mode_ = Mode::INDOOR;
      RCLCPP_INFO(this->get_logger(), "switching to INDOOR mode");
    }

    // Predict step
    auto f = [this](const Eigen::VectorXd& s, const Eigen::VectorXd& u, double dt) {
      Eigen::VectorXd pred(6);
      double yaw = s(2);
      pred(0) = s(0) + s(3) * cos(yaw) * dt;
      pred(1) = s(1) + s(3) * sin(yaw) * dt;
      pred(2) = s(2) + s(5) * dt;
      pred(3) = s(3) + u(0) * dt;
      pred(4) = 0.0;
      pred(5) = s(5);
      return pred;
    };

    auto F = [](const Eigen::VectorXd& s, const Eigen::VectorXd&, double dt) {
      Eigen::MatrixXd Fj(6, 6);
      Fj.setIdentity();
      double y = s(2), v = s(3);
      Fj(0, 2) = -v * sin(y) * dt;
      Fj(0, 3) = cos(y) * dt;
      Fj(1, 2) = v * cos(y) * dt;
      Fj(1, 3) = sin(y) * dt;
      Fj(2, 5) = dt;
      return Fj;
    };

    Eigen::VectorXd u(1); u << imu_ax_;
    ekf_.predict(u, dt, f, F);

    // Update with odometry (always available)
    if (has_odom_) {
      auto h_odom = [](const Eigen::VectorXd& s) {
        Eigen::VectorXd z(2);
        z << s(3), s(5);
        return z;
      };
      auto H_odom = [](const Eigen::VectorXd&) {
        Eigen::MatrixXd H(2, 6);
        H.setZero();
        H(0, 3) = 1.0;
        H(1, 5) = 1.0;
        return H;
      };
      Eigen::VectorXd z(2);
      z << odom_linear_, imu_yaw_rate_;
      ekf_.update(z, h_odom, H_odom);
    }

    // Publish fused odometry
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = ekf_.state()(0);
    odom.pose.pose.position.y = ekf_.state()(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, ekf_.state()(2));
    odom.pose.pose.orientation = tf2::toMsg(q);
    fused_pub_->publish(odom);
  }

  autocar_perception::ExtendedKalmanFilter ekf_;
  Mode mode_{Mode::OUTDOOR};
  rclcpp::Time last_gps_time_{0, 0, RCL_ROS_TIME};
  double gps_lat_{0.0}, gps_lon_{0.0}, gps_alt_{0.0};
  double imu_yaw_rate_{0.0}, imu_ax_{0.0};
  double odom_x_{0.0}, odom_y_{0.0}, odom_linear_{0.0};
  bool has_odom_{false};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationFusion>());
  rclcpp::shutdown();
  return 0;
}
