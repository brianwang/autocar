#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <random>
#include <algorithm>
#include <cmath>

class SimulationBridge : public rclcpp::Node {
public:
  SimulationBridge() : Node("simulation_bridge") {
    // Declare parameters
    this->declare_parameter("noise_enabled", true);

    bool noise_enabled = this->get_parameter("noise_enabled").as_bool();
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    RCLCPP_INFO(this->get_logger(), "simulation_bridge starting (noise=%s)",
                noise_enabled ? "enabled" : "disabled");

    // === Subscribers (Gazebo topics) ===
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/model/autocar_car/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_pub_->publish(*msg);
      });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/model/autocar_car/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_pub_->publish(*msg);
      });

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/model/autocar_car/scan", 10,
      [this, noise_enabled](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto out = *msg;
        if (noise_enabled) {
          std::normal_distribution<double> nd(0.0, 0.02);
          for (auto& r : out.ranges) {
            r = std::clamp(r + nd(rng_), msg->range_min, msg->range_max);
          }
        }
        scan_pub_->publish(out);
      });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/model/autocar_car/imu", 10,
      [this, noise_enabled](const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto out = *msg;
        if (noise_enabled) {
          std::normal_distribution<double> nd(0.0, 0.01);
          out.angular_velocity.x += nd(rng_);
          out.angular_velocity.y += nd(rng_);
          out.angular_velocity.z += nd(rng_);
          out.linear_acceleration.x += nd(rng_);
          out.linear_acceleration.y += nd(rng_);
          out.linear_acceleration.z += nd(rng_);
        }
        imu_pub_->publish(out);
      });

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/model/autocar_car/gps", 10,
      [this, noise_enabled](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        auto out = *msg;
        if (noise_enabled) {
          std::normal_distribution<double> nd(0.0, 0.0001);
          out.latitude += nd(rng_);
          out.longitude += nd(rng_);
        }
        gps_pub_->publish(out);
      });

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/model/autocar_car/depth_image", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        depth_pub_->publish(*msg);
      });

    // === Publishers (standard topic names) ===
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/depth", 10);

    RCLCPP_INFO(this->get_logger(), "simulation_bridge ready — 6 topic mappings active");
  }

private:
  std::mt19937 rng_{std::random_device{}()};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationBridge>());
  rclcpp::shutdown();
  return 0;
}
