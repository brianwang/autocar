#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "autocar_planning/dwa_planner.hpp"
#include <mutex>

class LocalPlanner : public rclcpp::Node {
public:
  LocalPlanner() : Node("local_planner"), dwa_(autocar_planning::DWAConfig()) {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LocalPlanner::scanCallback, this, std::placeholders::_1));
    global_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/global_path", 10, std::bind(&LocalPlanner::globalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_fused", 10, std::bind(&LocalPlanner::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&LocalPlanner::planLoop, this));
    RCLCPP_INFO(this->get_logger(), "local_planner initialized");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr m) {
    std::lock_guard<std::mutex> lk(mtx_); last_scan_ = *m;
  }
  void globalCallback(const nav_msgs::msg::Path::SharedPtr m) {
    std::lock_guard<std::mutex> lk(mtx_); global_path_ = *m;
  }
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr m) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = m->header;
    pose.pose = m->pose.pose;
    std::lock_guard<std::mutex> lk(mtx_);
    current_pose_ = pose;
    current_vel_ = m->twist.twist;
  }

  void planLoop() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (global_path_.poses.empty()) return;
    auto cmd = dwa_.plan(current_pose_, current_vel_, global_path_, last_scan_);
    cmd_vel_pub_->publish(cmd);
  }

  autocar_planning::DWAPlanner dwa_;
  std::mutex mtx_;
  sensor_msgs::msg::LaserScan last_scan_;
  nav_msgs::msg::Path global_path_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist current_vel_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
