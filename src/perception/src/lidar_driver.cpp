#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarDriver : public rclcpp::Node {
public:
  LidarDriver() : Node("lidar_driver") {
    this->declare_parameter("port", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 230400);
    this->declare_parameter("frame_id", "laser_frame");
    this->declare_parameter("angle_min", 0.0);
    this->declare_parameter("angle_max", 6.283);
    this->declare_parameter("range_min", 0.05);
    this->declare_parameter("range_max", 12.0);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LidarDriver::publishScan, this));

    RCLCPP_INFO(this->get_logger(), "lidar_driver initialized");
  }

private:
  void publishScan() {
    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = this->now();
    scan.header.frame_id = this->get_parameter("frame_id").as_string();
    scan.angle_min = this->get_parameter("angle_min").as_double();
    scan.angle_max = this->get_parameter("angle_max").as_double();
    scan.angle_increment = 0.01745;
    scan.range_min = this->get_parameter("range_min").as_double();
    scan.range_max = this->get_parameter("range_max").as_double();
    scan.ranges.resize(360, 10.0);
    scan_pub_->publish(scan);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDriver>());
  rclcpp::shutdown();
  return 0;
}
