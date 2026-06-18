#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

class GpsDriver : public rclcpp::Node {
public:
  GpsDriver() : Node("gps_driver") {
    this->declare_parameter("port", "/dev/ttyAMA0");
    this->declare_parameter("baud_rate", 9600);
    this->declare_parameter("frame_id", "gps_link");
    this->declare_parameter("publish_rate", 10.0);

    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);

    double rate = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&GpsDriver::publishFix, this));

    RCLCPP_INFO(this->get_logger(), "gps_driver initialized");
  }

private:
  void publishFix() {
    auto fix = sensor_msgs::msg::NavSatFix();
    fix.header.stamp = this->now();
    fix.header.frame_id = this->get_parameter("frame_id").as_string();
    fix.latitude = 39.9042;
    fix.longitude = 116.4074;
    fix.altitude = 50.0;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    fix_pub_->publish(fix);
  }

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsDriver>());
  rclcpp::shutdown();
  return 0;
}
