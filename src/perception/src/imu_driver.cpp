#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuDriver : public rclcpp::Node {
public:
  ImuDriver() : Node("imu_driver") {
    this->declare_parameter("i2c_bus", 1);
    this->declare_parameter("address", 0x68);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("publish_rate", 100.0);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    double rate = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&ImuDriver::publishImu, this));

    RCLCPP_INFO(this->get_logger(), "imu_driver initialized");
  }

private:
  void publishImu() {
    auto imu = sensor_msgs::msg::Imu();
    imu.header.stamp = this->now();
    imu.header.frame_id = this->get_parameter("frame_id").as_string();
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.z = 0.0;
    imu.linear_acceleration.x = 0.0;
    imu_pub_->publish(imu);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuDriver>());
  rclcpp::shutdown();
  return 0;
}
