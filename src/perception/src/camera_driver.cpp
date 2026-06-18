#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraDriver : public rclcpp::Node {
public:
  CameraDriver() : Node("camera_driver") {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/depth", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(66),
      std::bind(&CameraDriver::publishFrame, this));

    RCLCPP_INFO(this->get_logger(), "camera_driver initialized");
  }

private:
  void publishFrame() {
    auto img = sensor_msgs::msg::Image();
    img.header.stamp = this->now();
    img.header.frame_id = "camera_frame";
    img.height = 480;
    img.width = 640;
    img.encoding = "rgb8";
    img.is_bigendian = false;
    img.step = 640 * 3;
    img.data.resize(640 * 480 * 3, 0);
    image_pub_->publish(img);

    auto depth = sensor_msgs::msg::Image();
    depth.header = img.header;
    depth.height = 480;
    depth.width = 640;
    depth.encoding = "16UC1";
    depth.is_bigendian = false;
    depth.step = 640 * 2;
    depth.data.resize(640 * 480 * 2, 0);
    depth_pub_->publish(depth);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDriver>());
  rclcpp::shutdown();
  return 0;
}
