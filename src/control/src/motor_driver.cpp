#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "autocar_control/chassis_protocol.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <string>
#include <mutex>

// ── Platform-specific networking ──
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  typedef SOCKET socket_t;
  constexpr socket_t INVALID_SOCKET_VAL = INVALID_SOCKET;
  #define SOCKERRNO WSAGetLastError()
  #define SHUT_RDWR SD_BOTH
  static int winSockInit() {
    WSADATA wsa;
    return WSAStartup(MAKEWORD(2,2), &wsa);
  }
  static void winSockClean() { WSACleanup(); }
#else
  #include <fcntl.h>
  #include <unistd.h>
  #include <termios.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <netdb.h>
  typedef int socket_t;
  constexpr socket_t INVALID_SOCKET_VAL = -1;
  #define SOCKERRNO errno
  static int winSockInit() { return 0; }     // no-op on POSIX
  static void winSockClean() {}
#endif

using namespace std::chrono_literals;

// ============================================================
//  Port Abstraction: SerialPort | TcpPort
// ============================================================

class PortInterface {
public:
  virtual ~PortInterface() = default;
  virtual bool open(const std::string& addr, int param) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
  virtual int write(const uint8_t* data, size_t len) = 0;
  virtual int read(uint8_t* buf, size_t max_len) = 0;
};

// ── Serial Port (POSIX only) ──
class SerialPort : public PortInterface {
public:
  SerialPort() = default;
  ~SerialPort() override { close(); }

  bool open(const std::string& port, int baud) override {
#ifdef _WIN32
    (void)port; (void)baud; return false;
#else
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) return false;

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { close(); return false; }

    cfsetospeed(&tty, static_cast<speed_t>(baud));
    cfsetispeed(&tty, static_cast<speed_t>(baud));

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; }
    tcflush(fd_, TCIOFLUSH);
    return true;
#endif
  }

  void close() override {
#ifndef _WIN32
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
#endif
  }

  bool isOpen() const override { return fd_ >= 0; }

  int write(const uint8_t* data, size_t len) override {
#ifndef _WIN32
    return static_cast<int>(::write(fd_, data, len));
#else
    (void)data; (void)len; return -1;
#endif
  }

  int read(uint8_t* buf, size_t max_len) override {
#ifndef _WIN32
    return static_cast<int>(::read(fd_, buf, max_len));
#else
    (void)buf; (void)max_len; return -1;
#endif
  }

private:
#ifndef _WIN32
  int fd_ = -1;
#else
  int fd_ = -1;
#endif
};

// ── TCP Port (cross-platform) ──
class TcpPort : public PortInterface {
public:
  TcpPort() = default;
  ~TcpPort() override { close(); }

  bool open(const std::string& host, int port) override {
    sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ == INVALID_SOCKET_VAL) return false;

    // Resolve hostname
    struct hostent* he = gethostbyname(host.c_str());
    if (!he) { close(); return false; }

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    std::memcpy(&addr.sin_addr, he->h_addr_list[0], static_cast<size_t>(he->h_length));
    addr.sin_port = htons(static_cast<uint16_t>(port));

    // Non-blocking connect with timeout
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(sock_, FIONBIO, &mode);
#else
    int flags = fcntl(sock_, F_GETFL, 0);
    fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
#endif

    int rc = ::connect(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (rc < 0) {
#ifdef _WIN32
      if (WSAGetLastError() != WSAEWOULDBLOCK) { close(); return false; }
#else
      if (errno != EINPROGRESS) { close(); return false; }
#endif
      // Wait for connection with poll/select
      struct timeval tv{5, 0};  // 5s timeout
      fd_set fdset;
      FD_ZERO(&fdset);
      FD_SET(sock_, &fdset);
      rc = select(static_cast<int>(sock_ + 1), NULL, &fdset, NULL, &tv);
      if (rc <= 0) { close(); return false; }
    }

    // Restore blocking mode
#ifdef _WIN32
    mode = 0;
    ioctlsocket(sock_, FIONBIO, &mode);
#else
    fcntl(sock_, F_SETFL, flags & ~O_NONBLOCK);
#endif

    // Set receive timeout
#ifdef _WIN32
    int timeout = 100;
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO,
               reinterpret_cast<char*>(&timeout), sizeof(timeout));
#else
    struct timeval tv{0, 100000};  // 100ms
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

    return true;
  }

  void close() override {
    if (sock_ != INVALID_SOCKET_VAL) {
#ifdef _WIN32
      shutdown(sock_, SHUT_RDWR);
      closesocket(sock_);
#else
      ::shutdown(sock_, SHUT_RDWR);
      ::close(sock_);
#endif
      sock_ = INVALID_SOCKET_VAL;
    }
  }

  bool isOpen() const override { return sock_ != INVALID_SOCKET_VAL; }

  int write(const uint8_t* data, size_t len) override {
    return static_cast<int>(::send(sock_, reinterpret_cast<const char*>(data),
                                    static_cast<int>(len), 0));
  }

  int read(uint8_t* buf, size_t max_len) override {
    return static_cast<int>(::recv(sock_, reinterpret_cast<char*>(buf),
                                    static_cast<int>(max_len), 0));
  }

private:
  socket_t sock_ = INVALID_SOCKET_VAL;
};

// ============================================================
//  Motor Driver Node
// ============================================================
class MotorDriver : public rclcpp::Node {
public:
  MotorDriver() : Node("motor_driver") {
    // ── parameters ──
    this->declare_parameter<std::string>("protocol", "serial");
    this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
    this->declare_parameter<int>("serial_baud", 115200);
    this->declare_parameter<std::string>("tcp_host", "127.0.0.1");
    this->declare_parameter<int>("tcp_port", 9999);
    this->declare_parameter<double>("wheel_base", 0.178);
    this->declare_parameter<double>("max_speed", 0.5);
    this->declare_parameter<bool>("simulation", false);

    std::string protocol = this->get_parameter("protocol").as_string();
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    max_speed_  = this->get_parameter("max_speed").as_double();
    bool sim    = this->get_parameter("simulation").as_bool();

    // ── publishers ──
    odom_pub_    = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("/battery_voltage", 10);
    fault_pub_   = this->create_publisher<std_msgs::msg::String>("/motor_fault", 10);

    // ── cmd_vel subscriber ──
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MotorDriver::cmdVelCallback, this, std::placeholders::_1));

    // ── TF broadcaster ──
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ── port init ──
    if (sim) {
      RCLCPP_INFO(this->get_logger(), "simulation mode (no hardware)");
    } else if (protocol == "tcp") {
      port_ = std::make_unique<TcpPort>();
      std::string host = this->get_parameter("tcp_host").as_string();
      int port = this->get_parameter("tcp_port").as_int();
      if (port_->open(host, port)) {
        RCLCPP_INFO(this->get_logger(), "TCP connected: %s:%d", host.c_str(), port);
      } else {
        RCLCPP_WARN(this->get_logger(), "TCP connect failed: %s:%d, fallback sim", host.c_str(), port);
        sim = true;
      }
    } else {  // serial
      port_ = std::make_unique<SerialPort>();
      std::string device = this->get_parameter("serial_port").as_string();
      int baud = this->get_parameter("serial_baud").as_int();
      if (port_->open(device, baud)) {
        RCLCPP_INFO(this->get_logger(), "serial opened: %s (%d baud)", device.c_str(), baud);
      } else {
        RCLCPP_WARN(this->get_logger(), "serial open failed: %s, fallback sim", device.c_str());
        sim = true;
      }
    }

    simulation_ = sim || !port_ || !port_->isOpen();

    // ── timers ──
    control_timer_ = this->create_wall_timer(20ms, std::bind(&MotorDriver::controlLoop, this));
    odom_timer_    = this->create_wall_timer(50ms, std::bind(&MotorDriver::publishOdometry, this));

    RCLCPP_INFO(this->get_logger(), "motor_driver ready [protocol=%s sim=%d]",
                protocol.c_str(), simulation_);
  }

  ~MotorDriver() override {
    if (port_) port_->close();
  }

private:
  // ── cmd_vel ──
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_vx_ = static_cast<float>(msg->linear.x);
    latest_wz_ = static_cast<float>(msg->angular.z);
  }

  // ── control loop @ 50 Hz ──
  void controlLoop() {
    if (simulation_) {
      simTick();
    } else {
      serialTick();
    }
  }

  // ── simulation: direct speed tracking ──
  void simTick() {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    double left  = latest_vx_ - latest_wz_ * wheel_base_ / 2.0;
    double right = latest_vx_ + latest_wz_ * wheel_base_ / 2.0;
    double scale = 1.0;
    double max_whl = std::max(std::abs(left), std::abs(right));
    if (max_whl > max_speed_) scale = max_speed_ / max_whl;
    sim_left_speed_  = left  * scale;
    sim_right_speed_ = right * scale;
  }

  // ── serial/TCP: send control, read feedback ──
  void serialTick() {
    autocar_control::ControlFrame cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd.vx           = latest_vx_;
      cmd.wz           = latest_wz_;
      cmd.motor_enable = true;
    }

    uint8_t tx_buf[autocar_control::CONTROL_FRAME_SIZE];
    autocar_control::encodeControlFrame(cmd, tx_buf);
    port_->write(tx_buf, sizeof(tx_buf));

    // Read feedback
    uint8_t rx_buf[autocar_control::FEEDBACK_FRAME_SIZE];
    int n = port_->read(rx_buf, sizeof(rx_buf));
    if (n >= static_cast<int>(sizeof(rx_buf))) {
      // Search for valid frame at each offset
      for (int off = 0; off <= n - static_cast<int>(sizeof(rx_buf)); ++off) {
        autocar_control::FeedbackFrame fb;
        if (autocar_control::decodeFeedbackFrame(rx_buf + off, fb)) {
          onFeedback(fb);
          break;
        }
      }
    }
  }

  // ── process feedback ──
  void onFeedback(const autocar_control::FeedbackFrame& fb) {
    double left_now  = static_cast<double>(fb.left_mileage);
    double right_now = static_cast<double>(fb.right_mileage);
    double d_left  = left_now  - prev_left_mileage_;
    double d_right = right_now - prev_right_mileage_;

    if (prev_left_mileage_init_) {
      double d_center = (d_left + d_right) / 2.0;
      double d_theta  = (d_right - d_left) / wheel_base_;

      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_x_     += d_center * std::cos(odom_theta_ + d_theta / 2.0);
        odom_y_     += d_center * std::sin(odom_theta_ + d_theta / 2.0);
        odom_theta_ += d_theta;
        odom_vx_     = d_center / 0.05;
        odom_wz_     = d_theta  / 0.05;
      }
    } else {
      prev_left_mileage_init_ = true;
    }

    prev_left_mileage_  = left_now;
    prev_right_mileage_ = right_now;
    battery_voltage_ = static_cast<double>(fb.battery_voltage);

    if (fb.fault_code != 0 && fb.fault_code != prev_fault_) {
      prev_fault_ = fb.fault_code;
      auto msg = std_msgs::msg::String();
      char desc[64];
      std::snprintf(desc, sizeof(desc), "chassis fault code: 0x%02X", fb.fault_code);
      msg.data = desc;
      fault_pub_->publish(msg);
      RCLCPP_WARN(this->get_logger(), "%s", desc);
    }
  }

  // ── publish odometry @ 20 Hz ──
  void publishOdometry() {
    if (simulation_) {
      simPublishOdometry();
    } else {
      hwPublishOdometry();
    }
  }

  void simPublishOdometry() {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    double dt = 0.05;
    double d_center = sim_left_speed_ * dt;
    double d_theta  = (sim_right_speed_ - sim_left_speed_) * dt / wheel_base_;

    odom_x_     += d_center * std::cos(odom_theta_ + d_theta / 2.0);
    odom_y_     += d_center * std::sin(odom_theta_ + d_theta / 2.0);
    odom_theta_ += d_theta;
    odom_vx_     = sim_left_speed_;
    odom_wz_     = (sim_right_speed_ - sim_left_speed_) / wheel_base_;
    battery_voltage_ = 12.0;

    doPublish();
  }

  void hwPublishOdometry() {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    doPublish();
  }

  void doPublish() {
    auto now = this->now();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_theta_);
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x  = odom_vx_;
    odom.twist.twist.angular.z = odom_wz_;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp     = now;
    tf.header.frame_id  = "odom";
    tf.child_frame_id   = "base_link";
    tf.transform.translation.x = odom_x_;
    tf.transform.translation.y = odom_y_;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    std_msgs::msg::Float32 bat;
    bat.data = static_cast<float>(battery_voltage_);
    battery_pub_->publish(bat);
  }

  // ── state ──
  std::unique_ptr<PortInterface> port_;
  double wheel_base_{0.178};
  double max_speed_{0.5};
  bool simulation_{true};

  std::mutex cmd_mutex_;
  float latest_vx_{0.0f};
  float latest_wz_{0.0f};

  double sim_left_speed_{0.0};
  double sim_right_speed_{0.0};

  std::mutex odom_mutex_;
  double odom_x_{0.0}, odom_y_{0.0}, odom_theta_{0.0};
  double odom_vx_{0.0}, odom_wz_{0.0};

  float prev_left_mileage_{0.0f};
  float prev_right_mileage_{0.0f};
  bool  prev_left_mileage_init_{false};
  double battery_voltage_{12.0};
  uint8_t prev_fault_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
};

// ============================================================
int main(int argc, char** argv) {
  winSockInit();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriver>());
  rclcpp::shutdown();
  winSockClean();
  return 0;
}
