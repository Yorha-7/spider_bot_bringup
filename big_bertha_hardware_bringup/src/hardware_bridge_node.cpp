#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "big_bertha_hardware_bringup/router_bridge.hpp"

class HardwareBridgeNode : public rclcpp::Node {
public:
  HardwareBridgeNode()
  : Node("hardware_bridge"), bridge_(nullptr) {
    declare_parameter("imu_frame_id", "imu_link");
    declare_parameter("imu_rate", 30.0);
    declare_parameter("servo_count", 12);
    declare_parameter("servo_freq", 50);
    declare_parameter("servo_angle_min", -1.57);
    declare_parameter("servo_angle_max", 1.57);
    declare_parameter("servo_pulse_min", 500);
    declare_parameter("servo_pulse_max", 2500);
    declare_parameter("router_socket", "/var/run/arduino-router.sock");
    declare_parameter("mock", false);

    imu_frame_ = get_parameter("imu_frame_id").as_string();
    imu_rate_ = get_parameter("imu_rate").as_double();
    servo_count_ = get_parameter("servo_count").as_int();
    angle_min_ = get_parameter("servo_angle_min").as_double();
    angle_max_ = get_parameter("servo_angle_max").as_double();
    pulse_min_ = get_parameter("servo_pulse_min").as_int();
    pulse_max_ = get_parameter("servo_pulse_max").as_int();
    bool mock = get_parameter("mock").as_bool();

    init_bridge(mock);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 1);
    cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 1,
        std::bind(&HardwareBridgeNode::cmd_callback, this,
                  std::placeholders::_1));
    ping_srv_ = create_service<std_srvs::srv::Trigger>(
        "/hardware/ping",
        std::bind(&HardwareBridgeNode::ping_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    auto period = std::chrono::duration<double>(1.0 / imu_rate_);
    imu_timer_ = create_wall_timer(
        period, std::bind(&HardwareBridgeNode::publish_imu, this));

    RCLCPP_INFO(get_logger(),
                "Started | IMU @ %.1f Hz on /%s | %ld servos | %s",
                imu_rate_, imu_frame_.c_str(), servo_count_,
                mock ? "MOCK" : "HW");
  }

private:
  void init_bridge(bool mock) {
    if (mock) {
      RCLCPP_WARN(get_logger(), "Mock mode enabled — no hardware");
      bridge_ = std::make_unique<MockBridge>();
      return;
    }
    try {
      std::string sock_path = get_parameter("router_socket").as_string();
      bridge_ = std::make_unique<RouterBridge>(sock_path);
      RCLCPP_INFO(get_logger(), "Connected to router at %s",
                  sock_path.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "RouterBridge init failed: %s", e.what());
      RCLCPP_WARN(get_logger(), "Falling back to mock mode");
      bridge_ = std::make_unique<MockBridge>();
    }
  }

  void publish_imu() {
    if (!bridge_) return;
    std::string raw;
    try {
      raw = bridge_->call("read_imu");
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "read_imu failed: %s", e.what());
      return;
    }

    std::vector<double> parts;
    std::istringstream iss(raw);
    double val;
    while (iss >> val) {
      parts.push_back(val);
    }
    if (parts.size() < 6) return;

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = now();
    msg.header.frame_id = imu_frame_;
    msg.angular_velocity.x = parts[0];
    msg.angular_velocity.y = parts[1];
    msg.angular_velocity.z = parts[2];
    msg.linear_acceleration.x = parts[3];
    msg.linear_acceleration.y = parts[4];
    msg.linear_acceleration.z = parts[5];
    imu_pub_->publish(msg);
  }

  void cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!bridge_) return;
    if (static_cast<int64_t>(msg->data.size()) < servo_count_) {
      RCLCPP_WARN(get_logger(),
                  "cmd has %zu values, expected %ld",
                  msg->data.size(), servo_count_);
      return;
    }

    std::ostringstream pulse_str;
    for (int64_t i = 0; i < servo_count_; ++i) {
      if (i > 0) pulse_str << " ";
      pulse_str << rad_to_pulse(msg->data[i]);
    }

    try {
      bridge_->call("set_servos", pulse_str.str());
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "set_servos failed: %s", e.what());
    }
  }

  int rad_to_pulse(double rad) const {
    double t = (rad - angle_min_) / (angle_max_ - angle_min_);
    t = std::max(0.0, std::min(1.0, t));
    return static_cast<int>(pulse_min_ + t * (pulse_max_ - pulse_min_));
  }

  void ping_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    if (!bridge_) {
      response->success = false;
      response->message = "bridge not available";
      return;
    }
    try {
      std::string result = bridge_->call("ping");
      response->success = (result == "ok");
      response->message = result;
    } catch (const std::exception & e) {
      response->success = false;
      response->message = e.what();
    }
  }

  // -- members --
  std::unique_ptr<BridgeInterface> bridge_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ping_srv_;
  rclcpp::TimerBase::SharedPtr imu_timer_;

  std::string imu_frame_;
  double imu_rate_;
  int64_t servo_count_;
  double angle_min_;
  double angle_max_;
  int pulse_min_;
  int pulse_max_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
