#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>

class PreApproach : public rclcpp::Node {
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Twist = geometry_msgs::msg::Twist;

public:
  PreApproach() : Node("pre_approach_node"), rs_(STOPPED) {
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    laser_sub_ = create_subscription<LaserScan>(
        "/scan", qos,
        [this](const LaserScan::SharedPtr msg) { return callback(msg); });
    pub_ = create_publisher<Twist>("/diffbot_base_controller/cmd_vel_unstamped",
                                   10);
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               [this]() { return callback(); });
    RCLCPP_INFO(get_logger(), "Node is ready");
  }

  enum RobotState {
    STOPPED,
    MOVING,
    TURNING,
  };

private:
  void callback(const LaserScan::SharedPtr msg) {
    if (get_robot_state() == TURNING) {
      return;
    }
    int size = static_cast<int>(msg->ranges.size()) - 1;
    // indx for forward +-45 degree on linear.x axis;
    double forward_range = M_PI / 8;
    int forward_start = std::clamp(
        static_cast<int>(std::round((-forward_range - msg->angle_min) /
                                    msg->angle_increment)),
        0, size);
    int forward_end = std::clamp(
        static_cast<int>(std::round((forward_range - msg->angle_min) /
                                    msg->angle_increment)),
        0, size);

    auto min_foward_distance = *std::min_element(
        msg->ranges.begin() + forward_start, msg->ranges.begin() + forward_end);
    if (min_foward_distance > 0.3) {
      set_robot_state(MOVING);
    } else {
      set_robot_state(TURNING);
    }
  }

  void callback() {
    Twist cmd;
    switch (get_robot_state()) {
    case STOPPED:
      break;
    case MOVING:
      cmd.linear.x = 0.5;
      break;
    case TURNING:
      cmd.angular.z = M_PI / 4;
      break;
    }
    pub_->publish(cmd);
  }

  void set_robot_state(RobotState rs) { rs_ = rs; }
  RobotState get_robot_state() { return rs_; }

  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  RobotState rs_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproach>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}