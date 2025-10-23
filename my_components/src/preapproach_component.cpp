#include "my_components/preapproach_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace my_components {
PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("preapproach", options) {

  pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);
  timer_ = create_wall_timer(1s, std::bind(&PreApproach::callback, this));
}

void PreApproach::callback() {
  auto msg = std::make_unique<geometry_msgs::msg::Twist>();
  msg->linear.x = 0.2;
  msg->angular.z = 0.2;
  pub_->publish(std::move(msg));
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)