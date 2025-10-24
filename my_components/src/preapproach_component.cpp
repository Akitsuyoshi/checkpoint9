#include "my_components/preapproach_component.hpp"

using namespace std::chrono_literals;

namespace my_components {
PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("preapproach", options) {

  reentrant_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  mutually_exclusive_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = reentrant_group_;
  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
  laser_sub_ = create_subscription<LaserScan>(
      "/scan", qos,
      [this](const LaserScan::SharedPtr msg) { return callback(msg); },
      sub_options);
  odom_sub_ = create_subscription<Odometry>(
      "/odom", 10,
      [this](const Odometry::SharedPtr msg) { return callback(msg); },
      sub_options);
  pub_ =
      create_publisher<Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
  timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() { return callback(); },
      mutually_exclusive_group_);

  obstacle_ = 0.3;
  degrees_ = (-90) * M_PI / 180.0;

  //   param
  declare_parameter<bool>("final_approach", false);

  //   Move robot
  set_robot_state(MOVING);
  RCLCPP_INFO(get_logger(), "Node is ready");
}

void PreApproach::callback(const LaserScan::SharedPtr msg) {
  if (get_robot_state() != MOVING) {
    return;
  }
  int size = static_cast<int>(msg->ranges.size()) - 1;
  double forward_range = M_PI / 4;
  int forward_start =
      std::clamp(static_cast<int>(std::round((-forward_range - msg->angle_min) /
                                             msg->angle_increment)),
                 0, size);
  int forward_end =
      std::clamp(static_cast<int>(std::round((forward_range - msg->angle_min) /
                                             msg->angle_increment)),
                 0, size);

  auto min_foward_distance = *std::min_element(
      msg->ranges.begin() + forward_start, msg->ranges.begin() + forward_end);
  if (min_foward_distance > obstacle_ + 0.05) {
    set_robot_state(MOVING);
  } else {
    RCLCPP_INFO(get_logger(), "Obstacle detected (%.3f m < %.3f m)",
                min_foward_distance, obstacle_);
    set_robot_state(TURNING);
  }
}

void PreApproach::callback(const Odometry::SharedPtr msg) {
  if (get_robot_state() != TURNING) {
    return;
  }
  auto q = msg->pose.pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  if (!init_yaw_) {
    init_yaw_ = yaw;
  }
  double yaw_diff = norm_angle(yaw - init_yaw_);
  if (std::abs(yaw_diff) >= std::abs(degrees_) - 0.02) {
    RCLCPP_INFO(get_logger(), "Turn completed: rotated %.3f radians", yaw_diff);
    set_robot_state(STOPPED);
    if (!get_parameter("final_approach").as_bool()) {
      std::thread([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rclcpp::shutdown();
      }).detach();
    }
  }
}

void PreApproach::callback() {
  Twist cmd;
  switch (get_robot_state()) {
  case STOPPED:
    break;
  case MOVING:
    cmd.linear.x = 0.4;
    break;
  case TURNING:
    cmd.angular.z = (degrees_ > 0) ? M_PI / 8 : -M_PI / 8;
    break;
  }
  pub_->publish(cmd);
}

double PreApproach::norm_angle(double angle) const {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

void PreApproach::set_robot_state(RobotState rs) {
  std::lock_guard<std::mutex> lck(mutex_);
  if (rs != rs_) {
    RCLCPP_INFO(get_logger(), "Robot state changed");
  }
  rs_ = rs;
}

PreApproach::RobotState PreApproach::get_robot_state() {
  std::lock_guard<std::mutex> lck(mutex_);
  return rs_;
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)