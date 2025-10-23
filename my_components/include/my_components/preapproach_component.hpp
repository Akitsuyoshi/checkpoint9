#ifndef COMPOSITION__PREAPPROACH_COMPONENT_HPP_
#define COMPOSITION__PREAPPROACH_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>
#include <mutex>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace my_components {

class PreApproach : public rclcpp::Node {
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;

public:
  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions &options);

  enum RobotState {
    STOPPED,
    MOVING,
    TURNING,
  };

protected:
  void callback(const LaserScan::SharedPtr msg);
  void callback(const Odometry::SharedPtr msg);
  void callback();

  double norm_angle(double angle) const;
  void set_robot_state(RobotState rs);
  RobotState get_robot_state();

private:
  std::mutex mutex_;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;
  rclcpp::CallbackGroup::SharedPtr mutually_exclusive_group_;

  RobotState rs_;
  double init_yaw_;
  double obstacle_;
  double degrees_;
};

} // namespace my_components

#endif // COMPOSITION__PREAPPROACH_COMPONENT_HPP_