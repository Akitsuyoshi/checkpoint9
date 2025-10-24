#ifndef COMPOSITION__PREAPPROACH_COMPONENT_HPP_
#define COMPOSITION__PREAPPROACH_COMPONENT_HPP_

#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <tf2_ros/buffer.h>
#include <thread>

namespace my_components {

class AttachServer : public rclcpp::Node {
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;
  using GoToLoading = custom_interfaces::srv::GoToLoading;

public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

protected:
  void callback(const std::shared_ptr<GoToLoading::Request> request,
                std::shared_ptr<GoToLoading::Response> response);
  void callback(const LaserScan::SharedPtr msg);
  void callback(const Odometry::SharedPtr msg);

  bool detect_shelf_legs();
  std::pair<float, float> get_xy(int idx, const std::vector<float> &ranges,
                                 float angle_min, float angle_increment) const;
  void publish_tf(bool is_static = false);
  void follow_tf();
  void final_approach();
  double get_yaw(const geometry_msgs::msg::Quaternion &q) const;
  void move_robot(double error_distance, double error_yaw,
                  double error_tolerance, std::function<void()> callback);

private:
  std::mutex mutex_;
  rclcpp::Service<GoToLoading>::SharedPtr service_;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::CallbackGroup::SharedPtr mutually_exclusive_group_1_;
  rclcpp::CallbackGroup::SharedPtr mutually_exclusive_group_2_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_elevaup_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  LaserScan::SharedPtr scan_msg_;
  Odometry::SharedPtr odom_msg_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_tf_;
  rclcpp::TimerBase::SharedPtr timer_final_;

  double mx_;
  double my_;
  double goal_x_;
  double goal_y_;
  bool reached_tf_coord;
  bool started_forward;
  bool finished_forward;
};

} // namespace my_components

#endif // COMPOSITION__ATTACHSERVER_COMPONENT_HPP_