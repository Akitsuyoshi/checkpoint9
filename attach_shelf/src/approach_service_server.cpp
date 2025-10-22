#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

class ApproachShelf : public rclcpp::Node {
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;
  using GoToLoading = custom_interfaces::srv::GoToLoading;

public:
  ApproachShelf() : Node("approach_shelf_server") {
    std::string service_n = "/approach_shelf";
    service_ = create_service<GoToLoading>(
        service_n, [this](const std::shared_ptr<GoToLoading::Request> request,
                          std::shared_ptr<GoToLoading::Response> response) {
          callback(request, response);
        });

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    mutually_exclusive_group_1_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mutually_exclusive_group_2_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options_laser;
    sub_options_laser.callback_group = mutually_exclusive_group_1_;
    laser_sub_ = create_subscription<LaserScan>(
        "/scan", qos, [this](const LaserScan::SharedPtr msg) { callback(msg); },
        sub_options_laser);
    rclcpp::SubscriptionOptions sub_options_odom;
    sub_options_odom.callback_group = mutually_exclusive_group_2_;
    odom_sub_ = create_subscription<Odometry>(
        "/odom", 10, [this](const Odometry::SharedPtr msg) { callback(msg); },
        sub_options_odom);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<Twist>("/diffbot_base_controller/cmd_vel_unstamped",
                                   10);
    pub_elevaup_ = create_publisher<std_msgs::msg::String>("/elevator_up", 10);

    RCLCPP_INFO(get_logger(), "Service Server Ready");
  }

private:
  void callback(const std::shared_ptr<GoToLoading::Request> request,
                std::shared_ptr<GoToLoading::Response> response) {
    RCLCPP_INFO(get_logger(), "Service Requested");
    if (!odom_msg_ || !scan_msg_) {
      response->complete = false;
      RCLCPP_INFO(get_logger(), "Odom/Scan data is not available");
      return;
    }
    if (!detect_shelf_legs()) {
      response->complete = false;
      RCLCPP_INFO(get_logger(), "2 legs are not found");
      return;
    }
    if (!request->attach_to_shelf) {
      publish_tf(true);
      response->complete = true;
      RCLCPP_INFO(get_logger(), "Published statif tf");
      return;
    }

    reached_tf_coord = false;
    started_forward = false;
    finished_forward = false;

    auto promise1 = std::make_shared<std::promise<void>>();
    std::future<void> future1 = promise1->get_future();
    timer_tf_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this, promise1]() {
          if (reached_tf_coord) {
            timer_tf_->cancel();
            promise1->set_value();
            return;
          }
          detect_shelf_legs();
          publish_tf();
          follow_tf();
        },
        mutually_exclusive_group_1_);

    if (future1.wait_for(std::chrono::seconds(15)) !=
        std::future_status::ready) {
      timer_tf_->cancel();
      response->complete = false;
      RCLCPP_INFO(get_logger(), "Service timedout");
      return;
    }
    auto promise2 = std::make_shared<std::promise<void>>();
    std::future<void> future2 = promise2->get_future();
    timer_final_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this, promise2]() {
          if (finished_forward) {
            timer_final_->cancel();
            promise2->set_value();
            return;
          }
          final_approach();
        },
        mutually_exclusive_group_2_);
    if (future2.wait_for(std::chrono::seconds(15)) !=
        std::future_status::ready) {
      timer_final_->cancel();
      response->complete = false;
      RCLCPP_INFO(get_logger(), "Service timedout");
      return;
    }
    response->complete = true;
    RCLCPP_INFO(get_logger(), "Service completed");
  }

  void callback(const LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lck(mutex_);
    scan_msg_ = msg;
  }
  void callback(const Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lck(mutex_);
    odom_msg_ = msg;
  }

  bool detect_shelf_legs() {
    std::unique_lock<std::mutex> lck(mutex_);
    const auto intensities = scan_msg_->intensities;
    const auto ranges = scan_msg_->ranges;
    const auto angle_min = scan_msg_->angle_min;
    const auto angle_increment = scan_msg_->angle_increment;
    lck.unlock();
    std::vector<std::vector<float>> clusters;
    std::vector<float> temp_cluster;
    for (size_t i = 0; i < intensities.size(); ++i) {
      if (intensities[i] == 8000.0) {
        temp_cluster.push_back(i);
      } else if (!temp_cluster.empty()) {
        clusters.push_back(temp_cluster);
        temp_cluster.clear();
      }
    }

    if (clusters.size() != 2) {
      return false;
    }

    auto leg1 = get_xy(clusters[0][clusters[0].size() / 2], ranges, angle_min,
                       angle_increment);
    auto leg2 = get_xy(clusters[1][clusters[1].size() / 2], ranges, angle_min,
                       angle_increment);
    mx_ = (leg1.first + leg2.first) / 2.0;
    my_ = (leg1.second + leg2.second) / 2.0;

    return true;
  }

  std::pair<float, float> get_xy(int idx, const std::vector<float> &ranges,
                                 float angle_min, float angle_increment) {
    float range = ranges[idx];
    float angle = angle_min + idx * angle_increment;
    return std::make_pair(range * std::cos(angle), range * std::sin(angle));
  }

  void publish_tf(bool is_static = false) {
    std::unique_lock<std::mutex> lck(mutex_);
    const auto header_framer_id = scan_msg_->header.frame_id;
    lck.unlock();
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = header_framer_id;
    t.child_frame_id = "cart_frame";

    t.transform.translation.x = mx_;
    t.transform.translation.y = my_;
    t.transform.translation.z = 0.0;
    // no rotation
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    if (is_static) {
      tf_static_broadcaster_->sendTransform(t);
    } else {
      tf_broadcaster_->sendTransform(t);
    }
  }

  void follow_tf() {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("robot_base_footprint", "cart_frame",
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(get_logger(),
                  "Could not transform robot_base_footprint to cart_frame: %s",
                  e.what());
      return;
    }
    double dx = t.transform.translation.x;
    double dy = t.transform.translation.y;
    double error_distance = std::hypot(dx, dy);
    double error_yaw = std::atan2(dy, dx);
    move_robot(error_distance, error_yaw, 0.1, [this]() {
      reached_tf_coord = true;
      RCLCPP_INFO(get_logger(), "Finished moving");
    });
  }

  void final_approach() {
    std::unique_lock<std::mutex> lck(mutex_);
    double x = odom_msg_->pose.pose.position.x;
    double y = odom_msg_->pose.pose.position.y;
    double yaw = get_yaw(odom_msg_->pose.pose.orientation);
    lck.unlock();
    if (!started_forward) {
      goal_x_ = x + 0.3 * std::cos(yaw);
      goal_y_ = y + 0.3 * std::sin(yaw);
      started_forward = true;
      RCLCPP_INFO(get_logger(), "Moving 30cm forward based on odom");
    }
    double dx = goal_x_ - x;
    double dy = goal_y_ - y;
    double error_distance = std::hypot(dx, dy);
    double error_yaw = std::atan2(dy, dx) - yaw;
    move_robot(error_distance, error_yaw, 0.05, [this]() {
      finished_forward = true;
      pub_elevaup_->publish(std_msgs::msg::String());
      RCLCPP_INFO(get_logger(), "Published elevator UP");
    });
  }

  double get_yaw(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  void move_robot(double error_distance, double error_yaw,
                  double error_tolerance, std::function<void()> callback) {

    RCLCPP_INFO(get_logger(), "error distance:%.3f m", error_distance);
    RCLCPP_INFO(get_logger(), "error yaw:%.3f rad", error_yaw);

    Twist cmd;
    if (error_distance < error_tolerance) {
      pub_->publish(cmd);
      callback();
    } else {
      cmd.linear.x = std::max(0.2, error_distance * 0.7);
      cmd.angular.z = std::clamp(error_yaw * 0.5, -M_PI / 6, M_PI / 6);
      pub_->publish(cmd);
    }
  }

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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachShelf>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}