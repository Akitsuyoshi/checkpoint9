#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <cstddef>
#include <tf2_ros/buffer.h>

class ApproachShelf : public rclcpp::Node {
  using Empty = std_srvs::srv::Empty;
  using LaserScan = sensor_msgs::msg::LaserScan;

public:
  ApproachShelf() : Node("approach_shelf_server") {
    std::string service_n = "/approach_shelf";
    service_ = create_service<Empty>(
        service_n, [this](const std::shared_ptr<Empty::Request> request,
                          std::shared_ptr<Empty::Response> response) {
          callback(request, response);
        });

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    laser_sub_ = create_subscription<LaserScan>(
        "/scan", qos,
        [this](const LaserScan::SharedPtr msg) { callback(msg); });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "Service Server Ready");
  }

private:
  void callback(const std::shared_ptr<Empty::Request> request,
                std::shared_ptr<Empty::Response> response) {
    RCLCPP_INFO(get_logger(), "Service Requested");
    (void)request;
    (void)response;

    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (!detect_shelf_legs()) {
        RCLCPP_INFO(get_logger(), "2 legs are not found");
        timer_->cancel();
        return;
      }
      publish_tf();
      lookup_tf();
    });

    RCLCPP_INFO(get_logger(), "Service Completed");
  }

  void callback(const LaserScan::SharedPtr msg) { scan_msg_ = msg; }

  bool detect_shelf_legs() {
    std::vector<std::vector<float>> clusters;
    std::vector<float> temp_cluster;
    for (size_t i = 0; i < scan_msg_->intensities.size(); ++i) {
      if (scan_msg_->intensities[i] == 8000.0) {
        temp_cluster.push_back(i);
      } else if (!temp_cluster.empty()) {
        clusters.push_back(temp_cluster);
        temp_cluster.clear();
      }
    }

    if (clusters.size() != 2) {
      return false;
    }

    auto leg1 = get_xy(clusters[0][clusters[0].size() / 2]);
    auto leg2 = get_xy(clusters[1][clusters[1].size() / 2]);
    mx_ = (leg1.first + leg2.first) / 2.0;
    my_ = (leg1.second + leg2.second) / 2.0;

    return true;
  }

  std::pair<float, float> get_xy(int idx) {
    float range = scan_msg_->ranges[idx];
    float angle = scan_msg_->angle_min + idx * scan_msg_->angle_increment;
    return std::make_pair(range * std::cos(angle), range * std::sin(angle));
  }

  void publish_tf() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = scan_msg_->header.frame_id;
    t.child_frame_id = "cart_frame";

    t.transform.translation.x = mx_;
    t.transform.translation.y = my_;
    t.transform.translation.z = 0.0;
    // no rotation
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);
  }

  void lookup_tf() {
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
    error_distance_ = std::hypot(dx, dy);
    error_yaw_ = std::atan2(dy, dx);

    RCLCPP_INFO(get_logger(), "error distance:%.3f m", error_distance_);
    RCLCPP_INFO(get_logger(), "error yaw:%.3f rad", error_yaw_);
  }

  rclcpp::Service<Empty>::SharedPtr service_;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  LaserScan::SharedPtr scan_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  double mx_;
  double my_;
  double error_distance_;
  double error_yaw_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachShelf>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}