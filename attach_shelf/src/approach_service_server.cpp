#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <cstddef>

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
    RCLCPP_INFO(get_logger(), "Service Server Ready");
  }

private:
  void callback(const std::shared_ptr<Empty::Request> request,
                std::shared_ptr<Empty::Response> response) {
    RCLCPP_INFO(get_logger(), "Service Requested");
    (void)request;
    (void)response;
    if (!detect_shelf_legs()) {
      RCLCPP_INFO(get_logger(), "2 legs are not found");
      return;
    }
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

  rclcpp::Service<Empty>::SharedPtr service_;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
  LaserScan::SharedPtr scan_msg_;

  double mx_;
  double my_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachShelf>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}