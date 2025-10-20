#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

class ApproachShelf : public rclcpp::Node {
  using Empty = std_srvs::srv::Empty;

public:
  ApproachShelf() : Node("approach_shelf_server") {
    std::string service_n = "/approach_shelf";
    service_ = create_service<Empty>(
        service_n, [this](const std::shared_ptr<Empty::Request> request,
                          std::shared_ptr<Empty::Response> response) {
          callback(request, response);
        });
    RCLCPP_INFO(get_logger(), "Service Server Ready");
  }

private:
  void callback(const std::shared_ptr<Empty::Request> request,
                std::shared_ptr<Empty::Response> response) {
    RCLCPP_INFO(get_logger(), "Service Requested");
    RCLCPP_INFO(get_logger(), "Service Completed");
  }

  rclcpp::Service<Empty>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachShelf>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}