#include "my_components/attachclient_component.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client", options) {

  std::string n_service = "/approach_shelf";
  client_ = create_client<GoToLoading>(n_service);
  // Wait for the service to be available
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service %s not available, waiting again...",
                n_service.c_str());
  }

  // One-shot timer to call the service once
  timer_ = create_wall_timer(100ms, [this]() {
    callback();
    timer_->cancel();
  });
}

void AttachClient::callback() {
  auto request = std::make_shared<GoToLoading::Request>();
  request->attach_to_shelf = true;

  auto fut = client_->async_send_request(
      request, [this](rclcpp::Client<GoToLoading>::SharedFuture result) {
        auto response = result.get();
        RCLCPP_INFO(get_logger(), "Service Response: %s",
                    response->complete ? "true" : "false");

        std::thread([]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          rclcpp::shutdown();
        }).detach();
      });
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)