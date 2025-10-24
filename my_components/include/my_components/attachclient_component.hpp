#ifndef COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_
#define COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_

#include "custom_interfaces/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
  using GoToLoading = custom_interfaces::srv::GoToLoading;

public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

protected:
  void callback();

private:
  rclcpp::Client<GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace my_components

#endif // COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_