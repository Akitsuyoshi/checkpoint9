#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

class PreApproach : public rclcpp::Node {
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;
  using GoToLoading = custom_interfaces::srv::GoToLoading;

public:
  PreApproach() : Node("pre_approach_node"), rs_(MOVING) {
    reentrant_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mutually_exclusive_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = reentrant_group_;
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    laser_sub_ = create_subscription<LaserScan>(
        "/scan", qos, [this](const LaserScan::SharedPtr msg) { callback(msg); },
        sub_options);
    odom_sub_ = create_subscription<Odometry>(
        "/odom", 10, [this](const Odometry::SharedPtr msg) { callback(msg); },
        sub_options);
    pub_ = create_publisher<Twist>("/diffbot_base_controller/cmd_vel_unstamped",
                                   10);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() { callback(); },
        mutually_exclusive_group_);

    // Params
    declare_parameter<double>("obstacle", 0.3);
    get_parameter("obstacle", obstacle_);
    declare_parameter<int>("degrees", -90);
    int degrees_int;
    get_parameter("degrees", degrees_int);
    degrees_ = static_cast<double>(degrees_int);
    // convert from degree to radiant
    degrees_ *= M_PI / 180.0;
    declare_parameter<bool>("final_approach", true);

    // Servie client
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

    RCLCPP_INFO(get_logger(), "Node is ready");
  }

  enum RobotState {
    STOPPED,
    MOVING,
    TURNING,
  };

private:
  void callback(const LaserScan::SharedPtr msg) {
    if (get_robot_state() != MOVING) {
      return;
    }
    int size = static_cast<int>(msg->ranges.size()) - 1;
    double forward_range = M_PI / 4;
    int forward_start = std::clamp(
        static_cast<int>(std::round((-forward_range - msg->angle_min) /
                                    msg->angle_increment)),
        0, size);
    int forward_end = std::clamp(
        static_cast<int>(std::round((forward_range - msg->angle_min) /
                                    msg->angle_increment)),
        0, size);

    auto min_foward_distance = *std::min_element(
        msg->ranges.begin() + forward_start, msg->ranges.begin() + forward_end);
    if (min_foward_distance > obstacle_) {
      set_robot_state(MOVING);
    } else {
      RCLCPP_INFO(get_logger(), "Obstacle detected (%.3f m < %.3f m)",
                  min_foward_distance, obstacle_);
      set_robot_state(TURNING);
    }
  }

  void callback(const Odometry::SharedPtr msg) {
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
    if (std::abs(yaw_diff) > std::abs(degrees_)) {
      RCLCPP_INFO(get_logger(), "Turn completed: rotated %.3f radians",
                  yaw_diff);
      set_robot_state(STOPPED);
      service_called_ = true;
    }
  }

  void callback() {
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

    if (service_called_) {
      send_service_request();
      service_called_ = false;
    }
  }

  double norm_angle(double angle) const {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  void set_robot_state(RobotState rs) {
    std::lock_guard<std::mutex> lck(mutex_);
    if (rs != rs_) {
      RCLCPP_INFO(get_logger(), "Robot state changed");
    }
    rs_ = rs;
  }
  RobotState get_robot_state() {
    std::lock_guard<std::mutex> lck(mutex_);
    return rs_;
  }

  void send_service_request() {
    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf = get_parameter("final_approach").as_bool();
    RCLCPP_INFO(get_logger(), "Service Request");
    auto fut = client_->async_send_request(
        request, [this](rclcpp::Client<GoToLoading>::SharedFuture result) {
          auto response = result.get();
          RCLCPP_INFO(get_logger(), "Service Response: %s",
                      response->complete ? "true" : "false");
        });
  }

  std::mutex mutex_;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;
  rclcpp::CallbackGroup::SharedPtr mutually_exclusive_group_;
  rclcpp::Client<GoToLoading>::SharedPtr client_;

  RobotState rs_;
  double init_yaw_;
  double obstacle_;
  double degrees_;
  bool service_called_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproach>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}