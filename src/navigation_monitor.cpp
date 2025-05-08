#include <chrono>
#include <memory>
#include <string>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class NavigationLifecycleMonitor : public rclcpp::Node {
public:
  NavigationLifecycleMonitor() : Node("navigation_lifecycle_monitor") {

    publisher_ = this->create_publisher<std_msgs::msg::String>("navigation_status", 10);

    timer_ = create_wall_timer(
      500ms, std::bind(&NavigationLifecycleMonitor::check_states, this));
  }

private:
  void check_states() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start check states....");
    
    std::shared_ptr<rclcpp::Node> controller_node = rclcpp::Node::make_shared("node_controller_client_");
    controller_client_ = controller_node->create_client<lifecycle_msgs::srv::GetState>(
      "/controller_server/get_state");

    std::shared_ptr<rclcpp::Node> planner_node = rclcpp::Node::make_shared("node_planner_client_");
    planner_client_ = planner_node->create_client<lifecycle_msgs::srv::GetState>(
      "/planner_server/get_state");

    std::shared_ptr<rclcpp::Node> bt_navigator_node = rclcpp::Node::make_shared("node_bt_navigator_client_");
    bt_navigator_client_ = bt_navigator_node->create_client<lifecycle_msgs::srv::GetState>(
      "/bt_navigator/get_state");

    while (!controller_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "controller_client_ service not available, waiting again...");
    }
    while (!planner_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "planner_client_ service not available, waiting again...");
    }
    while (!bt_navigator_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "bt_navigator_client_ service not available, waiting again...");
    }

    auto controller_future = controller_client_->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    auto planner_future = planner_client_->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    auto bt_navigator_future = bt_navigator_client_->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>());

    //注意报错：Node has already been added to an executor.
    if (rclcpp::spin_until_future_complete(
          controller_node,
          controller_future, 1s) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "Failed to get controller_server state");
      return;
    }
    if (rclcpp::spin_until_future_complete(
          planner_node,
          planner_future, 1s) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "Failed to get planner_server state");
      return;
    }
    if (rclcpp::spin_until_future_complete(
          bt_navigator_node,
          bt_navigator_future, 1s) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "Failed to get bt_navigator state");
      return;
    }

    const uint8_t active_state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    bool all_active = true;

    all_active &= (controller_future.get()->current_state.id == active_state);
    all_active &= (planner_future.get()->current_state.id == active_state);
    all_active &= (bt_navigator_future.get()->current_state.id == active_state);

    auto msg = std_msgs::msg::String();
    if (all_active) {
      msg.data = "All navigation nodes are ACTIVE!";
      RCLCPP_INFO(get_logger(), msg.data.c_str());
    } else {
      msg.data = "Navigation nodes NOT ready";
      RCLCPP_WARN(get_logger(), msg.data.c_str());
    }
    publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr controller_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr planner_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr bt_navigator_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationLifecycleMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}