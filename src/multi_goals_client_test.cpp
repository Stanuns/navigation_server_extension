#include <rclcpp/rclcpp.hpp>
#include "robot_interfaces/srv/navigation_server.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MultiGoalsNavigationClient : public rclcpp::Node
{
public:
    MultiGoalsNavigationClient() : Node("multi_goals_client_test")
    {
        client_ = create_client<robot_interfaces::srv::NavigationServer>("navigation_server");
        
        // Wait for service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
        }
        
        send_request(1);  // Send command 1 with waypoints
    }

private:
    rclcpp::Client<robot_interfaces::srv::NavigationServer>::SharedPtr client_;

    void send_request(uint32_t cmd)
    {
        auto request = std::make_shared<robot_interfaces::srv::NavigationServer::Request>();
        request->cmd_name = cmd;
        
        if (cmd == 1) {
            // Create three sample waypoints
            geometry_msgs::msg::Pose pose1, pose2, pose3;
            
            // Waypoint 1
            pose1.position.x = 0.10487;
            pose1.position.y = 0.36428;
            pose1.position.z = 0.0;
            pose1.orientation.z = -0.85467;
            pose1.orientation.w = 0.51918;
            
            // Waypoint 2
            pose2.position.x = 0.057;
            pose2.position.y = -0.09;
            pose2.position.z = 0.0;
            pose2.orientation.z = -0.85467;
            pose2.orientation.w = 0.51918;
            
            // Waypoint 3
            pose3.position.x = -0.10;
            pose3.position.y = -0.42;
            pose3.position.z = 0.0;
            pose3.orientation.z = -0.85467;
            pose3.orientation.w = 0.51918;
            
            request->poses_list = {pose1, pose2, pose3};
        }

        auto result_future = client_->async_send_request(request);
        
        // Wait for the result
        if (rclcpp::spin_until_future_complete(
                this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(get_logger(), "Service call result: %s", result->message.c_str());
            RCLCPP_INFO(get_logger(), "Success: %s", result->result ? "true" : "false");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to call service");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiGoalsNavigationClient>();
    rclcpp::shutdown();
    return 0;
}