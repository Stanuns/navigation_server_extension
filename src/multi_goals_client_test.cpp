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
            geometry_msgs::msg::Pose pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8, pose9, pose10;
            
            // Waypoint 1
            pose1.position.x = 0.25487;
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
            pose3.position.x = -0.25;
            pose3.position.y = -0.42;
            pose3.position.z = 0.0;
            pose3.orientation.z = -0.85467;
            pose3.orientation.w = 0.51918;

            // Waypoint 4
            pose4.position.x = -0.31;
            pose4.position.y = -0.73;
            pose4.position.z = 0.0;
            pose4.orientation.z = 0.988871;
            pose4.orientation.w = -0.148777;

            // Waypoint 5
            pose5.position.x = 0.20;
            pose5.position.y = 0.313;
            pose5.position.z = 0.0;
            pose5.orientation.z = 0.520416;
            pose5.orientation.w = 0.853913;

            // Waypoint 6
            pose6.position.x = 0.42537;
            pose6.position.y = 0.78219;
            pose6.position.z = 0.0;
            pose6.orientation.z = 0.456265;
            pose6.orientation.w = 0.889844;

            // Waypoint 7
            pose7.position.x = -0.37;
            pose7.position.y = -0.87;
            pose7.position.z = 0.0;
            pose7.orientation.z = -0.838048;
            pose7.orientation.w = 0.545596;

            // Waypoint 8
            pose8.position.x = 0.057;
            pose8.position.y = -0.09;
            pose8.position.z = 0.0;
            pose8.orientation.z = -0.85467;
            pose8.orientation.w = 0.51918;

            // Waypoint 9
            pose9.position.x = 0.20;
            pose9.position.y = 0.313;
            pose9.position.z = 0.0;
            pose9.orientation.z = 0.520416;
            pose9.orientation.w = 0.853913;

            // Waypoint 10
            pose10.position.x = -0.31;
            pose10.position.y = -0.73;
            pose10.position.z = 0.0;
            pose10.orientation.z = 0.988871;
            pose10.orientation.w = -0.148777;
            
            request->poses_list = {pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8, pose9, pose10};
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