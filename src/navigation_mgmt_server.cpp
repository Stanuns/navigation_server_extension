#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "robot_interfaces/srv/navigation_server.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <vector>

/***
 *  navigation2 service server
 */

using NavigationServer = robot_interfaces::srv::NavigationServer;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class NavigationMgmtServer : public rclcpp::Node
{
public:
    NavigationMgmtServer() : Node("navigation_mgmt_server")
    {
        // Initialize action client for FollowWaypoints
        action_client_ = rclcpp_action::create_client<FollowWaypoints>(
            this, "follow_waypoints");

        // Create service server
        service_ = create_service<NavigationServer>(
            "navigation_server",
            std::bind(&NavigationMgmtServer::handle_service_request, this,
                     std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
    rclcpp::Service<NavigationServer>::SharedPtr service_;
    GoalHandleFollowWaypoints::SharedPtr goal_handle_;
    // std::shared_ptr<NavigationServer::Response> active_response_;

    void handle_service_request(
        const std::shared_ptr<NavigationServer::Request> request,
        std::shared_ptr<NavigationServer::Response> response)
    {
        // active_response_ = response;

        switch (request->cmd_name) {
            case 1:  // Send poses list 并开始导航
                send_waypoints(request->poses_list, response);
                
                RCLCPP_INFO(get_logger(), "send_waypoints......");
                break;
            case 2:  // Pause
                // Implement pause logic if needed
                response->result = true;
                response->message = "Pause command received";
                break;
            case 3:  // Resume
                // Implement resume logic if needed
                response->result = true;
                response->message = "Resume command received";
                break;
            case 4:  // Cancel current list
                cancel_navigation(response);
                break;
            case 5:  // Manual re-localization
                // Implement re-localization logic
                response->result = true;
                response->message = "Re-localization command received";
                break;
            default:
                response->result = false;
                response->message = "Invalid command";
                break;
        }
    }

    void send_waypoints(
        const std::vector<geometry_msgs::msg::Pose>& poses,
        std::shared_ptr<NavigationServer::Response> response)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            response->result = false;
            response->message = "Action server not available";
            return;
        }

        auto goal_msg = FollowWaypoints::Goal();
        for (const auto& pose : poses) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose = pose;
            pose_stamped.header.frame_id = "map";  
            pose_stamped.header.stamp = now();
            goal_msg.poses.push_back(pose_stamped);
        }

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this, response](const GoalHandleFollowWaypoints::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    response->result = false;
                    response->message = "Goal was rejected by server";

                    RCLCPP_INFO(get_logger(), "goal_response_callback...goal_handle is false...");
                } else {
                    this->goal_handle_ = goal_handle;
                    response->result = true;
                    response->message = "Goal accepted by server, processing...";

                    RCLCPP_INFO(get_logger(), "goal_response_callback......");
                }
            };

            send_goal_options.feedback_callback =
                [this](GoalHandleFollowWaypoints::SharedPtr,
                    const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
                    // RCLCPP_INFO(get_logger(), "Current waypoint: %d",
                    //         feedback->current_waypoint);
                    // if (active_response_) {
                    //     active_response_->message = "Current waypoint: " + 
                    //                             std::to_string(feedback->current_waypoint);
                    // }
                };

        send_goal_options.result_callback =
            [this](const GoalHandleFollowWaypoints::WrappedResult& result) {
                // if (!active_response_) return;
                
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        // active_response_->result = true;
                        // active_response_->message = "Navigation succeeded";
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        // active_response_->result = false;
                        // active_response_->message = "Navigation aborted";
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        // active_response_->result = false;
                        // active_response_->message = "Navigation canceled";
                        break;
                    default:
                        // active_response_->result = false;
                        // active_response_->message = "Unknown result";
                        break;
                }
                this->goal_handle_.reset();
            };

        auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void cancel_navigation(std::shared_ptr<NavigationServer::Response> response)
    {
        if (!goal_handle_) {
            response->result = false;
            response->message = "No active navigation to cancel";
            return;
        }

        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        if (rclcpp::spin_until_future_complete(
                shared_from_this(), future_cancel) != rclcpp::FutureReturnCode::SUCCESS) {
            response->result = false;
            response->message = "Failed to cancel navigation";
            return;
        }

        goal_handle_.reset();
        response->result = true;
        response->message = "Navigation canceled successfully";
        
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationMgmtServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}