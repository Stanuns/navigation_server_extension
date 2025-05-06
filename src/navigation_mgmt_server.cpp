#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "robot_interfaces/srv/navigation_server.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <vector>
#include <rclcpp/executors.hpp>

/***
 *  navigation2 service server
 */

using NavigationServer = robot_interfaces::srv::NavigationServer;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
using namespace std::placeholders;

class NavigationMgmtServer : public rclcpp::Node
{
public:
    NavigationMgmtServer() : Node("navigation_mgmt_server")//, executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
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
    // rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    bool navigation_canceled_ = false;
    // std::mutex navigation_cancel_mutex_;
    // std::condition_variable navigation_cancel_cv_;

    void handle_service_request(
        const std::shared_ptr<NavigationServer::Request> request,
        std::shared_ptr<NavigationServer::Response> response)
    {
        // active_response_ = response;

        switch (request->cmd_name) {
            case 1:  // Send poses list 并开始导航
                try{
                    send_waypoints(request->poses_list, response);
                }catch (const std::exception& e) {
                    response->result = false;
                    response->message = std::string("Error in send_waypoints: ") + e.what();
                    RCLCPP_ERROR(get_logger(), "send_waypoints error: %s", e.what());
                    break;
                } catch (...) {
                    response->result = false;
                    response->message = "Unknown error in send_waypoints";
                    RCLCPP_ERROR(get_logger(), "Unknown error in send_waypoints");
                    break;
                }
                response->result = true;
                response->message = std::string("Send navigation2 waypoints successfully");
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
                try{
                    cancel_navigation(response);
                }catch (const std::exception& e) {
                    response->result = false;
                    response->message = std::string("Error in cancel current list: ") + e.what();
                    RCLCPP_ERROR(get_logger(), "Cancel current list error: %s", e.what());
                    break;
                } catch (...) {
                    response->result = false;
                    response->message = "Unknown error in cancel current list";
                    RCLCPP_ERROR(get_logger(), "Unknown error in cancel current list");
                    break;
                }
                response->result = true;
                response->message = "Navigation canceled successfully";
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
            throw std::runtime_error("Action server not available");
        }

        auto goal_msg = FollowWaypoints::Goal();
        if (poses.empty()) {
            throw std::runtime_error("Empty poses list provided");
        }
        for (const auto& pose : poses) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose = pose;
            pose_stamped.header.frame_id = "map";  
            pose_stamped.header.stamp = now();
            goal_msg.poses.push_back(pose_stamped);
        }

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&NavigationMgmtServer::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&NavigationMgmtServer::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&NavigationMgmtServer::result_callback, this, _1);

        auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_INFO(get_logger(), "Goal was rejected by server...");
            throw std::runtime_error("Goal was rejected by server...");
        } else {
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(get_logger(), "Goal accepted by server, processing...");
        }
    }

    void feedback_callback(GoalHandleFollowWaypoints::SharedPtr,
                const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "Current waypoint: %d",
                feedback->current_waypoint);
        // if (active_response_) {
        //     active_response_->message = "Current waypoint: " + 
        //                             std::to_string(feedback->current_waypoint);
        // }
    }

    void result_callback(const GoalHandleFollowWaypoints::WrappedResult& result) {
        // if (!active_response_) return;
        // std::lock_guard<std::mutex> lock(navigation_cancel_mutex_);
            
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // active_response_->result = true;
                // active_response_->message = "Navigation succeeded";
                RCLCPP_INFO(get_logger(), "Navigation all points succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                // active_response_->result = false;
                // active_response_->message = "Navigation aborted";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                // active_response_->result = false;
                // active_response_->message = "Navigation canceled";
                navigation_canceled_ = true;
                // navigation_cancel_cv_.notify_one();
                RCLCPP_INFO(get_logger(), "Navigation all points canceled");
                break;
            default:
                // active_response_->result = false;
                // active_response_->message = "Unknown result";
                break;
        }
        this->goal_handle_.reset();
    }

    void cancel_navigation(std::shared_ptr<NavigationServer::Response> response)
    {
        if (!goal_handle_) {
            throw std::runtime_error("No active navigation to cancel");
        }

        navigation_canceled_ = false;
        

        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);

        
        goal_handle_.reset();
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