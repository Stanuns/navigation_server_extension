#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "robot_interfaces/srv/navigation_server.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <vector>
#include <rclcpp/executors.hpp>
#include "robot_interfaces/msg/navigation_state.hpp"

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
        nav_state_publisher_ = create_publisher<robot_interfaces::msg::NavigationState>("/nav_state", 10);
        timer_ = this->create_wall_timer(
            200ms, std::bind(&NavigationMgmtServer::NavigationStateCallback, this));  
             
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
    // bool navigation_canceled_ = false;
    // std::mutex navigation_cancel_mutex_;
    // std::condition_variable navigation_cancel_cv_;
    rclcpp::TimerBase::SharedPtr timer_;
    robot_interfaces::msg::NavigationState nav_state_;
    Publisher<robot_interfaces::msg::NavigationState>::SharedPtr nav_state_publisher_;
    bool is_paused_ = false;
    std::vector<geometry_msgs::msg::PoseStamped> current_waypoints_;

    void handle_service_request(
        const std::shared_ptr<NavigationServer::Request> request,
        std::shared_ptr<NavigationServer::Response> response)
    {
        // active_response_ = response;

        switch (request->cmd_name) {
            case 1:  // Send poses list 并开始导航
                try{
                    send_waypoints(request->poses_list);
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
                try {
                    pause_navigation();
                } catch (const std::exception& e) {
                    response->result = false;
                    response->message = std::string("Error pausing navigation: ") + e.what();
                    RCLCPP_ERROR(get_logger(), "pause_navigation error: %s", e.what());
                    break;
                }
                response->result = true;
                response->message = "Pause command received";
                break;
            case 3:  // Resume
                try {
                    resume_navigation();
                } catch (const std::exception& e) {
                    response->result = false;
                    response->message = std::string("Error resuming navigation: ") + e.what();
                    RCLCPP_ERROR(get_logger(), "resume_navigation error: %s", e.what());
                    break;
                }
                response->result = true;
                response->message = "Resume command received";
                break;
            case 4:  // Cancel current list
                try{
                    cancel_navigation();
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
                try {
                    relocalize_robot(request->pose_estimate, response);
                } catch (const std::exception& e) {
                    response->result = false;
                    response->message = std::string("Error in re-localization: ") + e.what();
                    RCLCPP_ERROR(get_logger(), "relocalize_robot error: %s", e.what());
                    break;
                }
                response->result = true;
                response->message = "Re-localization completed successfully";
                break;
            default:
                response->result = false;
                response->message = "Invalid command";
                break;
        }
    }

    void send_waypoints(const std::vector<geometry_msgs::msg::Pose>& poses)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            throw std::runtime_error("Action server not available");
        }

        current_waypoints_.clear();
        for (const auto& pose : poses) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose = pose;
            pose_stamped.header.frame_id = "map";  
            pose_stamped.header.stamp = now();
            current_waypoints_.push_back(pose_stamped);
        }

        if (current_waypoints_.empty()) {
            throw std::runtime_error("Empty poses list provided");
        }

        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses = current_waypoints_;

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        // send_goal_options.goal_response_callback =
        //     std::bind(&NavigationMgmtServer::goal_response_callback, this, _1);
        // send_goal_options.feedback_callback =
        //     std::bind(&NavigationMgmtServer::feedback_callback, this, _1, _2);
        // send_goal_options.result_callback =
        //     std::bind(&NavigationMgmtServer::result_callback, this, _1);
        send_goal_options.goal_response_callback =
            [this, response](const GoalHandleFollowWaypoints::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_INFO(get_logger(), "Goal was rejected by server...");
                    nav_state_.state = -1;
                    throw std::runtime_error("Goal was rejected by server...");
                } else {
                    this->goal_handle_ = goal_handle;
                    RCLCPP_INFO(get_logger(), "Goal accepted by server, processing...");
                    nav_state_.state = 1;
                    nav_state_.goals_num = current_waypoints_.poses.size();
                    nav_state_.current_goal_index = 1;
                }
            };
        send_goal_options.feedback_callback =
            [this](GoalHandleFollowWaypoints::SharedPtr,
                const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
                    RCLCPP_INFO(get_logger(), "Reached waypoint %d of %d",
                    feedback->current_waypoint + 1, nav_state_.goals_num);
                    nav_state_.state = 1;
                    nav_state_.current_goal_index = feedback->current_waypoint + 1;
            };
        send_goal_options.result_callback =
            [this](const GoalHandleFollowWaypoints::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "Navigation all points succeeded");
                        nav_state_.state = -1;
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        nav_state_.state = -1;
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        
                        nav_state_.state = -1;
                        RCLCPP_INFO(get_logger(), "Navigation all points canceled");
                        break;
                    default:
                        nav_state_.state = -1;
                        break;
                }
                nav_state_.current_goal_index = 0;
                nav_state_.goals_num = 0;
                this->goal_handle_.reset();
            };

        auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr& goal_handle) {
    //     if (!goal_handle) {
    //         RCLCPP_INFO(get_logger(), "Goal was rejected by server...");
    //         throw std::runtime_error("Goal was rejected by server...");
    //     } else {
    //         this->goal_handle_ = goal_handle;
    //         RCLCPP_INFO(get_logger(), "Goal accepted by server, processing...");
    //     }
    // }

    // void feedback_callback(GoalHandleFollowWaypoints::SharedPtr,
    //             const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
    //     RCLCPP_INFO(get_logger(), "Current waypoint: %d",
    //             feedback->current_waypoint);
    //     // if (active_response_) {
    //     //     active_response_->message = "Current waypoint: " + 
    //     //                             std::to_string(feedback->current_waypoint);
    //     // }
    //     nav_state_.state = 1;
    //     nav_state_.current_goal_index = feedback->current_waypoint + 1;
    // }

    // void result_callback(const GoalHandleFollowWaypoints::WrappedResult& result) {
    //     // if (!active_response_) return;
    //     // std::lock_guard<std::mutex> lock(navigation_cancel_mutex_);
            
    //     switch (result.code) {
    //         case rclcpp_action::ResultCode::SUCCEEDED:
    //             // active_response_->result = true;
    //             // active_response_->message = "Navigation succeeded";
    //             RCLCPP_INFO(get_logger(), "Navigation all points succeeded");
    //             break;
    //         case rclcpp_action::ResultCode::ABORTED:
    //             // active_response_->result = false;
    //             // active_response_->message = "Navigation aborted";
    //             break;
    //         case rclcpp_action::ResultCode::CANCELED:
    //             // active_response_->result = false;
    //             // active_response_->message = "Navigation canceled";
    //             navigation_canceled_ = true;
    //             // navigation_cancel_cv_.notify_one();
    //             RCLCPP_INFO(get_logger(), "Navigation all points canceled");
    //             break;
    //         default:
    //             // active_response_->result = false;
    //             // active_response_->message = "Unknown result";
    //             break;
    //     }
    //     this->goal_handle_.reset();
    // }


    void pause_navigation() {
        if (!goal_handle_) {
            throw std::runtime_error("No active navigation to pause");
        }
        
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        is_paused_ = true;
        nav_state_.state = 0;
    }

    void resume_navigation() {
        if (!is_paused_ || current_waypoints_.empty()) {
            throw std::runtime_error("No paused navigation to resume");
        }
        
        // Create new goal with remaining waypoints
        auto remaining_waypoints = std::vector<geometry_msgs::msg::PoseStamped>(
            current_waypoints_.begin() + nav_state_.current_goal_index - 1,
            current_waypoints_.end());
            
        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses = remaining_waypoints;

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this, response](const GoalHandleFollowWaypoints::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    nav_state_.state = -1;
                    throw std::runtime_error("Goal was rejected by server...");
                } else {
                    this->goal_handle_ = goal_handle;
                    nav_state_.state = 1;
                }
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleFollowWaypoints::SharedPtr,
                const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
                    nav_state_.current_goal_index = feedback->current_waypoint + 1;
            };

        send_goal_options.result_callback =
            [this](const GoalHandleFollowWaypoints::WrappedResult& result) {
                nav_state_.state = -1;
                nav_state_.current_goal_index = 0;
                this->goal_handle_.reset();
                is_paused_ = false;
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void cancel_navigation()
    {
        if (!goal_handle_) {
            throw std::runtime_error("No active navigation to cancel");
        }

        // navigation_canceled_ = false;
        
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);

        goal_handle_.reset();
        is_paused_ = false;
    }

    void NavigationStateCallback(){
        nav_state_.header.stamp = this->get_clock()->now(); 
        nav_state_publisher_->publish(nav_state_);
    }

    void relocalize_robot(const geometry_msgs::msg::Pose& pose_estimate)
    {
        // Create a service client for the re-localization service
        auto client = this->create_client<std_srvs::srv::Trigger>("reinitialize_global_localization");
        
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            throw std::runtime_error("Re-localization service not available");
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);

        // Wait for the result (with timeout)
        if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future, std::chrono::seconds(5)) 
        {
            auto result = future.get();
            if (!result->success) {
                throw std::runtime_error("Re-localization service failed");
            }
        } else {
            throw std::runtime_error("Re-localization service timed out");
        }

        // Optionally set the initial pose (if your system supports it)
        // You might need to create another service client for this
        RCLCPP_INFO(get_logger(), "Re-localization completed successfully");
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