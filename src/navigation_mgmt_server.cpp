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
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include <chrono>
#include "geometry_msgs/msg/twist.hpp"

/***
 *  navigation2 service server
 */

using NavigationServer = robot_interfaces::srv::NavigationServer;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
using namespace std::placeholders;
using namespace rclcpp;
using namespace std::chrono_literals;

class NavigationMgmtServer : public rclcpp::Node
{
public:
    NavigationMgmtServer() : Node("navigation_mgmt_server")//, executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    {
        nav_state_.state = -1;
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        relocalize_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        nav_state_publisher_ = create_publisher<robot_interfaces::msg::NavigationState>("/nav_state", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&NavigationMgmtServer::NavigationStateCallback, this));  
             
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
    int last_goal_index_before_paused_ = 1;
    std::vector<geometry_msgs::msg::PoseStamped> current_waypoints_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr relocalize_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void handle_service_request(
        const std::shared_ptr<NavigationServer::Request> request,
        std::shared_ptr<NavigationServer::Response> response)
    {
        // active_response_ = response;

        switch (request->cmd_name) {
            case 1:  // Send poses list 并开始导航
                try{
                    send_waypoints(request->poses_list, false);
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
                response->message = "Navigation pause command received";
                break;
            case 3:  // Resume
                try {
                    send_waypoints(request->poses_list, true);
                } catch (const std::exception& e) {
                    response->result = false;
                    response->message = std::string("Error resuming navigation: ") + e.what();
                    RCLCPP_ERROR(get_logger(), "resume_navigation error: %s", e.what());
                    break;
                }
                response->result = true;
                response->message = "Resume navigation command received";
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
                    relocalize_robot(request->pose_estimate);
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

    void send_waypoints(const std::vector<geometry_msgs::msg::Pose>& poses, bool if_resume)
    {
        auto goal_msg = FollowWaypoints::Goal();
        if(!if_resume){
            //重置变量
            is_paused_ = false;
            last_goal_index_before_paused_ = 1;

            if(nav_state_.state != -1){
                throw std::runtime_error("Currently in navigation or navigation paused, need to cancel the current navigation");
            }

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

            goal_msg.poses = current_waypoints_;
        }else{
            //resume navigation
            if (!is_paused_ || current_waypoints_.empty() || nav_state_.state == -1 || nav_state_.state == 1) {
                //debug
                // RCLCPP_ERROR(get_logger(), "relocalize_robot error: %s", is_paused_);
                throw std::runtime_error("No paused navigation to resume");
            }
            auto remaining_waypoints = std::vector<geometry_msgs::msg::PoseStamped>(
                current_waypoints_.begin() + nav_state_.current_goal_index - 1,
                current_waypoints_.end());
                
            goal_msg.poses = remaining_waypoints;
        }
        

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        // send_goal_options.goal_response_callback =
        //     std::bind(&NavigationMgmtServer::goal_response_callback, this, _1);
        // send_goal_options.feedback_callback =
        //     std::bind(&NavigationMgmtServer::feedback_callback, this, _1, _2);
        // send_goal_options.result_callback =
        //     std::bind(&NavigationMgmtServer::result_callback, this, _1);
        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowWaypoints::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_INFO(get_logger(), "Goal was rejected by server...");
                    nav_state_.state = -1;
                    throw std::runtime_error("Goal was rejected by server...");
                } else {
                    this->goal_handle_ = goal_handle;
                    RCLCPP_INFO(get_logger(), "Goal accepted by server, processing...");
                    nav_state_.state = 1;
                    nav_state_.goals_num = current_waypoints_.size();
                    nav_state_.current_goal_index = 1;
                }
            };
        send_goal_options.feedback_callback =
            [this](GoalHandleFollowWaypoints::SharedPtr,
                const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
                    RCLCPP_INFO(get_logger(), "Reached waypoint %d of %d",
                    feedback->current_waypoint + last_goal_index_before_paused_, nav_state_.goals_num);
                    nav_state_.state = 1;
                    nav_state_.current_goal_index = last_goal_index_before_paused_ + feedback->current_waypoint;
            };
        send_goal_options.result_callback =
            [this](const GoalHandleFollowWaypoints::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "Navigation all points succeeded");
                        nav_state_.state = -1;
                        nav_state_.current_goal_index = 0;
                        nav_state_.goals_num = 0;
                        is_paused_ = false;
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        // nav_state_.state = -1;
                        // nav_state_.current_goal_index = 0;
                        // nav_state_.goals_num = 0;
                        RCLCPP_INFO(get_logger(), "Navigation ABORTED");
                        break;
                    case rclcpp_action::ResultCode::CANCELED: 
                        if(!is_paused_){
                            nav_state_.state = -1;
                            nav_state_.current_goal_index = 0;
                            nav_state_.goals_num = 0;
                        }else{
                            nav_state_.state = 0;
                            last_goal_index_before_paused_ = nav_state_.current_goal_index;
                        }
                        RCLCPP_INFO(get_logger(), "Navigation all points canceled");
                        break;
                    default:
                        nav_state_.state = -1;
                        break;
                }
                
                //navigation2在运动结束时，需要将/cmd_vel置0
                geometry_msgs::msg::Twist msg;
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                for(int ii=0; ii < 20; ii++){
                    cmd_vel_pub_->publish(msg);
                    sleep(0.01);
                }

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
        //注意这部分执行完才会执行send_goal_options.result_callback
        if(nav_state_.state == -1 || nav_state_.state == 0){
            throw std::runtime_error("Currently navigation PAUSED or NOT in navigation, Need to in navigation firstly");
        }

        if (!goal_handle_) {
            throw std::runtime_error("No active navigation to pause");
        }
        
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        is_paused_ = true;
        nav_state_.state = 0;
    }

    void cancel_navigation()
    {
        //注意这部分执行完才会执行send_goal_options.result_callback
        if(nav_state_.state == -1){
            throw std::runtime_error("Currently NOT in navigation, cannot be cancel");
        }

        if(is_paused_){
            nav_state_.state = -1;
            nav_state_.current_goal_index = 0;
            nav_state_.goals_num = 0;
        }else{
            if (!goal_handle_) {
                throw std::runtime_error("No active navigation to cancel");
            }
            
            auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        }
        last_goal_index_before_paused_ = 1;
        is_paused_ = false;
        current_waypoints_.clear();
        goal_handle_.reset();
    }

    void NavigationStateCallback(){
        nav_state_.header.stamp = this->get_clock()->now(); 
        nav_state_publisher_->publish(nav_state_);
    }

    void relocalize_robot(const geometry_msgs::msg::Pose& pose_estimate)
    {
        if(pose_estimate.orientation.z == 0.0 && pose_estimate.orientation.w == 0.0){
            throw std::invalid_argument("Pose estimate message is fault or can be empty");
        }
        //设置重定位点，只需要向topic /initialpose发布pose
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map"; // 通常使用 map 坐标系
        pose_msg.pose.pose = pose_estimate;
        relocalize_publisher_->publish(pose_msg);

        RCLCPP_INFO(this->get_logger(), "Successfully set initial pose");
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