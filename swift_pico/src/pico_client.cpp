#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "waypoint_navigation/action/nav_to_waypoint.hpp"
#include "waypoint_navigation/srv/get_waypoints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

/**
 * Waypoint Client Node
 * 
 * Action client that requests waypoints from service and navigates to each one.
 * 
 * Service Client: /waypoints (waypoint_navigation::srv::GetWaypoints)
 * Action Client: /waypoint_navigation (waypoint_navigation::action::NavToWaypoint)
 */
class WayPointClient : public rclcpp::Node
{
public:
    using NavToWaypoint = waypoint_navigation::action::NavToWaypoint;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavToWaypoint>;

    WayPointClient() : Node("waypoint_client")
    {
        goals_.clear();
        goal_index_ = 0;
        
        // Create service client
        service_client_ = this->create_client<waypoint_navigation::srv::GetWaypoints>("waypoints");
        
        // Create action client
        action_client_ = rclcpp_action::create_client<NavToWaypoint>(this, "waypoint_navigation");
        
        RCLCPP_INFO(this->get_logger(), "Waypoint client started");
    }
    
    // Request waypoints from service
    bool receive_goals()
    {
        // Wait for service to be available
        while (!service_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        
        // Create request (empty for GetWaypoints)
        auto request = std::make_shared<waypoint_navigation::srv::GetWaypoints::Request>();
        
        // Send request
        auto future = service_client_->async_send_request(request);
        
        // Wait for response
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            
            // Extract waypoints from response arrays
            size_t num_waypoints = response->x.size();
            if (num_waypoints != response->y.size() || num_waypoints != response->z.size()) {
                RCLCPP_ERROR(this->get_logger(), "Waypoint arrays have different sizes!");
                return false;
            }
            
            goals_.clear();
            for (size_t i = 0; i < num_waypoints; ++i) {
                std::vector<double> waypoint = {response->x[i], response->y[i], response->z[i]};
                goals_.push_back(waypoint);
                RCLCPP_INFO(this->get_logger(), "Waypoint %zu: [%.2f, %.2f, %.2f]",
                           i, waypoint[0], waypoint[1], waypoint[2]);
            }
            
            RCLCPP_INFO(this->get_logger(), "Received %zu waypoints from service", num_waypoints);
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
            return false;
        }
    }
    
    // Send goal to action server
    void send_goal(const std::vector<double>& waypoint)
    {
        // Wait for action server to be available
        if (!action_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }
        
        // Create goal message
        auto goal_msg = NavToWaypoint::Goal();
        goal_msg.x = waypoint[0];
        goal_msg.y = waypoint[1];
        goal_msg.z = waypoint[2];
        
        RCLCPP_INFO(this->get_logger(), "Sending goal %zu: [%.2f, %.2f, %.2f]",
                   goal_index_, goal_msg.x, goal_msg.y, goal_msg.z);
        
        // Set up goal response callback
        auto send_goal_options = rclcpp_action::Client<NavToWaypoint>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WayPointClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&WayPointClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&WayPointClient::result_callback, this, _1);
        
        // Send goal
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    // Process all waypoints sequentially
    void process_waypoints()
    {
        if (goals_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No waypoints to process");
            return;
        }
        
        goal_index_ = 0;
        send_goal(goals_[0]);
        
        // Spin to process callbacks
        rclcpp::spin_some(this->shared_from_this());
    }

private:
    // Action client callbacks
    void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
    }
    
    void feedback_callback(
        GoalHandleNav::SharedPtr,
        const std::shared_ptr<const NavToWaypoint::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
                   "Received feedback! Current position: [%.2f, %.2f, %.2f]",
                   feedback->current_x, feedback->current_y, feedback->current_z);
    }
    
    void result_callback(const GoalHandleNav::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), 
                           "Goal %zu succeeded! Stabilization time: %.2f seconds",
                           goal_index_, result.result->stabilization_time);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal %zu was aborted", goal_index_);
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal %zu was canceled", goal_index_);
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        
        // Move to next waypoint
        goal_index_++;
        
        if (goal_index_ < goals_.size()) {
            // Send next goal
            std::this_thread::sleep_for(500ms); // Small delay between waypoints
            send_goal(goals_[goal_index_]);
        } else {
            // All waypoints completed
            RCLCPP_INFO(this->get_logger(), "All %zu waypoints have been reached successfully!",
                       goals_.size());
            rclcpp::shutdown();
        }
    }
    
    // Member variables
    std::vector<std::vector<double>> goals_;
    size_t goal_index_;
    
    rclcpp::Client<waypoint_navigation::srv::GetWaypoints>::SharedPtr service_client_;
    rclcpp_action::Client<NavToWaypoint>::SharedPtr action_client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto waypoint_client = std::make_shared<WayPointClient>();
    
    // Request waypoints from service
    if (!waypoint_client->receive_goals()) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_client"), 
                    "Failed to receive waypoints. Exiting.");
        rclcpp::shutdown();
        return 1;
    }
    
    // Process waypoints
    waypoint_client->process_waypoints();
    
    // Spin until shutdown
    try {
        rclcpp::spin(waypoint_client);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_client"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
