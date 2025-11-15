#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <thread>
#include <mutex>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "swift_msgs/msg/swift_msgs.hpp"
#include "error_msg/msg/error.hpp"
#include "waypoint_navigation/action/nav_to_waypoint.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

/**
 * Waypoint Server Node
 * 
 * Action server that navigates the drone to waypoints using PID control.
 * Uses the same PID logic as pico_controller_PID.cpp but with dynamic setpoints.
 * 
 * Action: /waypoint_navigation (waypoint_navigation::action::NavToWaypoint)
 * Subscribes: /whycon/poses, /rotors/odometry
 * Publishes: /drone_command, /position_error
 * 
 * Coordinate System:
 * - All coordinates are uniform: WhyCon coordinates (decimeters)
 * - Waypoints from action goals are in WhyCon coordinates (decimeters)
 * - WhyCon provides position in decimeters
 * - No coordinate conversion needed
 */
class WayPointServer : public rclcpp::Node
{
public:
    using NavToWaypoint = waypoint_navigation::action::NavToWaypoint;
    using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavToWaypoint>;

    WayPointServer() : Node("waypoint_server")
    {
        // Initialize PID controller variables
        drone_position_[0] = 0.0f;
        drone_position_[1] = 0.0f;
        drone_position_[2] = 0.0f;
        
        // Initialize desired_state to invalid value - will be set to current position on first WhyCon update
        // This prevents PID from running before a goal is received
        desired_state_[0] = std::numeric_limits<float>::quiet_NaN();
        desired_state_[1] = std::numeric_limits<float>::quiet_NaN();
        desired_state_[2] = std::numeric_limits<float>::quiet_NaN();
        
        goal_active_ = false;
        position_received_ = false;
        
        // PID gains [roll, pitch, throttle] - using EXACT same values from pico_controller_PID.cpp
        // Kp_[0] = 0.0f;   // Roll (not tuned yet)
        // Kp_[1] = 0.0f;   // Pitch (not tuned yet)
        // Kp_[2] = 60.5f;  // Throttle (tuned from Task 1c)
        
        Kp_[0] = 1.0f;   
        Kp_[1] = 1.0f;   
        Kp_[2] = 10.0f; 

        Ki_[0] = 0.0f;
        Ki_[1] = 0.025f;
        Ki_[2] = 0.108f;
        
        Kd_[0] = 15.0f;
        Kd_[1] = 15.0f;
        Kd_[2] = 200.0f;
        
        // PID state variables
        prev_error_[0] = 0.0f;      
        prev_error_[1] = 0.0f;
        prev_error_[2] = 0.0f;
        
        error_sum_[0] = 0.0f;
        error_sum_[1] = 0.0f;
        error_sum_[2] = 0.0f;
        
        filtered_derivative_[0] = 0.0f;
        filtered_derivative_[1] = 0.0f;
        filtered_derivative_[2] = 0.0f;
        
        // Limits - same as pico_controller_PID.cpp
        max_values_[0] = 2000.0f;
        max_values_[1] = 2000.0f;
        max_values_[2] = 5000.0f;
        
        min_values_[0] = 1000.0f;
        min_values_[1] = 1000.0f;
        min_values_[2] = 1000.0f;
        
        // Initialize command
        cmd_.rc_roll = 1500;
        cmd_.rc_pitch = 1500;
        cmd_.rc_yaw = 1500;
        cmd_.rc_throttle = 1500;
        cmd_.rc_aux4 = 2000;
        
        // Yaw from odometry
        yaw_ = 0.0;
        
        // Sphere detection variables
        time_inside_sphere_ = 0.0;
        max_time_inside_sphere_ = 0.0;
        point_in_sphere_start_time_ = -1.0;
        duration_ = 0.0;
        current_time_ = 0.0;
        
        // Sample time for PID (40ms = 25 Hz) - same as pico_controller_PID.cpp
        sample_time_ = 40ms;
        
        // D-term filter coefficient - same as pico_controller_PID.cpp
        d_filter_alpha_ = 0.6f;
        
        // Create publishers
        command_pub_ = this->create_publisher<swift_msgs::msg::SwiftMsgs>("/drone_command", 10);
        pos_error_pub_ = this->create_publisher<error_msg::msg::Error>("/position_error", 10);
        
        // Create subscribers
        whycon_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/whycon/poses", 1, std::bind(&WayPointServer::whycon_callback, this, _1));
        
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rotors/odometry", 10, std::bind(&WayPointServer::odometry_callback, this, _1));
        
        // Create action server
        action_server_ = rclcpp_action::create_server<NavToWaypoint>(
            this,
            "waypoint_navigation",
            std::bind(&WayPointServer::handle_goal, this, _1, _2),
            std::bind(&WayPointServer::handle_cancel, this, _1),
            std::bind(&WayPointServer::handle_accepted, this, _1));
        
        // Arm the drone
        arm();
        
        // Create timer for PID control loop
        pid_timer_ = this->create_wall_timer(
            sample_time_, std::bind(&WayPointServer::pid_control, this));
        
        RCLCPP_INFO(this->get_logger(), "Waypoint server started - waiting for first position update");
    }

private:
    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavToWaypoint::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f, y=%.2f, z=%.2f (WhyCon coordinates)",
                    goal->x, goal->y, goal->z);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Goal cancelled");
        goal_active_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        // Execute goal in a separate thread
        std::thread{std::bind(&WayPointServer::execute_goal, this, _1), goal_handle}.detach();
    }
    
    void execute_goal(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        
        // Waypoints are already in WhyCon coordinates (decimeters) - use directly
        float waypoint_x = static_cast<float>(goal_handle->get_goal()->x);
        float waypoint_y = static_cast<float>(goal_handle->get_goal()->y);
        float waypoint_z = static_cast<float>(goal_handle->get_goal()->z);
        
        // Set desired state (in WhyCon coordinates - decimeters)
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            desired_state_[0] = waypoint_x;
            desired_state_[1] = waypoint_y;
            desired_state_[2] = waypoint_z;
            goal_active_ = true;
        }
        
        RCLCPP_INFO(this->get_logger(), "New waypoint set: [%.2f, %.2f, %.2f] (WhyCon coordinates)",
                    desired_state_[0], desired_state_[1], desired_state_[2]);
        
        // Reset sphere detection variables
        max_time_inside_sphere_ = 0.0;
        point_in_sphere_start_time_ = -1.0;
        time_inside_sphere_ = 0.0;
        duration_ = current_time_;
        
        // Reset PID integral term when new goal is set (prevent windup from previous goal)
        error_sum_[0] = 0.0f;
        error_sum_[1] = 0.0f;
        error_sum_[2] = 0.0f;
        
        // Create feedback message
        auto feedback = std::make_shared<NavToWaypoint::Feedback>();
        
        // Loop until goal is reached or cancelled
        rclcpp::Rate loop_rate(50); // 50 Hz for feedback
        
        while (rclcpp::ok() && goal_active_) {
            // Check if goal is cancelled
            if (goal_handle->is_canceling()) {
                auto result = std::make_shared<NavToWaypoint::Result>();
                result->stabilization_time = current_time_ - duration_;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal cancelled");
                goal_active_ = false;
                return;
            }
            
            // Update feedback with current position (WhyCon coordinates - decimeters)
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                feedback->current_x = static_cast<double>(drone_position_[0]);
                feedback->current_y = static_cast<double>(drone_position_[1]);
                feedback->current_z = static_cast<double>(drone_position_[2]);
            }
            
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            
            // Check if drone is within sphere (0.8 decimeters for PID controller)
            // As per Python template: "If you are using PID controller, it will become 0.8"
            float radius = 0.8f;  // 0.8 decimeters
            bool drone_is_in_sphere = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                drone_is_in_sphere = is_drone_in_sphere(
                    drone_position_, desired_state_, radius);
            }
            
            // Track time inside sphere
            if (!drone_is_in_sphere && point_in_sphere_start_time_ < 0.0) {
                // Not in sphere, not tracking
            } else if (drone_is_in_sphere && point_in_sphere_start_time_ < 0.0) {
                // First time entering sphere
                point_in_sphere_start_time_ = current_time_;
            } else if (drone_is_in_sphere && point_in_sphere_start_time_ >= 0.0) {
                // Still in sphere
                time_inside_sphere_ = current_time_ - point_in_sphere_start_time_;
                if (time_inside_sphere_ > max_time_inside_sphere_) {
                    max_time_inside_sphere_ = time_inside_sphere_;
                }
            } else if (!drone_is_in_sphere && point_in_sphere_start_time_ >= 0.0) {
                // Exited sphere
                point_in_sphere_start_time_ = -1.0;
                time_inside_sphere_ = 0.0;
            }
            
            // Check if drone has been in sphere for 2 seconds (as per requirements)
            if (max_time_inside_sphere_ >= 2.0) {
                break;
            }
            
            loop_rate.sleep();
        }
        
        // Goal succeeded
        auto result = std::make_shared<NavToWaypoint::Result>();
        result->stabilization_time = current_time_ - duration_;
        goal_handle->succeed(result);
        goal_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal succeeded! Stabilization time: %.2f seconds",
                    result->stabilization_time);
    }
    
    // Subscriber callbacks
    void whycon_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "No poses received");
            return;
        }
        
        // Update drone position (WhyCon coordinates in decimeters)
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            drone_position_[0] = static_cast<float>(msg->poses[0].position.x);
            drone_position_[1] = static_cast<float>(msg->poses[0].position.y);
            drone_position_[2] = static_cast<float>(msg->poses[0].position.z);
            
            // On first position update, initialize desired_state to current position
            // This prevents PID from trying to move to (0,0,0) before a goal is received
            if (!position_received_) {
                desired_state_[0] = drone_position_[0];
                desired_state_[1] = drone_position_[1];
                desired_state_[2] = drone_position_[2];
                position_received_ = true;
                RCLCPP_INFO(this->get_logger(), "First position received: [%.2f, %.2f, %.2f] - PID will hold position until goal received",
                           drone_position_[0], drone_position_[1], drone_position_[2]);
            }
        }
        
        // Update current time from header (in seconds)
        current_time_ = static_cast<double>(msg->header.stamp.sec) + 
                       static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    }
    
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract yaw from quaternion
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        
        // Convert quaternion to Euler angles
        tf2::Quaternion q(qx, qy, qz, qw);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        yaw_ = yaw * 180.0 / M_PI; // Convert to degrees
    }
    
    // PID control function - EXACT same logic as pico_controller_PID.cpp
    void pid_control()
    {
        // Don't run PID if position hasn't been received yet
        if (!position_received_) {
            return;
        }
        
        float current_pos[3];
        float current_setpoint[3];
        
        // Get current state (thread-safe)
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_pos[0] = drone_position_[0];
            current_pos[1] = drone_position_[1];
            current_pos[2] = drone_position_[2];
            current_setpoint[0] = desired_state_[0];
            current_setpoint[1] = desired_state_[1];
            current_setpoint[2] = desired_state_[2];
        }
        
        // Compute error [roll, pitch, throttle] - EXACT same as pico_controller_PID.cpp
        float error[3];
        error[0] = current_setpoint[0] - current_pos[0];  // Roll (X-axis)
        error[1] = current_pos[1] - current_setpoint[1];  // Pitch (Y-axis) - sign reversed
        error[2] = current_setpoint[2] - current_pos[2];  // Throttle (Z-axis)
        
        // Proportional term
        float p_out[3];
        p_out[0] = Kp_[0] * error[0];
        p_out[1] = Kp_[1] * error[1];
        p_out[2] = Kp_[2] * error[2];
        
        // Integral term with anti-windup - EXACT same as pico_controller_PID.cpp
        error_sum_[0] += error[0];
        error_sum_[1] += error[1];
        error_sum_[2] += error[2];
        
        // Clamp integral to prevent windup (same limits as pico_controller_PID.cpp)
        error_sum_[0] = ((error_sum_[0] < -400.0f) ? -400.0f : (error_sum_[0] > 400.0f) ? 400.0f : error_sum_[0]);
        error_sum_[1] = ((error_sum_[1] < -400.0f) ? -400.0f : (error_sum_[1] > 400.0f) ? 400.0f : error_sum_[1]);
        error_sum_[2] = ((error_sum_[2] < -400.0f) ? -400.0f : (error_sum_[2] > 400.0f) ? 400.0f : error_sum_[2]);
        
        float i_out[3];
        i_out[0] = Ki_[0] * error_sum_[0];
        i_out[1] = Ki_[1] * error_sum_[1];
        i_out[2] = Ki_[2] * error_sum_[2];
        
        // Derivative term with low-pass filter - EXACT same as pico_controller_PID.cpp
        float raw_derivative[3];
        raw_derivative[0] = error[0] - prev_error_[0];
        raw_derivative[1] = error[1] - prev_error_[1];
        raw_derivative[2] = error[2] - prev_error_[2];
        
        // Apply the exponential moving average filter
        for (int i = 0; i < 3; ++i) {
            filtered_derivative_[i] = (d_filter_alpha_ * filtered_derivative_[i]) + 
                                      ((1.0f - d_filter_alpha_) * raw_derivative[i]);
        }
        
        float d_out[3];
        d_out[0] = Kd_[0] * filtered_derivative_[0];
        d_out[1] = Kd_[1] * filtered_derivative_[1];
        d_out[2] = Kd_[2] * filtered_derivative_[2];
        
        // Total PID output
        float out_roll = p_out[0] + i_out[0] + d_out[0];
        float out_pitch = p_out[1] + i_out[1] + d_out[1];
        float out_throttle = p_out[2] + i_out[2] + d_out[2];
        
        // Apply output to neutral command (1500) - EXACT same as pico_controller_PID.cpp
        cmd_.rc_roll = 1500 + static_cast<int>(out_roll);
        cmd_.rc_pitch = 1500 + static_cast<int>(out_pitch);
        cmd_.rc_throttle = 1500 - static_cast<int>(out_throttle);  // Negative for throttle
        
        // Limit command values - EXACT same logic as pico_controller_PID.cpp
        cmd_.rc_roll = (cmd_.rc_roll < min_values_[0]) ? min_values_[0] : ((cmd_.rc_roll > max_values_[0]) ? max_values_[0] : cmd_.rc_roll);
        cmd_.rc_pitch = (cmd_.rc_pitch < min_values_[1]) ? min_values_[1] : ((cmd_.rc_pitch > max_values_[1]) ? max_values_[1] : cmd_.rc_pitch);
        cmd_.rc_throttle = (cmd_.rc_throttle < min_values_[2]) ? min_values_[2] : ((cmd_.rc_throttle > max_values_[2]) ? max_values_[2] : cmd_.rc_throttle);
        
        // Update previous errors
        prev_error_[0] = error[0];
        prev_error_[1] = error[1];
        prev_error_[2] = error[2];
        
        // Publish command
        command_pub_->publish(cmd_);
        
        // Publish error
        auto error_msg = error_msg::msg::Error();
        error_msg.roll_error = error[0];
        error_msg.pitch_error = error[1];
        error_msg.throttle_error = error[2];
        error_msg.yaw_error = 0.0f;
        pos_error_pub_->publish(error_msg);
    }
    
    // Helper function to check if drone is in sphere
    bool is_drone_in_sphere(const float drone_pos[3], 
                           const float sphere_center[3], 
                           float radius)
    {
        float dx = drone_pos[0] - sphere_center[0];
        float dy = drone_pos[1] - sphere_center[1];
        float dz = drone_pos[2] - sphere_center[2];
        float distance_sq = dx * dx + dy * dy + dz * dz;
        return distance_sq <= (radius * radius);
    }
    
    // Arm/disarm functions
    void disarm()
    {
        auto cmd = swift_msgs::msg::SwiftMsgs();
        cmd.rc_roll = 1000;
        cmd.rc_pitch = 1000;
        cmd.rc_yaw = 1000;
        cmd.rc_throttle = 1000;
        cmd.rc_aux4 = 1000;
        command_pub_->publish(cmd);
    }
    
    void arm()
    {
        disarm();
        auto cmd = swift_msgs::msg::SwiftMsgs();
        cmd.rc_roll = 1500;
        cmd.rc_pitch = 1500;
        cmd.rc_yaw = 1500;
        cmd.rc_throttle = 1500;
        cmd.rc_aux4 = 2000;
        command_pub_->publish(cmd);
    }
    
    // Member variables
    float drone_position_[3];
    float desired_state_[3];
    swift_msgs::msg::SwiftMsgs cmd_;
    std::chrono::milliseconds sample_time_;
    
    // PID gains [roll, pitch, throttle] - same as pico_controller_PID.cpp
    float Kp_[3];
    float Ki_[3];
    float Kd_[3];
    
    // PID state
    float prev_error_[3];
    float error_sum_[3];
    float filtered_derivative_[3];
    float d_filter_alpha_;
    
    // Limits
    float max_values_[3];
    float min_values_[3];
    
    // Yaw
    double yaw_;
    
    // Sphere detection
    double time_inside_sphere_;
    double max_time_inside_sphere_;
    double point_in_sphere_start_time_;
    double duration_;
    double current_time_;
    
    // Control flags
    bool goal_active_;
    bool position_received_;
    std::mutex state_mutex_;  // Protect shared state between threads
    
    // ROS2 interfaces
    rclcpp::Publisher<swift_msgs::msg::SwiftMsgs>::SharedPtr command_pub_;
    rclcpp::Publisher<error_msg::msg::Error>::SharedPtr pos_error_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr whycon_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp_action::Server<NavToWaypoint>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr pid_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto waypoint_server = std::make_shared<WayPointServer>();
    
    // Use MultiThreadedExecutor for action server
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(waypoint_server);
    
    try {
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_server"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
