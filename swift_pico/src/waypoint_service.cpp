#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "waypoint_navigation/srv/get_waypoints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

/**
 * Waypoint Service Node
 * 
 * Provides a service that returns 8 fixed waypoints for the drone to navigate.
 * Service: /waypoints (waypoint_navigation::srv::GetWaypoints)
 * 
 * Waypoints are in WhyCon coordinates (decimeters) - uniform coordinate system:
 * 1. Hover: [-7.00, 0.00, 29.22]
 * 2. wp1:   [-7.64, 3.06, 29.22]
 * 3. wp2:   [-8.22, 6.02, 29.22]
 * 4. wp3:   [-9.11, 9.27, 29.27]
 * 5. wp4:   [-5.98, 8.81, 29.27]
 * 6. wp5:   [-3.26, 8.41, 29.88]
 * 7. wp6:   [0.87, 8.18, 29.05]
 * 8. wp7:   [3.93, 7.35, 29.05]
 */
class WayPoints : public rclcpp::Node
{
public:
    WayPoints() : Node("waypoints_service")
    {
        // Create service server
        service_ = this->create_service<waypoint_navigation::srv::GetWaypoints>(
            "waypoints",
            std::bind(&WayPoints::waypoint_callback, this, _1, _2));
        
        RCLCPP_INFO(this->get_logger(), "Waypoint service server started on 'waypoints'");
    }

private:
    void waypoint_callback(
        const std::shared_ptr<waypoint_navigation::srv::GetWaypoints::Request> /* request */,
        std::shared_ptr<waypoint_navigation::srv::GetWaypoints::Response> response)
    {
        // 8 fixed waypoints (hover + wp1-wp7)
        // Format: [x, y, z] in WhyCon coordinates (decimeters) - uniform coordinate system
        const std::vector<double> waypoints_x = {
            -7.00,  // Hover
            -7.64,  // wp1
            -8.22,  // wp2
            -9.11,  // wp3
            -5.98,  // wp4
            -3.26,  // wp5
             0.87,  // wp6
             3.93   // wp7
        };
        
        const std::vector<double> waypoints_y = {
             0.00,  // Hover
             3.06,  // wp1
             6.02,  // wp2
             9.27,  // wp3
             8.81,  // wp4
             8.41,  // wp5
             8.18,  // wp6
             7.35   // wp7
        };
        
        const std::vector<double> waypoints_z = {
            29.22,  // Hover
            29.22,  // wp1
            29.22,  // wp2
            29.27,  // wp3
            29.27,  // wp4
            29.88,  // wp5
            29.05,  // wp6
            29.05   // wp7
        };
        
        // Populate response arrays
        response->x = waypoints_x;
        response->y = waypoints_y;
        response->z = waypoints_z;
        
        RCLCPP_INFO(this->get_logger(), "Waypoint service called: Returning %zu waypoints (WhyCon coordinates)",
                    waypoints_x.size());
    }
    
    rclcpp::Service<waypoint_navigation::srv::GetWaypoints>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto waypoints = std::make_shared<WayPoints>();
    
    try {
        rclcpp::spin(waypoints);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_service"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
