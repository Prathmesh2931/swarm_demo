#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
import sys
import math

class FlyingDroneController(Node):
    def __init__(self, drone_name):
        super().__init__(f'{drone_name}_controller')
        self.drone_name = drone_name
        
        # Subscribe to setpoints
        self.setpoint_sub = self.create_subscription(
            Point,
            f'/{drone_name}/setpoint',
            self.setpoint_callback,
            10
        )
        
        # Subscribe to odometry from Gazebo (via bridge)
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{drone_name}/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for motor commands (via bridge to Gazebo)
        self.motor_pub = self.create_publisher(
            Float64MultiArray, 
            f'/{drone_name}/command/motor_speed', 
            10
        )
        
        # Republish sensor data with better naming
        self.imu_sub = self.create_subscription(Imu, f'/{drone_name}/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, f'/{drone_name}/gps/fix', self.gps_callback, 10)
        
        self.imu_pub = self.create_publisher(Imu, f'/{drone_name}/imu', 10)
        self.gps_pub = self.create_publisher(NavSatFix, f'/{drone_name}/gps', 10)
        
        # Flight controller timer
        self.control_timer = self.create_timer(0.1, self.flight_control)
        
        # State variables
        self. current_pos = [0.0, 0.0, 1.0]
        self.target_pos = [0.0, 0.0, 2.0]
        self.is_flying = False
        self.hover_thrust = 400.0
        
        # PID gains
        self.kp = [2.0, 2.0, 5.0]
        self.kd = [1.0, 1.0, 2.0]
        
        self.get_logger().info(f"Flying Drone Controller for {drone_name} started! 🚁")

    def setpoint_callback(self, msg):
        self.target_pos = [msg.x, msg.y, msg.z]
        self.is_flying = True
        self.get_logger().info(f"{self.drone_name} FLYING TO: {msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}")

    def odom_callback(self, msg):
        # Update current position from Gazebo odometry
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z

    def imu_callback(self, msg):
        self.imu_pub.publish(msg)

    def gps_callback(self, msg):
        self.gps_pub.publish(msg)

    def flight_control(self):
        # Calculate control
        if self.is_flying:
            # Position errors
            dx = self.target_pos[0] - self.current_pos[0]
            dy = self.target_pos[1] - self.current_pos[1]
            dz = self.target_pos[2] - self.current_pos[2]
            
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            if distance < 0.2:
                self.is_flying = False
                self.get_logger().info(f"{self.drone_name} REACHED TARGET! 🎯")

            # PID control
            thrust = self.hover_thrust + (self.kp[2] * dz)
            roll_torque = -(self.kp[1] * dy)  # Negative for correct direction
            pitch_torque = (self.kp[0] * dx)
            yaw_torque = 0.0
            
            # Convert to motor speeds (quadcopter X configuration)
            motor_0 = thrust - roll_torque + pitch_torque - yaw_torque  # Front Right
            motor_1 = thrust + roll_torque - pitch_torque - yaw_torque  # Back Left  
            motor_2 = thrust + roll_torque + pitch_torque + yaw_torque  # Front Left
            motor_3 = thrust - roll_torque - pitch_torque + yaw_torque  # Back Right
            
            # Limit motor speeds
            motors = [max(0, min(800, m)) for m in [motor_0, motor_1, motor_2, motor_3]]
            
            self.get_logger().info(f"{self.drone_name} FLYING: pos=({self.current_pos[0]:.1f},{self.current_pos[1]:.1f},{self.current_pos[2]:.1f}) motors={motors[0]:.0f},{motors[1]:.0f},{motors[2]:. 0f},{motors[3]:.0f}")
        else:
            # Hover
            motors = [self.hover_thrust, self.hover_thrust, self.hover_thrust, self.hover_thrust]
        
        # Publish motor commands
        motor_msg = Float64MultiArray()
        motor_msg.data = motors
        self.motor_pub.publish(motor_msg)

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: drone_controller.py <drone_name>")
        return
        
    drone_name = sys.argv[1]
    rclpy.init(args=args)
    node = FlyingDroneController(drone_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger(). info(f"Flying drone controller for {drone_name} shutting down")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()