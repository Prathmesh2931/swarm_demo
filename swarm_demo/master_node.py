#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import json
import time

class MasterNode(Node):
    def __init__(self):
        super().__init__('swarm_master')
        
        # Publisher for swarm commands
        self.cmd_publisher = self.create_publisher(
            String, 
            '/swarm/master_command', 
            10
        )
        
        # Timer for demo commands
        self.timer = self.create_timer(5.0, self.publish_command)
        self.command_counter = 0
        
        # Predefined mission waypoints
        self.missions = [
            {"type": "formation", "target": [0.0, 0.0, 2.0], "formation": "line"},
            {"type": "formation", "target": [5.0, 0.0, 3.0], "formation": "triangle"},
            {"type": "formation", "target": [0.0, 5.0, 2.5], "formation": "diamond"},
            {"type": "hover", "duration": 3.0},
            {"type": "land", "target": [0.0, 0.0, 0.5]}
        ]
        
        self.get_logger().info("Swarm Master Node started")

    def publish_command(self):
        if self.command_counter < len(self. missions):
            mission = self.missions[self.command_counter]
            
            # Create command message
            cmd_msg = String()
            cmd_msg.data = json.dumps(mission)
            
            self.cmd_publisher.publish(cmd_msg)
            
            self.get_logger().info(f"Published mission {self.command_counter + 1}: {mission}")
            self.command_counter += 1
        else:
            self.get_logger().info("Mission sequence complete")

def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Master node shutting down")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()