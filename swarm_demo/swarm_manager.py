#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import json
import math

class SwarmManager(Node):
    def __init__(self):
        super().__init__('swarm_manager')
        
        # Subscriber for master commands
        self. cmd_subscriber = self. create_subscription(
            String,
            '/swarm/master_command',
            self.command_callback,
            10
        )
        
        # Publishers for individual drone setpoints
        self.drone_setpoint_pubs = {}
        drone_names = ['drone_master', 'drone_1', 'drone_2', 'drone_3']
        
        for drone in drone_names:
            topic = f'/{drone}/setpoint'
            self.drone_setpoint_pubs[drone] = self.create_publisher(Point, topic, 10)
        
        # Formation definitions
        self.formations = {
            "line": {
                'drone_master': [0.0, 0.0, 0.0],
                'drone_1': [2.0, 0.0, 0.0],
                'drone_2': [-2.0, 0.0, 0.0],
                'drone_3': [0.0, -2.0, 0.0]
            },
            "triangle": {
                'drone_master': [0.0, 0.0, 0.0],
                'drone_1': [1.5, 1.0, 0.0],
                'drone_2': [-1.5, 1.0, 0.0],
                'drone_3': [0.0, -1.5, 0.0]
            },
            "diamond": {
                'drone_master': [0.0, 0.0, 0.0],
                'drone_1': [1.0, 0.0, 0.5],
                'drone_2': [-1.0, 0.0, 0.5],
                'drone_3': [0.0, 1.0, -0.5]
            }
        }
        
        self.get_logger().info("Swarm Manager Node started")

    def command_callback(self, msg):
        try:
            command = json.loads(msg.data)
            cmd_type = command.get('type')

            if cmd_type == 'formation':
                self.execute_formation(command)
            elif cmd_type == 'hover':
                self.execute_hover(command)
            elif cmd_type == 'land':
                self.execute_land(command)
            else:
                self.get_logger().warn(f"Unknown command type: {cmd_type}")

        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode command JSON")

    def execute_formation(self, command):
        target = command.get('target', [0.0, 0.0, 2.0])
        formation_type = command.get('formation', 'line')
        
        if formation_type not in self.formations:
            self.get_logger().error(f"Unknown formation: {formation_type}")
            return
        
        formation = self.formations[formation_type]
        
        for drone_name, offset in formation.items():
            if drone_name in self.drone_setpoint_pubs:
                setpoint = Point()
                setpoint.x = target[0] + offset[0]
                setpoint.y = target[1] + offset[1]
                setpoint.z = target[2] + offset[2]
                
                self.drone_setpoint_pubs[drone_name].publish(setpoint)
        
        self.get_logger().info(f"Executed {formation_type} formation at {target}")

    def execute_hover(self, command):
        duration = command.get('duration', 5.0)
        self.get_logger(). info(f"Hovering for {duration} seconds")
        # Keep current positions - no new setpoints needed

    def execute_land(self, command):
        target = command.get('target', [0.0, 0.0, 0.5])

        for drone_name in self.drone_setpoint_pubs:
            setpoint = Point()
            setpoint.x = target[0]
            setpoint.y = target[1]
            setpoint.z = target[2]

            self.drone_setpoint_pubs[drone_name].publish(setpoint)
        
        self.get_logger().info(f"Landing sequence initiated at {target}")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Swarm manager shutting down")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()