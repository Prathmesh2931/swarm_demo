import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('swarm_demo')
    model_file = os.path.join(pkg_dir, 'models', 'simple_quad', 'model.sdf')
    bridge_config = os.path.join(pkg_dir, 'config', 'bridge_config.yaml')

    # Start Ignition Gazebo
    gazebo_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', 'empty.sdf'],
        output='screen'
    )

    # Start ROS-Ignition Bridge
    bridge_cmd = TimerAction(
        period=5.0,  # Wait for Gazebo to start
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{"config_file": bridge_config}]
            )
        ]
    )

    # Drone spawn positions and names
    drone_configs = [
        {'name': 'drone_master', 'x': 0.0, 'y': 0.0, 'z': 1.0},
        {'name': 'drone_1', 'x': 2.0, 'y': 0.0, 'z': 1.0},
        {'name': 'drone_2', 'x': -2.0, 'y': 0.0, 'z': 1.0},
        {'name': 'drone_3', 'x': 0.0, 'y': 2.0, 'z': 1.0}
    ]

    # Create spawn commands
    spawn_commands = []
    for i, config in enumerate(drone_configs):
        x, y, z = config['x'], config['y'], config['z']
        spawn_cmd = TimerAction(
            period=3.0 + i * 2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ign', 'service', '-s', '/world/empty/create',
                        '--reqtype', 'ignition.msgs.EntityFactory',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '5000',
                        '--req', f'sdf_filename: "{model_file}" name: "{config["name"]}" pose: {{position: {{x: {x} y: {y} z: {z}}}}}'
                    ],
                    output='screen'
                )
            ]
        )
        spawn_commands.append(spawn_cmd)

    ld = LaunchDescription()
    ld.add_action(LogInfo(msg="Starting Swarm Demo with ROS2 Bridge... "))
    ld.add_action(gazebo_cmd)
    ld.add_action(bridge_cmd)
    
    for cmd in spawn_commands:
        ld.add_action(cmd)

    return ld