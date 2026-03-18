# Swarm Demo — Dynamic ROS 2 / Gazebo Multi-Robot Bridging

A proof-of-concept for **GSoC 2025 — Scalable Multi-Robot Integration and Automated Bridging for ROS 2 and Gazebo** (mentor: Addisu Taddese).

Demonstrates automatic ROS ↔ Gazebo bridge generation and namespace isolation for multi-robot simulations — no manual bridge configuration required.

---

## What This Shows

| Problem (status quo) | This demo |
|---|---|
| Bridge YAML must be hand-written per robot | Generated automatically at launch from `robots.yaml` |
| Adding a robot = editing 3+ files | Adding a robot = 4 lines in one file |
| Separate SDF per robot with hardcoded namespaces | Single SDF template, namespace injected at launch |
| Topic collisions in multi-robot setups | Fully isolated namespaces per drone |

**4 drones, 23 isolated ROS topics, ~34 Hz odometry per drone — from a single launch command.**

---

## System Requirements

- ROS 2 Humble
- Gazebo Ignition (Fortress)
- `ros_gz_bridge`, `ros_gz_sim`
- `actuator_msgs`, `nav_msgs`, `sensor_msgs`

---

## Setup

```bash
mkdir ~/pico_ws && cd ~/pico_ws
git clone -b ig_demo https://github.com/Prathmesh2931/swarm_demo.git --recursive src
colcon build --symlink-install
source install/setup.bash
```

---

## Launch

```bash
ros2 launch rotors_swift_gazebo swarm.launch.py
```

On launch, the system automatically:
1. Reads `config/robots.yaml` to get the fleet definition
2. Generates `swarm_bridge_config.yaml` with all ROS ↔ Gazebo bridge entries
3. Generates a namespaced SDF for each robot from a single template
4. Spawns all robots in Gazebo with isolated namespaces
5. Starts a controller and interface node per robot

---

## Adding a Robot

Open `src/rotors_simulator/rotors_swift_gazebo/config/robots.yaml` and add one entry:

```yaml
robots:
  - name: slave4          # becomes the ROS namespace
    sdf:  swift_pico_template.sdf
    x:  2.0
    y:  2.0
    z:  0.2
```

Re-launch. No other files need to change.

---

## Verify

```bash
# All auto-generated ROS topics
ros2 topic list | grep -E "master|slave"

# Live odometry per drone (~34 Hz)
ros2 topic hz /master/rotors/odometry
ros2 topic hz /slave1/rotors/odometry

# GPS data flowing
ros2 topic echo /master/navsat --once
```

---

## Key Files

```
rotors_swift_gazebo/
├── config/
│   ├── robots.yaml                  ← edit this to change the fleet
│   └── swarm_bridge_config.yaml     ← auto-generated at launch, do not edit
├── launch/
│   └── swarm.launch.py              ← reads robots.yaml, generates bridge + SDFs
rotors_swift_description/
└── models/swift_pico/
    └── swift_pico_template.sdf      ← single SDF with {robot_name} placeholder
```

---

## How the Bridge Generation Works

`swarm.launch.py` calls `_generate_bridge_inline()` before any node starts. It reads the `bridge_topics` templates from `robots.yaml` and expands `{name}` for every robot in the fleet:

```
robots.yaml  →  _generate_bridge_inline()  →  swarm_bridge_config.yaml  →  parameter_bridge node
```

For a 4-robot fleet this produces 13 bridge entries (3 topics × 4 robots + 1 shared clock). Adding a robot adds 3 more entries with zero manual work.

---

## Limitations / Next Steps

This demo uses launch-time generation rather than true runtime discovery. The full GSoC project would extend this by:

- Modifying Gazebo C++ plugins to natively accept a `<namespace>` SDF parameter via the ECS API, eliminating the need for SDF templating entirely
- Implementing a runtime bridge watcher that detects new Gazebo topics as models are spawned and automatically creates ROS bridges without any config file

---

## Author

Prathmesh — GSoC 2026 applicant