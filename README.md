# eYRC25-26 Krishi Drone Theme

## Task 1c guide:
mkdir ~/pico_ws<br>
cd ~/pico_ws/<br>
using ssh: **git clone -b task1/task1c git@github.com:Himanshu-Umap/krishi-drone.git --recursive src**<br>
or <br>
using https: **git clone -b task1/task1c https://github.com/Himanshu-Umap/krishi-drone.git  --recursive src**<br>
colcon build<br>

every time you build or start new terminal run: **source ~/.bashrc**<br>
open a new terminal and run: **ros2 launch swift_pico task_1c.launch.py**<br>
In new treminal run: **ros2 run swift_pico pico_controller_PID_cpp**<br>
For recording: press **ctrl+alt+shift+R**<br>

To launch Plotjuggler for visualizing ROS 2 topics, run: **ros2 run plotjuggler plotjuggler**<br>


## Problem Statement:
The goal of this challenge is to navigate a nano drone through a smart greenhouse, guided by real-time vision from an overhead camera, detect infested plants and performs precise, targeted pesticide pickup and application to affected crop areas.

## Implementation:
The challenge will be divided into two phases:
- Simulation: Participants will use the Gazebo simulator to fly the drone and perform a preliminary run of the challenge.

- Hardware: Those who successfully complete the simulation phase may advance to the second stage, where they will receive hardware to replicate their simulation work in a real-world environment.

## Theme developers:
- Arun P Madhu
- Shakthi Magender
- Joel Siby
