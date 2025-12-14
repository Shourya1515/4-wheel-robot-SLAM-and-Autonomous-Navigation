🚀 4-Wheel Robot SLAM & Autonomous Navigation (ROS2)

This project implements a 4-wheel differential-drive robot in ROS2 capable of:

Real-time SLAM (Simultaneous Localization and Mapping)

Autonomous Navigation using Nav2

Obstacle avoidance & path planning

Simulation in Gazebo Harmonic

Visualization in RViz2

The robot integrates LIDAR, camera, TF transforms, ROS-Gazebo bridges, and Nav2 planners to navigate inside a custom world environment.

🎥 Demonstration Videos
🗺️ SLAM Mapping Demo

https://github.com/Shourya1515/4-wheel-robot-SLAM-and-Autonomous-Navigation/raw/main/mapping_updated.mp4

🧭 Autonomous Navigation Demo

https://github.com/Shourya1515/4-wheel-robot-SLAM-and-Autonomous-Navigation/raw/main/Nav%20updated.mp4

📦 Package Overview

This repository contains the full ROS2 package:

final_project_description


The package includes:

URDF/Xacro robot model

Gazebo simulation worlds

Nav2 configuration

SLAM toolbox configuration

RViz visualizations

Scripts for TF broadcasting

Custom meshes & sensor models

🧱 Folder Structure
final_project_description/
│
├── CMakeLists.txt
├── package.xml
│
├── config/
│   ├── display.rviz
│   ├── gazebo_bridge.yaml
│   ├── nav2_params.yaml
│   ├── slam.yaml
│   └── ros_gz_bridge_gazebo.yaml
│
├── launch/
│   ├── display.launch.py
│   ├── gazebo.launch.py
│   ├── house.launch.py
│   ├── navigation.launch.py
│   ├── rviz.launch.py
│   └── slam.launch.py
│
├── maps/
│   ├── my_navigation_map.pgm
│   └── my_navigation_map.yaml
│
├── meshes/
│   ├── base_link.stl
│   ├── lidar_1.stl
│   ├── camera_1.stl
│   ├── left_front_wheel_1.stl
│   ├── right_front_wheel_1.stl
│   ├── left_back_wheel_1.stl
│   └── right_back_wheel_1.stl
│
├── rviz/
│   └── navigation.rviz
│
├── scripts/
│   ├── activate_nav.sh
│   └── odom_to_tf.py
│
└── worlds/
    └── house_world.sdf

🤖 Robot Description

The robot is a 4-wheel differential-drive mobile robot with:

Sensors

2D LIDAR

Forward camera

Control

ros2_control + differential drive hardware interface

Gazebo plugins for wheel motion

TF tree:

map → odom → base_link → base_laser → camera

🧭 Autonomous Navigation (Nav2)

Launch the Nav2 stack:

ros2 launch final_project_description navigation.launch.py


Features:

Global Planner (Smac/NavFn)

Local Planner (DWB)

Costmaps

Recovery behaviors

Path planning + obstacle avoidance

Send goals using RViz2 Nav2 Goal Tool.

🗺️ SLAM (Mapping)

Run SLAM Toolbox:

ros2 launch final_project_description slam.launch.py


Capabilities:

Real-time mapping

Loop closure

LIDAR-based scan matching

Map saving

Save the map:

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: 'my_navigation_map'}"

🏠 Gazebo Simulation

Launch full Gazebo simulation:

ros2 launch final_project_description house.launch.py


Loads:

Custom world

Robot + sensors

Bridges for ROS topics

👀 RViz Visualization

Launch RViz configuration:

ros2 launch final_project_description rviz.launch.py


Includes:

LIDAR

Map

TF

Robot model

Navigation tools

🔀 ROS–Gazebo Bridge

Bridges handled in:

config/gazebo_bridge.yaml


Bridged topics:

Gazebo	ROS
/cmd_vel	/cmd_vel
/odom	/odom
/scan	/scan
/tf	/tf
/tf_static	/tf_static
🔧 TF Broadcasting Script

scripts/odom_to_tf.py publishes:

odom → base_link


Required for SLAM and Nav2.

🛠️ Build

Inside workspace:

colcon build
source install/setup.bash

▶️ Run Everything

SLAM + RViz + Gazebo:

ros2 launch final_project_description display.launch.py


Full navigation system:

bash scripts/activate_nav.sh

🚀 Future Improvements

EKF sensor fusion

IMU integration

Improved friction model

Path smoothing

Hybrid-A* planner

Multi-floor mapping
