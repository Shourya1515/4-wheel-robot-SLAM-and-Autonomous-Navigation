🚀 4-Wheel Robot SLAM & Autonomous Navigation (ROS2)

This project implements a 4-wheel differential-drive robot in ROS2 capable of:

Real-time SLAM (Simultaneous Localization and Mapping)

Autonomous Navigation using Nav2

Obstacle avoidance & path planning

Simulation in Gazebo Harmonic

Visualization in RViz2

The robot integrates LIDAR, camera, TF transforms, ROS-Gazebo bridges, and Nav2 planners to navigate inside a custom world.

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

Scripts (TF broadcaster, navigation activation)

Custom meshes & sensor models

📁 Folder Structure
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

ros2_control differential drive interface

Gazebo wheel motion plugin

TF Tree
map → odom → base_link → base_laser → camera

🧭 Autonomous Navigation (Nav2)

Launch Nav2:

ros2 launch final_project_description navigation.launch.py


Features:

Global Planner (Smac/NavFn)

Local Planner (DWB Controller)

Local & global costmaps

Recovery behaviors

Click Nav2 Goal in RViz to send navigation targets.

🗺️ SLAM (Mapping)

Run SLAM Toolbox:

ros2 launch final_project_description slam.launch.py


Capabilities:

Real-time mapping

Loop closure

LIDAR scan matching

Save the map:

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: 'my_navigation_map'}"

🏠 Gazebo Simulation

Launch the simulation:

ros2 launch final_project_description house.launch.py


Loads:

Custom house world

Robot + sensors

ROS–Gazebo bridges

👀 RViz Visualization

Launch RViz:

ros2 launch final_project_description rviz.launch.py


Displays:

LIDAR scan

Map

Robot model

TF

Navigation tools

🔗 ROS–Gazebo Bridge

Configured in:

config/gazebo_bridge.yaml


Bridges topics such as:

Gazebo Topic	ROS Topic
/cmd_vel	/cmd_vel
/scan	/scan
/odom	/odom
/tf	/tf
/tf_static	/tf_static
🔧 TF Broadcasting

The script odom_to_tf.py publishes:

odom → base_link


Required for SLAM and Nav2.

🛠️ Build Instructions

Inside workspace:

colcon build
source install/setup.bash

▶️ Run Everything Together

SLAM + Gazebo + RViz:

ros2 launch final_project_description display.launch.py


Full Nav2 system:

bash scripts/activate_nav.sh

🚀 Future Improvements

EKF sensor fusion (IMU + wheel odometry)

More realistic wheel friction modeling

Multi-floor mapping

Hybrid-A* global planner

Path smoothing
