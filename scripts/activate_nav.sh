#!/bin/bash

echo "=== Loading Map ==="
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '/home/shourya15/ros_project/install/final_project_description/share/final_project_description/maps/my_navigation_map.yaml'}"

echo "Waiting 3 seconds for map to load..."
sleep 3

echo ""
echo "=== Activating Navigation Nodes ==="

# Function to activate a node from any state
activate_node() {
    NODE=$1
    STATE=$(ros2 lifecycle get $NODE 2>/dev/null | awk '{print $1}')
    
    case $STATE in
        "unconfigured")
            echo "Configuring and activating $NODE..."
            ros2 lifecycle set $NODE configure
            ros2 lifecycle set $NODE activate
            ;;
        "inactive")
            echo "Activating $NODE..."
            ros2 lifecycle set $NODE activate
            ;;
        "active")
            echo "$NODE already active ✓"
            ;;
        *)
            echo "Unknown state for $NODE: $STATE"
            ;;
    esac
}

# Activate all nodes
activate_node /bt_navigator
activate_node /planner_server
activate_node /controller_server
activate_node /velocity_smoother
activate_node /collision_monitor
activate_node /behavior_server

echo ""
echo "=== Setting Initial Pose ==="
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -3.5, y: -3.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

echo ""
echo "✓ Navigation ready! Send goals in RViz with 2D Goal Pose."
