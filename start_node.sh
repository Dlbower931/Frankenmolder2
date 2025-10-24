#!/bin/bash

# Source the core ROS environment
source /opt/ros/noetic/setup.bash

# --- FIX: Source the correct aggregate path for the isolated build ---
source /app/devel_isolated/setup.bash 

# Start ROS Master (roscore) in the background
roscore &

# Wait for roscore to be fully up before launching nodes
sleep 5

# Start your ROS Node in the background
rosrun temperature_sensor_pkg temp_publisher_node.py &

echo "ROS services are running in the background."

# Keep the container running
while true; do sleep 1000; done