#!/bin/bash

# Source the ROS environment
source /opt/ros/noetic/setup.bash

# --- CRITICAL FIX: Source the correct aggregate setup file ---
# Isolated builds often use the 'install_isolated' path structure
# The final aggregate install file is generally the most reliable one to source.
source /app/install_isolated/setup.bash 

# Start ROS Master (roscore) in the background
roscore &

# Wait for roscore to be fully up before launching nodes
sleep 5

# Start your ROS Node in the background
rosrun temperature_sensor_pkg temp_publisher_node.py &

echo "ROS services are running in the background."

# Keep the container running
while true; do sleep 1000; done