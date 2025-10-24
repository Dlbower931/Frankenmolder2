#!/bin/bash

# Source the core ROS environment
source /opt/ros/noetic/setup.bash

# FIX: Source the correct aggregate path from the isolated build
source /app/devel_isolated/setup.bash 

# Start ROS Master (roscore) in the background
roscore &

# Wait for roscore to be fully up before launching nodes
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# Start your ROS Node in the background
rosrun temperature_sensor_pkg temp_publisher_node.py &

echo "ROS services are running in the background."
echo "Check status with: docker exec frankenmolder_ros rosnode list"

# Keep the container running indefinitely
while true; do sleep 1000; done