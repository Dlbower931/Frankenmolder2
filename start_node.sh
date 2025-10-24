#!/bin/bash

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source /app/devel/setup.bash

# Start ROS Master (roscore) in the background
roscore &

# Wait for roscore to be fully up before launching nodes
sleep 5

# Start your ROS Node in the background
rosrun temperature_sensor_pkg temp_publisher_node.py &

echo "ROS services are running in the background."

# ----------------------------------------------------
# 2. The Final Blocking Command (Crucial)
# ----------------------------------------------------

# This command MUST be a blocking command that never exits.
# If this command finishes, the container stops (exits with code 0).
while true; do sleep 1000; done