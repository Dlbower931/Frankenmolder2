#!/bin/bash

# --- Start ROS Master (roscore) ---
# roscore is the central nervous system for ROS. 
# We run it in the background (&) and discard output (>/dev/null 2>&1)
# You need to run this command if you are not using ros-master in a separate container.
echo "Starting ROS Master (roscore)..."
/opt/ros/noetic/bin/roscore >/dev/null 2>&1 &

# Give roscore a second to start up
sleep 1

# --- Launch the Temperature Publisher Node ---
echo "Launching Temperature Publisher Node..."

# 1. Source the ROS setup file for your workspace
source /app/devel/setup.bash

# 2. Launch your node using rosrun
# We run it in the background (&) so the script doesn't block.
# We redirect output to a log file so we can check it later.
rosrun temperature_sensor_pkg temp_publisher_node.py > /var/log/temp_publisher.log 2>&1 &

echo "ROS system initialized."
echo "View logs with: docker logs injection_molding_ros"

# Keep the container running indefinitely (otherwise it exits)
# This will log any output from the background tasks (roscore, rosnode)
tail -f /dev/null