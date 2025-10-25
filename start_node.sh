#!/bin/bash

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source /app/devel_isolated/setup.bash

# Start ROS Master
roscore &
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# Start Sensor Nodes
rosrun temperature_sensor_pkg extruder_zone1_temp_node.py &
rosrun temperature_sensor_pkg extruder_zone2_temp_node.py &

# Start Control Nodes
# rosrun temperature_sensor_pkg extruder_zone1_control_node.py & # Removed as it doesn't exist yet

# Start Background Logging
echo "Starting background rosbag recording..."
LOG_DIR="/data/ros_logs" # Use the mounted volume path
mkdir -p $LOG_DIR
# Record all topics (-a), save to LOG_DIR, name with date+time prefix (-o), split every 1000 messages (-L 1000)
rosbag record -a -o $LOG_DIR/frankenmolder_log -L 1000 &
# ------------------------------------------------

# Launch the Topic Watchdog (using the new package name)
rosrun frankenmolder_utils topic_watchdog.py &

# --- START THE GUI NODE ---
# Launches the Tkinter window onto the Pi's host desktop via X-Forwarding
rosrun frankenmolder_gui extruder_gui_node.py &

echo "ROS services are running in the background."
while true; do sleep 1000; done
