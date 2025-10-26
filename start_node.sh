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

# Start Background Logging for a fixed duration
echo "Starting background rosbag recording (30 second segments)..."
LOG_DIR="/data/ros_logs" # Use the mounted volume path
mkdir -p $LOG_DIR
# Record all topics (-a), save to LOG_DIR, name with date+time prefix (-o), record for 30 seconds (--duration=30s)
# Note: This command will EXIT after 30 seconds. To make it continuous, it needs to be wrapped in a loop.
(cd $LOG_DIR && rosbag record -a -o frankenmolder_log --duration=30s) &
# ------------------------------------------------

# Launch the Topic Watchdog (using the new package name)
rosrun frankenmolder_utils topic_watchdog.py &

# --- START THE GUI NODE ---
# Launches the Tkinter window onto the Pi's host desktop via X-Forwarding
rosrun frankenmolder_gui extruder_gui_node.py &

echo "ROS services are running in the background."
while true; do sleep 1000; done
