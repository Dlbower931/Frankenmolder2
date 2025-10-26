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
rosrun temperature_sensor_pkg extruder_zone1_control_node.py &
# Add Zone 3 here when ready:
# rosrun temperature_sensor_pkg extruder_zone3_temp_node.py &

# Start Control Nodes (If/when created)
# rosrun temperature_sensor_pkg extruder_zone1_control_node.py &

# Launch the Topic Watchdog
rosrun frankenmolder_utils topic_watchdog.py &

# --- START THE GUI NODE ---
rosrun frankenmolder_gui extruder_gui_node.py &

# --- Start Continuous Background Logging Loop ---
echo "Starting continuous background rosbag recording (30 second segments)..."
LOG_DIR="/data/ros_logs" # Use the mounted volume path
mkdir -p $LOG_DIR

# Loop indefinitely to restart rosbag record
while true; do
    echo "Starting new 30s rosbag segment..."
    # Record all topics (-a), save to LOG_DIR, name with date+time prefix (-o), record for 30 seconds (--duration=30s)
    # The command will exit after 30s, and the loop will restart it.
    (cd $LOG_DIR && rosbag record -a -o frankenmolder_log --duration=30s)
    
    # Optional: Add a small delay if needed, though usually not required
    sleep 1 
done & # Run the entire loop in the background
# ------------------------------------------------

echo "ROS services and continuous logging are running in the background."

# Keep the main container script alive
while true; do sleep 1000; done
