#!/bin/bash

# Source the ROS environment
# This path comes from the 'catkin_make_isolated --install' command in your Dockerfile
source /app/install_isolated/setup.bash

# Start ROS Master
roscore &
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# --- LAUNCH THE CAN BRIDGE NODE ---
echo "Attempting to launch CAN Bridge Node..."
rosrun can_bridge_pkg can_bridge_node.py &
echo "CAN Bridge Node launch command sent."

# --- LAUNCH THE TOPIC WATCHDOG ---
echo "Attempting to launch Topic Watchdog..."
rosrun frankenmolder_utils topic_watchdog.py &
echo "Topic Watchdog launch command sent."

# --- START THE GUI NODE ---
echo "Attempting to launch GUI Node..."
rosrun frankenmolder_gui extruder_gui_node.py &
echo "GUI Node launch command sent."

# --- Start Continuous Background Logging Loop ---
echo "Starting continuous background rosbag recording (30 second segments)..."
LOG_DIR="/data/ros_logs" # Use the mounted volume path
mkdir -p $LOG_DIR
( while true; do
    echo "Starting new 30s rosbag segment..."
    # Record all topics (-a), save to LOG_DIR, name with date+time prefix (-o), record for 30 seconds (--duration=30s)
    # The command will exit after 30s, and the loop will restart it.
    (cd $LOG_DIR && rosbag record -a -o frankenmolder_log --duration=30s)
    sleep 1 
done ) & # Run the entire loop in the background
# ------------------------------------------------

echo "ROS services and continuous logging are running in the background."

# Keep the main container script alive
while true; do sleep 1000; done
