#!/bin/bash

# NOTE: We no longer need to chmod /dev/spi* or /dev/tty*

# Source the ROS environment
# This path is correct for the multi-stage build's install space
SETUP_PATH="/app/install_isolated/setup.bash"
echo "Sourcing environment from: $SETUP_PATH"
source /opt/ros/noetic/setup.bash
source "$SETUP_PATH"


# Start ROS Master
roscore &
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# --- LAUNCH NEW CAN BRIDGE NODE ---
echo "Attempting to launch CAN Bridge Node..."
rosrun can_bridge_pkg can_bridge_node.py &
echo "CAN Bridge Node launch command sent."
# ------------------------------------

# --- Old nodes are no longer needed ---
# rosrun temperature_sensor_pkg ... (DELETED)
# rosrun heater_control_pkg ... (DELETED)
# rosrun pico_bridge_pkg ... (DELETED)

# Launch the Topic Watchdog (Still useful)
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
    (cd $LOG_DIR && rosbag record -a -o frankenmolder_log --duration=30s)
    sleep 1 
done ) & 
# ------------------------------------------------

echo "ROS services and continuous logging are running in the background."

# Keep the main container script alive
while true; do sleep 1000; done