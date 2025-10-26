#!/bin/bash

# --- FIX: Explicitly set permissions for SPI devices ---
echo "Attempting to set permissions for /dev/spidev*..."
chmod 666 /dev/spidev0.0 || echo "WARN: Failed to chmod /dev/spidev0.0"
chmod 666 /dev/spidev0.1 || echo "WARN: Failed to chmod /dev/spidev0.1"
echo "SPI device permissions set (or attempted)."
# ------------------------------------------------------

# Source the ROS environment (Correct path for root user build)
source /opt/ros/noetic/setup.bash
source /app/devel_isolated/setup.bash

# Start ROS Master
roscore &
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# Start Sensor Nodes
echo "Attempting to launch Sensor Node 1..."
rosrun temperature_sensor_pkg extruder_zone1_temp_node.py &
echo "Sensor Node 1 launch command sent."

echo "Attempting to launch Sensor Node 2..."
rosrun temperature_sensor_pkg extruder_zone2_temp_node.py &
echo "Sensor Node 2 launch command sent."

# Launch Heater Control Node directly with python3
echo "Attempting to launch Heater Control Node directly with python3..."
python3 /app/src/heater_control_pkg/src/heater_control_node.py &
EXIT_CODE=$?
echo "Heater Control Node direct launch command sent. Exit code: $EXIT_CODE"

# Launch the Topic Watchdog
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

# Loop indefinitely to restart rosbag record (running as root, no sudo needed)
( while true; do
    echo "Starting new 30s rosbag segment..."
    # Record without sudo
    (cd $LOG_DIR && rosbag record -a -o frankenmolder_log --duration=30s)

    sleep 1
done ) & # Run the entire loop in the background
# ------------------------------------------------

echo "ROS services and continuous logging are running in the background."

# Keep the main container script alive
while true; do sleep 1000; done
