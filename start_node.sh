#!/bin/bash

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source /app/devel_isolated/setup.bash

# Start ROS Master
roscore &
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# --- Restore Node Launches ---
echo "Attempting to launch Sensor Node 1..."
rosrun temperature_sensor_pkg extruder_zone1_temp_node.py &
echo "Sensor Node 1 launch command sent."

echo "Attempting to launch Sensor Node 2..."
rosrun temperature_sensor_pkg extruder_zone2_temp_node.py &
echo "Sensor Node 2 launch command sent."

# Launch Heater Control Node directly with python3 (Keep this method as it was working before)
echo "Attempting to launch Heater Control Node directly with python3..."
python3 /app/src/heater_control_pkg/src/heater_control_node.py &
EXIT_CODE=$?
echo "Heater Control Node direct launch command sent. Exit code: $EXIT_CODE"

echo "Attempting to launch Topic Watchdog..."
rosrun frankenmolder_utils topic_watchdog.py &
echo "Topic Watchdog launch command sent."

echo "Attempting to launch GUI Node..."
rosrun frankenmolder_gui extruder_gui_node.py &
echo "GUI Node launch command sent."

# --- Restore Continuous Background Logging Loop ---
echo "Starting continuous background rosbag recording (30 second segments)..."
LOG_DIR="/data/ros_logs" # Use the mounted volume path
mkdir -p $LOG_DIR

# Loop indefinitely to restart rosbag record (running as root)
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
