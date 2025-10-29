#!/bin/bash

# --- Set permissions for SPI devices ---
echo "Attempting to set permissions for /dev/spidev*..."
chmod 666 /dev/spidev0.0 || echo "WARN: Failed to chmod /dev/spidev0.0"
chmod 666 /dev/spidev0.1 || echo "WARN: Failed to chmod /dev/spidev0.1"
chmod 666 /dev/spidev0.2 || echo "WARN: Failed to chmod /dev/spidev0.2"
echo "SPI device permissions set (or attempted)."

# --- Set permissions for Pico USB port ---
echo "Attempting to set permissions for /dev/ttyACM0..."
chmod 666 /dev/ttyACM0 || echo "WARN: Failed to chmod /dev/ttyACM0"
echo "Pico USB port permissions set (or attempted)."
# ------------------------------------------------------


# Source the ROS environment (Check both potential locations)
SETUP_PATH="/app/install_isolated/setup.bash" # If using --install
if [ ! -f "$SETUP_PATH" ]; then
  SETUP_PATH="/app/devel_isolated/setup.bash" # Fallback to devel if install not found
fi
echo "Sourcing environment from: $SETUP_PATH"
source /opt/ros/noetic/setup.bash
source "$SETUP_PATH"


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

echo "Attempting to launch Sensor Node 3..."
rosrun temperature_sensor_pkg extruder_zone3_temp_node.py &
echo "Sensor Node 3 launch command sent."

# Launch Heater Control Node directly with python3
echo "Attempting to launch Heater Control Node directly with python3..."
# Need to use the correct path based on build structure
HEATER_SCRIPT_PATH="$SETUP_PATH/../lib/heater_control_pkg/heater_control_node.py" # Typical install location
if [ ! -f "$HEATER_SCRIPT_PATH" ]; then
    HEATER_SCRIPT_PATH="/app/src/heater_control_pkg/src/heater_control_node.py" # Fallback to src
fi
echo "Launching heater control from: $HEATER_SCRIPT_PATH"
python3 "$HEATER_SCRIPT_PATH" &
EXIT_CODE=$?
echo "Heater Control Node direct launch command sent. Exit code: $EXIT_CODE"

# --- Launch Pico Bridge Node ---
echo "Attempting to launch Pico Bridge Node..."
rosrun pico_bridge_pkg pico_bridge_node.py &
echo "Pico Bridge Node launch command sent."
# ------------------------------------

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