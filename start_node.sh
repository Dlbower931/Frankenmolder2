#!/bin/bash

# Source the ROS environment
source /opt/ros/noetic/setup.bash
# --- FIX: Source the correct setup file path for rosuser ---
source /home/rosuser/app/devel_isolated/setup.bash

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

# Attempting to launch Heater Control Node...
echo "Attempting to launch Heater Control Node..."
rosrun heater_control_pkg heater_control_node.py &
EXIT_CODE=$?
echo "Heater Control Node launch command sent. Exit code: $EXIT_CODE"

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

# Loop indefinitely to restart rosbag record
( while true; do
    echo "Starting new 30s rosbag segment..."
    # Record all topics (-a), save to LOG_DIR, name with date+time prefix (-o), record for 30 seconds (--duration=30s)
    # The command will exit after 30s, and the loop will restart it.
    # --- FIX: Run rosbag with sudo to ensure write permissions ---
    (cd $LOG_DIR && sudo rosbag record -a -o frankenmolder_log --duration=30s)

    # Optional: Add a small delay if needed, though usually not required
    sleep 1
done ) & # Run the entire loop in the background
# ------------------------------------------------

echo "ROS services and continuous logging are running in the background."

# Keep the main container script alive
while true; do sleep 1000; done
```

**To Apply the Fix:**

1.  Replace the content of your `start_node.sh` file with the code above.
2.  Stop and restart your container:
    ```bash
    docker-compose down
    docker-compose up -d
    

