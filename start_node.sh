#!/bin/bash

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source /app/devel_isolated/setup.bash

# Start ROS Master
roscore &
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# --- FOCUSED DEBUGGING: Run ONLY the heater control node directly ---
# --- Temporarily comment out other nodes ---
# echo "Attempting to launch Sensor Node 1..."
# rosrun temperature_sensor_pkg extruder_zone1_temp_node.py &
# echo "Sensor Node 1 launch command sent."
#
# echo "Attempting to launch Sensor Node 2..."
# rosrun temperature_sensor_pkg extruder_zone2_temp_node.py &
# echo "Sensor Node 2 launch command sent."
#
# echo "Attempting to launch Topic Watchdog..."
# rosrun frankenmolder_utils topic_watchdog.py &
# echo "Topic Watchdog launch command sent."
#
# echo "Attempting to launch GUI Node..."
# rosrun frankenmolder_gui extruder_gui_node.py &
# echo "GUI Node launch command sent."
#
# echo "Starting continuous background rosbag recording..."
# LOG_DIR="/data/ros_logs"
# mkdir -p $LOG_DIR
# ( while true; do ... done ) & # Comment out rosbag loop

# --- Run heater control node in foreground, redirecting output ---
echo "Attempting to launch Heater Control Node directly with python3 IN FOREGROUND..."
LOG_FILE="/data/ros_logs/heater_control.log"
ERR_FILE="/data/ros_logs/heater_control.err"
echo "Logging stdout to $LOG_FILE"
echo "Logging stderr to $ERR_FILE"

# Execute directly, capturing stdout and stderr
python3 /app/src/heater_control_pkg/src/heater_control_node.py > "$LOG_FILE" 2> "$ERR_FILE"
EXIT_CODE=$?
echo "Heater Control Node execution finished. Exit code: $EXIT_CODE"
echo "Check $LOG_FILE and $ERR_FILE for output/errors."
# -----------------------------------------------------------------


# --- Keep container alive (won't be reached if script above runs forever) ---
echo "Script finished (or crashed). Keeping container alive..."
while true; do sleep 1000; done

```

**After Updating the Canvas:**

1.  Stop the container: `docker-compose down`.
2.  **Crucially:** Clear the old log files on your Pi host to ensure we see fresh results:
    ```bash
    rm -f ~/apps/Frankenmolder2/pi_ros_logs/heater_control.*
    ```
3.  Restart the container: `docker-compose up -d`.
4.  Wait about 10-15 seconds.
5.  **Check the new log files:** Look inside the `~/apps/Frankenmolder2/pi_ros_logs/` directory on your Pi host for `heater_control.log` and `heater_control.err`.

    ```bash
    cat ~/apps/Frankenmolder2/pi_ros_logs/heater_control.log
    cat ~/apps/Frankenmolder2/pi_ros_logs/heater_control.err
    

