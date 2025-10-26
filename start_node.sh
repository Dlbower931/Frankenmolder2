#!/bin/bash

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source /app/devel_isolated/setup.bash

# Start ROS Master
roscore &
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# --- FOCUSED DEBUGGING: Run ONLY the heater control node directly ---
# ... (Commented out nodes remain commented) ...

# --- Run heater control node in foreground, redirecting output ---
echo "Attempting to launch Heater Control Node directly with python3 IN FOREGROUND..."
LOG_DIR="/data/ros_logs"
LOG_FILE="$LOG_DIR/heater_control.log"
ERR_FILE="$LOG_DIR/heater_control.err"
SCRIPT_PATH="/app/src/heater_control_pkg/src/heater_control_node.py"

echo "Logging stdout to $LOG_FILE"
echo "Logging stderr to $ERR_FILE"

# --- ADDED DEBUG CHECKS ---
echo "--- Pre-Execution Checks ---"
echo "Checking script path: $SCRIPT_PATH"
ls -l "$SCRIPT_PATH" # Check if file exists and permissions
echo "Checking log directory: $LOG_DIR"
ls -ld "$LOG_DIR" # Check directory existence and permissions
echo "Testing write access to log files..."
echo "Test log write" > "$LOG_FILE"
echo "Test error write" > "$ERR_FILE"
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to write test log files. Check permissions on $LOG_DIR"
else
    echo "Write test successful."
fi
echo "Current user: $(whoami)"
echo "--------------------------"
# --- END DEBUG CHECKS ---


# Execute directly, capturing stdout and stderr
# Overwrite test files with actual output
python3 "$SCRIPT_PATH" > "$LOG_FILE" 2> "$ERR_FILE"
EXIT_CODE=$?
echo "Heater Control Node execution finished. Exit code: $EXIT_CODE"
echo "Check $LOG_FILE and $ERR_FILE for output/errors."
# -----------------------------------------------------------------


# --- Keep container alive ---
echo "Script finished (or crashed). Keeping container alive..."
while true; do sleep 1000; done

```

**After Updating the Canvas:**

1.  Stop the container: `docker-compose down`.
2.  Clear old log files again: `rm -f ~/apps/Frankenmolder2/pi_ros_logs/heater_control.*`.
3.  Restart the container: `docker-compose up -d`.
4.  Check the main container logs *first* to see the output of the new debug checks:
    ```bash
    docker logs -f frankenmolder_ros
    ```
5.  Then, check the contents of the `heater_control.log` and `heater_control.err` files again on your Pi host:
    ```bash
    cat ~/apps/Frankenmolder2/pi_ros_logs/heater_control.log
    cat ~/apps/Frankenmolder2/pi_ros_logs/heater_control.err
    

