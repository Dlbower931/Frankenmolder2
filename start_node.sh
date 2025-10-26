#!/bin/bash

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source /app/devel_isolated/setup.bash

# Start ROS Master
roscore &
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# --- FOCUSED DEBUGGING: Test ONLY the python3 interpreter ---
# ... (Commented out nodes remain commented) ...

echo "Attempting to test python3 interpreter directly IN FOREGROUND..."
LOG_DIR="/data/ros_logs"
LOG_FILE="$LOG_DIR/heater_control.log"
ERR_FILE="$LOG_DIR/heater_control.err"
SCRIPT_PATH="/app/src/heater_control_pkg/src/heater_control_node.py" # Keep path for reference if needed later

echo "Logging stdout to $LOG_FILE"
echo "Logging stderr to $ERR_FILE"

# --- Pre-Execution Checks (Keep them for now) ---
echo "--- Pre-Execution Checks ---"
echo "Checking script path: $SCRIPT_PATH"
ls -l "$SCRIPT_PATH"
echo "Checking log directory: $LOG_DIR"
ls -ld "$LOG_DIR"
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

# --- Execute minimal Python command ---
echo "Executing: python3 -c \"print('Hello from Python', flush=True)\""
python3 -c "print('Hello from Python', flush=True)" > "$LOG_FILE" 2> "$ERR_FILE"
EXIT_CODE=$?
echo "Python test execution finished. Exit code: $EXIT_CODE"
echo "Check $LOG_FILE and $ERR_FILE for output/errors."
# -----------------------------------------------------------------


# --- Keep container alive ---
echo "Script finished. Keeping container alive..."
while true; do sleep 1000; done
```

**After Updating the Canvas:**

1.  Stop the container: `docker-compose down`.
2.  Clear old log files: `rm -f ~/apps/Frankenmolder2/pi_ros_logs/heater_control.*`.
3.  Restart the container: `docker-compose up -d`.
4.  Check the main container logs *first* for the pre-execution checks and the "Executing: python3..." message:
    ```bash
    docker logs -f frankenmolder_ros
    ```
5.  Then, check the contents of `heater_control.log`:
    ```bash
    cat ~/apps/Frankenmolder2/pi_ros_logs/heater_control.log
    

