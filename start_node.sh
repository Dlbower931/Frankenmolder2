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

# Start Control Nodes
rosrun temperature_sensor_pkg extruder_zone1_control_node.py &

# Start Background Logging
echo "Starting background rosbag recording..."
LOG_DIR="/data/ros_logs"
mkdir -p $LOG_DIR
rosbag record -a -o $LOG_DIR/frankenmolder_log -G 3600 &

# --- ADD THIS LINE: Launch the Topic Watchdog ---
rosrun temperature_sensor_pkg topic_watchdog.py &
# ---------------------------------------------

echo "ROS services are running in the background."
while true; do sleep 1000; done
```

---

## 5. Final Steps

1.  **Commit, Push, Pull** all the file changes (`start_node.sh`, `topic_watchdog.py`, `Dockerfile`).
2.  **Rebuild and Run:**
    ```bash
    docker-compose down
    docker-compose up -d --build
    ```

### How to Check

* **Rosbag:** You'll see `.bag` files appearing in the `pi_ros_logs` directory on your Pi host. Use `scp` to retrieve them.
* **Watchdog Errors:** If a sensor node crashes or stops publishing after 10 seconds, error messages from the watchdog will appear in the container logs. Check with:
    ```bash
    docker logs -f frankenmolder_ros
    