#!/bin/bash

# Source the ROS environment (assuming this is correct now)
source /opt/ros/noetic/setup.bash
source /app/devel_isolated/setup.bash 

# Start ROS Master
roscore &
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# Start Extruder Zone 1 Sensor
rosrun temperature_sensor_pkg extruder_zone1_temp_node.py &

# Start Extruder Zone 2 Sensor (NEW)
rosrun temperature_sensor_pkg extruder_zone2_temp_node.py &

# Launch Foxglove Bridge (assuming you are using the simplified launch file now)
roslaunch temperature_sensor_pkg foxglove_tailscale.launch & 

echo "ROS services are running in the background."
while true; do sleep 1000; done