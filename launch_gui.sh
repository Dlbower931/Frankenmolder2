#!/bin/bash
# launch_gui.sh - Executes the GUI inside the Docker container

CONTAINER_NAME="frankenmolder_ros"
GUI_SCRIPT="/app/install/frankenmolder_gui/lib/frankenmolder_gui/extruder_gui_node.py"

echo "--- Granting X access ---"
# Grant permissions to the container (required on Pi desktop)
xhost +local:docker

echo "--- Starting GUI inside container: $CONTAINER_NAME ---"
# Use 'docker exec' to run the GUI node directly, setting the DISPLAY variable
# The full path to the script is used because we cannot use rosrun inside docker exec easily.
# We also use 'nohup' and '&' to detach the process so your terminal stays open.
# NOTE: The path '/app/install/...' is where Catkin installs executable scripts.
docker exec -d \
    -e DISPLAY=$DISPLAY \
    -e ROS_MASTER_URI=http://100.119.150.8:11311 \
    $CONTAINER_NAME \
    /bin/bash -c "nohup python3 $GUI_SCRIPT &"

echo "GUI execution command sent to container. Check Pi desktop for window."