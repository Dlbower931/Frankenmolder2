#!/bin/bash
# launch_gui.sh - Executes the GUI inside the Docker container

CONTAINER_NAME="frankenmolder_ros"
# The path to the executable script inside the container's build environment
GUI_SCRIPT="/app/install/frankenmolder_gui/lib/frankenmolder_gui/extruder_gui_node.py" 

echo "--- Granting X access ---"
# Grant permissions to the container (required on Pi desktop)
# This is crucial when running with sudo
xhost +local:docker

echo "--- Starting GUI inside container: $CONTAINER_NAME ---"

# --- CRITICAL FIX: Use sudo to run docker exec, and pass DISPLAY explicitly ---
# Note: The use of '$USER' in the XAUTHORITY path is key.
docker exec -d \
    -u root \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY="/home/$SUDO_USER/.Xauthority" \
    -e ROS_MASTER_URI="http://100.119.150.8:11311" \
    $CONTAINER_NAME \
    /bin/bash -c "nohup python3 $GUI_SCRIPT &"

echo "GUI execution command sent to container. Check Pi desktop for window."