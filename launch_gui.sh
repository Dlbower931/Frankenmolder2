#!/bin/bash
# launch_gui.sh - Executes the GUI inside the Docker container

# --- Configuration ---
CONTAINER_NAME="frankenmolder_ros"
# The correct path to the script in the source directory (since it wasn't installed)
GUI_SCRIPT="/app/src/frankenmolder_gui/src/extruder_gui_node.py"

echo "--- Granting X access for Docker ---"
# Grant permissions to the container (required on Pi desktop)
# This is crucial when running with sudo
xhost +local:docker

# Determine the actual user's home path for XAUTHORITY
# $SUDO_USER holds the original user's name when run via sudo
if [ -z "$SUDO_USER" ]; then
    USER_HOME="$HOME"
    AUTH_USER="$USER"
else
    USER_HOME="/home/$SUDO_USER"
    AUTH_USER="$SUDO_USER"
fi

echo "--- Starting GUI inside container: $CONTAINER_NAME ---"

# The command to execute inside the container:
# 1. Source the base ROS installation
# 2. Source the isolated workspace setup file for custom packages
# 3. Run the Python node directly with python3
EXEC_CMD="source /opt/ros/noetic/setup.bash && source /app/devel_isolated/setup.bash && nohup python3 $GUI_SCRIPT &"

# --- Execute the command inside the container ---
docker exec -d \
    -u root \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY="$USER_HOME/.Xauthority" \
    -e ROS_MASTER_URI="http://100.119.150.8:11311" \
    $CONTAINER_NAME \
    /bin/bash -c "$EXEC_CMD"

echo "GUI execution command sent to container. Check Pi desktop for window."
echo "If it fails, use 'docker exec -it $CONTAINER_NAME /bin/bash' to debug."