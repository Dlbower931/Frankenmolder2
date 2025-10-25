#!/bin/bash

# --- 1. Set the ROS Master URI (CRITICAL for host-side ROS communication) ---
# Replace <PI_TAILSCALE_IP> with the actual Tailscale IP (e.g., 100.119.150.8)
export ROS_MASTER_URI=http://<PI_TAILSCALE_IP>:11311

# --- 2. Source the local Catkin workspace setup file ---
# This makes 'rosrun' and your package executables available
source ~/apps/Frankenmolder2/devel_isolated/setup.bash

# --- 3. Execute the GUI Node ---
# '&' is not needed here, as the script's job is done once the GUI is closed.
echo "Launching Frankenmolder GUI..."
rosrun frankenmolder_gui frankenmolder_gui_node.py