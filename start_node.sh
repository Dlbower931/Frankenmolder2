#!/bin/bash

# Source the ROS environment
# This path comes from the 'catkin_make_isolated --install' command in your Dockerfile
source /app/install_isolated/setup.bash

# --- CONFIGURE THE HOST'S CAN INTERFACE ---
# The container runs with network_mode: host + privileged, so this configures the Pi's can0.
# restart-ms: auto-recover from bus-off (e.g. while the ESP32 is reset or reflashed).
# txqueuelen: room to queue frames during a short outage instead of dropping them.
CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
CAN_RESTART_MS="${CAN_RESTART_MS:-100}"
CAN_TXQUEUELEN="${CAN_TXQUEUELEN:-100}"
echo "Configuring $CAN_IFACE (bitrate $CAN_BITRATE, restart-ms $CAN_RESTART_MS, txqueuelen $CAN_TXQUEUELEN)..."
if ip link set "$CAN_IFACE" down && \
   ip link set "$CAN_IFACE" up txqueuelen "$CAN_TXQUEUELEN" type can bitrate "$CAN_BITRATE" restart-ms "$CAN_RESTART_MS"; then
    ip -details link show "$CAN_IFACE" | grep -E "can state|bitrate" | sed 's/^/    /'
else
    echo "WARNING: Failed to configure $CAN_IFACE. The CAN bridge may not work."
fi

# Start ROS Master
roscore &
echo "Waiting 5 seconds for ROS Master..."
sleep 5

# --- LAUNCH THE CAN BRIDGE NODE ---
echo "Attempting to launch CAN Bridge Node..."
rosrun can_bridge_pkg can_bridge_node.py &
echo "CAN Bridge Node launch command sent."

# --- LAUNCH THE TOPIC WATCHDOG ---
echo "Attempting to launch Topic Watchdog..."
rosrun frankenmolder_utils topic_watchdog.py &
echo "Topic Watchdog launch command sent."

# --- START THE GUI NODE ---
# On boot the container (restart: always) can come up before the desktop has started
# and before the host has run `xhost +local:docker` (see setup_host_display.sh).
# Wait in the background until a Tk window can actually open on $DISPLAY, then launch.
(
    echo "Waiting for X display ${DISPLAY:-unset} to accept connections..."
    waited=0
    until python3 -c "import tkinter; tkinter.Tk().destroy()" >/dev/null 2>&1; do
        if [ $((waited % 30)) -eq 0 ]; then
            echo "  display not ready yet (${waited}s). Is the desktop up and has xhost +local:docker run?"
        fi
        sleep 2
        waited=$((waited + 2))
    done
    echo "Display ready after ${waited}s. Launching GUI Node..."
    rosrun frankenmolder_gui extruder_gui_node.py
    echo "GUI Node exited."
) &
echo "GUI Node launch scheduled (waits for display)."

# --- Start Continuous Background Logging Loop ---
echo "Starting continuous background rosbag recording (30 second segments)..."
LOG_DIR="/data/ros_logs" # Use the mounted volume path
mkdir -p $LOG_DIR
( while true; do
    echo "Starting new 30s rosbag segment..."
    # Record all topics (-a), save to LOG_DIR, name with date+time prefix (-o), record for 30 seconds (--duration=30s)
    # The command will exit after 30s, and the loop will restart it.
    (cd $LOG_DIR && rosbag record -a -o frankenmolder_log --duration=30s)
    sleep 1 
done ) & # Run the entire loop in the background
# ------------------------------------------------

echo "ROS services and continuous logging are running in the background."

# Keep the main container script alive
while true; do sleep 1000; done
