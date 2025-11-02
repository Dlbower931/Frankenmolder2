# --- STAGE 1: The Builder ---
# This stage builds all your ROS packages.
# We start from a full ROS image that has all the build tools.
# --- FIX: Corrected image name (removed extra '-ros') ---
FROM ros:noetic-desktop AS builder

# Install system dependencies for your packages
# We need can-utils (for host tools) and python3-can (for the python script)
# We also still need python3-tk for the GUI
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-can \
    python3-tk \
    can-utils \
    git \
    && rm -rf /var/lib/apt/lists/*

# We don't need spidev or RPi.GPIO anymore

# Set up the Catkin workspace
WORKDIR /app
RUN mkdir -p /app/src

# --- Copy ONLY the packages we need ---
# We are NOT copying temperature_sensor_pkg, heater_control_pkg, or pico_bridge_pkg
COPY can_bridge_pkg /app/src/can_bridge_pkg
COPY frankenmolder_gui /app/src/frankenmolder_gui
COPY frankenmolder_utils /app/src/frankenmolder_utils

# --- Make Python nodes executable ---
# We must do this *before* building
RUN chmod +x /app/src/can_bridge_pkg/src/can_bridge_node.py
RUN chmod +x /app/src/frankenmolder_gui/src/extruder_gui_node.py
RUN chmod +x /app/src/frankenmolder_utils/src/topic_watchdog.py

# --- Build the Catkin workspace ---
# Source ROS and build using --install
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    cd /app; \
    catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release"


# --- STAGE 2: The Final, Small Image ---
# We start from a minimal ROS image
# --- FIX: Corrected image name to 'ros:noetic-ros-base' ---
FROM ros:noetic-ros-base

# Install ONLY the runtime dependencies (not the build tools)
RUN apt-get update && apt-get install -y \
    python3-can \
    python3-tk \
    can-utils \
    # We still need rosbridge if the GUI uses it (can be removed if GUI uses ROS1 connection)
    ros-noetic-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Set up the app directory
WORKDIR /app

# Copy the built workspace from the 'builder' stage
# This copies the /app/install_isolated directory
COPY --from=builder /app/install_isolated /app/install_isolated

# Copy the start script
COPY start_node.sh /app/start_node.sh
RUN chmod +x /app/start_node.sh

# This final image is now much smaller!

