# Start from a ROS Noetic base image
FROM ros:noetic-ros-core-focal

# Set up environment variables
ENV HOME /app
WORKDIR /app

# Install Python dependencies and tools
# Note: You need python3-pip and python3-dev for spidev
RUN apt-get update && apt-get install -y \
    ros-noetic-rosbridge-server \
    python3-pip \
    python3-dev \
    git \
    build-essential \
    # --- CRITICAL: Add Tkinter dependency ---
    python3-tk \
    tk-dev \
    python3-rpi.gpio
    && rm -rf /var/lib/apt/lists/*

# Install the necessary Python library (e.g., spidev)
RUN pip3 install spidev

# Create the source directory and copy local code
RUN mkdir -p /app/src/temperature_sensor_pkg
COPY temperature_sensor_pkg /app/src/temperature_sensor_pkg
COPY frankenmolder_gui /app/src/frankenmolder_gui 
COPY frankenmolder_utils /app/src/frankenmolder_utils
COPY heater_control_pkg /app/src/heater_control_pkg

# Copy and make the top-level start script executable
COPY start_node.sh /app/start_node.sh
RUN chmod +x /app/start_node.sh

# --- CRITICAL FIX: Make the Python node executable (THIS IS THE MISSING STEP) ---
RUN chmod +x /app/src/frankenmolder_gui/src/extruder_gui_node.py
RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone1_temp_node.py
RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone2_temp_node.py
RUN chmod +x /app/src/heater_control_pkg/src/heater_control_node.py
RUN chmod +x /app/src/frankenmolder_utils/src/topic_watchdog.py

# Build the Catkin workspace (isolated build method)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    cd /app; \
    catkin_make_isolated"