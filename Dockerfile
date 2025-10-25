# Start from a ROS Noetic base image
FROM ros:noetic-ros-core-focal

# Set up environment variables
ENV HOME /app
WORKDIR /app

# Install Python dependencies and tools
# Note: You need python3-pip and python3-dev for spidev
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install the necessary Python library (e.g., spidev)
RUN pip3 install spidev

# Create the source directory and copy local code
RUN mkdir -p /app/src/temperature_sensor_pkg
COPY temperature_sensor_pkg /app/src/temperature_sensor_pkg

# Copy and make the top-level start script executable
COPY start_node.sh /app/start_node.sh
RUN chmod +x /app/start_node.sh

# --- CRITICAL FIX: Make the Python node executable (THIS IS THE MISSING STEP) ---
RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone1_temp_node.py
RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone2_temp_node.py

# Build the Catkin workspace (isolated build method)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    cd /app; \
    catkin_make_isolated"