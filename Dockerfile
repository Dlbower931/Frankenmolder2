# Start from a ROS Noetic base image
FROM ros:noetic-ros-core-focal

# Set up environment variables
ENV HOME /app
WORKDIR /app

# Install Python dependencies and tools
RUN apt-get update && apt-get install -y \
    ros-noetic-rosbridge-server \
    python3-pip \
    python3-dev \
    git \
    build-essential \
    python3-tk \
    tk-dev \
    python3-rpi.gpio \
    sudo \
    # --- ADDED: Install python3-serial for PICO ---
    python3-serial \
    && rm -rf /var/lib/apt/lists/* \
    # --- Create necessary groups if they don't exist ---
    && groupadd gpio || true \
    && groupadd spi || true \
    && groupadd dialout || true \
    && groupadd render || true

# Install spidev via pip
RUN pip3 install spidev

# Create the source directory and copy local code
RUN mkdir -p /app/src/temperature_sensor_pkg \
             /app/src/frankenmolder_gui \
             /app/src/frankenmolder_utils \
             /app/src/heater_control_pkg \
             /app/src/pico_bridge_pkg
COPY temperature_sensor_pkg /app/src/temperature_sensor_pkg
COPY frankenmolder_gui /app/src/frankenmolder_gui
COPY frankenmolder_utils /app/src/frankenmolder_utils
COPY heater_control_pkg /app/src/heater_control_pkg
COPY pico_bridge_pkg /app/src/pico_bridge_pkg

# --- Make Python nodes executable ---
RUN chmod +x /app/src/frankenmolder_gui/src/extruder_gui_node.py
RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone1_temp_node.py
RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone2_temp_node.py
RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone3_temp_node.py
RUN chmod +x /app/src/heater_control_pkg/src/heater_control_node.py
RUN chmod +x /app/src/frankenmolder_utils/src/topic_watchdog.py
# --- ADDED: chmod for pico bridge node ---
RUN chmod +x /app/src/pico_bridge_pkg/src/pico_bridge_node.py

# Copy and make the top-level start script executable
COPY start_node.sh /app/start_node.sh
RUN chmod +x /app/start_node.sh

# --- Ensure .ros directory exists and is writable by root ---
RUN mkdir -p /app/.ros && chmod -R 777 /app/.ros

# Build the Catkin workspace (isolated build method)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    cd /app; \
    catkin_make_isolated --install"

# --- Set final permissions ---
USER root
RUN chown -R root:root /app

# Set final working directory
WORKDIR /app