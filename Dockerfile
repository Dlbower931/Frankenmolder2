# Start from a ROS Noetic base image
FROM ros:noetic-ros-core-focal

# Set up environment variables
ENV HOME /app
WORKDIR /app

# Install dependencies including GPIO and sudo (sudo might still be useful for debugging)
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
    && rm -rf /var/lib/apt/lists/*

# Install spidev via pip
RUN pip3 install spidev

# --- Copy Code as root ---
# Create the source directory
RUN mkdir -p /app/src/temperature_sensor_pkg \
             /app/src/frankenmolder_gui \
             /app/src/frankenmolder_utils \
             /app/src/heater_control_pkg

# Copy packages (owner will be root)
COPY temperature_sensor_pkg /app/src/temperature_sensor_pkg
COPY frankenmolder_gui /app/src/frankenmolder_gui
COPY frankenmolder_utils /app/src/frankenmolder_utils
COPY heater_control_pkg /app/src/heater_control_pkg

# --- Make Python nodes executable (using /app paths) ---
RUN chmod +x /app/src/frankenmolder_gui/src/extruder_gui_node.py
RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone1_temp_node.py
RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone2_temp_node.py
RUN chmod +x /app/src/heater_control_pkg/src/heater_control_node.py
RUN chmod +x /app/src/frankenmolder_utils/src/topic_watchdog.py

# --- Copy start script ---
COPY start_node.sh /app/start_node.sh
RUN chmod +x /app/start_node.sh

# --- Build the Catkin workspace (as root) ---
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    cd /app; \
    catkin_make_isolated"

# --- Update Environment for root (Optional but good practice) ---
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /app/devel_isolated/setup.bash" >> ~/.bashrc

# Set final working directory
WORKDIR /app

# Command is still specified in docker-compose.yml, will run as root

