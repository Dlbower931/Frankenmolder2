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
       # Add sudo (needed for groupadd check)
       sudo \
       && rm -rf /var/lib/apt/lists/*

   # Install spidev and pyserial via pip
   RUN pip3 install spidev pyserial

   # --- FIX: Create device groups if they don't exist and add root ---
   # Check if groups exist before adding, add root to them
   RUN groupadd --force gpio || true && usermod -aG gpio root \
    && groupadd --force spi || true && usermod -aG spi root \
    && groupadd --force dialout || true && usermod -aG dialout root

   # Create the source directory and copy local code (as root)
   RUN mkdir -p /app/src/temperature_sensor_pkg /app/src/frankenmolder_gui /app/src/frankenmolder_utils /app/src/heater_control_pkg /app/src/pico_comm_pkg
   COPY temperature_sensor_pkg /app/src/temperature_sensor_pkg
   COPY frankenmolder_gui /app/src/frankenmolder_gui
   COPY frankenmolder_utils /app/src/frankenmolder_utils
   COPY heater_control_pkg /app/src/heater_control_pkg
   COPY pico_comm_pkg /app/src/pico_comm_pkg

   # --- Make Python nodes executable ---
   RUN chmod +x /app/src/frankenmolder_gui/src/extruder_gui_node.py
   RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone1_temp_node.py
   RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone2_temp_node.py
   RUN chmod +x /app/src/heater_control_pkg/src/heater_control_node.py
   RUN chmod +x /app/src/frankenmolder_utils/src/topic_watchdog.py
   RUN chmod +x /app/src/pico_comm_pkg/src/pico_serial_node.py

   # --- Copy start script ---
   COPY start_node.sh /app/start_node.sh
   RUN chmod +x /app/start_node.sh

   # --- Ensure .ros directory exists with open permissions ---
   RUN mkdir -p /app/.ros && chmod -R 777 /app/.ros

   # --- Build the Catkin workspace (as root) ---
   RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
       cd /app; \
       catkin_make_isolated"

   # --- Add Sourcing to Root's bashrc (Optional, for debugging) ---
   RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   RUN echo "source /app/devel_isolated/setup.bash" >> ~/.bashrc

   # Set final working directory
   WORKDIR /app

   # Command is specified in docker-compose.yml
   