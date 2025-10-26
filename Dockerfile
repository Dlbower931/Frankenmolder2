# Start from a ROS Noetic base image
FROM ros:noetic-ros-core-focal

# Set up environment variables
ENV HOME /app
WORKDIR /app

# Install dependencies including GPIO
RUN apt-get update && apt-get install -y \
    ros-noetic-rosbridge-server \
    python3-pip \
    python3-dev \
    git \
    build-essential \
    python3-tk \
    tk-dev \
    python3-rpi.gpio \
    # Add sudo for group management
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Install spidev via pip
RUN pip3 install spidev

# --- Add User and Group for GPIO Access ---
# Create a non-root user 'rosuser' with ID 1000 (standard for first user like 'pi')
# Adjust UID/GID if your host user ID is different (check with 'id -u' on Pi)
ARG USER_ID=1000
ARG GROUP_ID=1000 # GID is defined but not explicitly used for rosuser groupadd
# --- FIX: Create the gpio group first ---
# --- FIX: Create rosuser group without specifying GID to avoid conflict ---
RUN groupadd gpio \
 && groupadd rosuser \
 && useradd --uid $USER_ID --gid rosuser --create-home --shell /bin/bash rosuser \
 # Add user to the gpio group (critical for hardware access)
 && usermod -aG gpio rosuser \
 && usermod -aG sudo rosuser \
 && echo 'rosuser ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to the non-root user for subsequent operations
USER rosuser
WORKDIR /home/rosuser/app
ENV HOME /home/rosuser/app

# --- Copy Code as the new user ---
# Create the source directory (owned by rosuser now)
RUN mkdir -p /home/rosuser/app/src/temperature_sensor_pkg \
             /home/rosuser/app/src/frankenmolder_gui \
             /home/rosuser/app/src/frankenmolder_utils \
             /home/rosuser/app/src/heater_control_pkg

COPY --chown=rosuser:rosuser temperature_sensor_pkg /home/rosuser/app/src/temperature_sensor_pkg
COPY --chown=rosuser:rosuser frankenmolder_gui /home/rosuser/app/src/frankenmolder_gui
COPY --chown=rosuser:rosuser frankenmolder_utils /home/rosuser/app/src/frankenmolder_utils
COPY --chown=rosuser:rosuser heater_control_pkg /home/rosuser/app/src/heater_control_pkg

# --- Make Python nodes executable ---
RUN chmod +x /home/rosuser/app/src/frankenmolder_gui/src/extruder_gui_node.py
RUN chmod +x /home/rosuser/app/src/temperature_sensor_pkg/src/extruder_zone1_temp_node.py
RUN chmod +x /home/rosuser/app/src/temperature_sensor_pkg/src/extruder_zone2_temp_node.py
RUN chmod +x /home/rosuser/app/src/heater_control_pkg/src/heater_control_node.py
RUN chmod +x /home/rosuser/app/src/frankenmolder_utils/src/topic_watchdog.py

# --- Copy start script ---
COPY --chown=rosuser:rosuser start_node.sh /home/rosuser/app/start_node.sh
RUN chmod +x /home/rosuser/app/start_node.sh

# --- Build the Catkin workspace (as rosuser) ---
# Need to source ROS setup again as rosuser
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    cd /home/rosuser/app; \
    catkin_make_isolated"

# --- Update Environment for rosuser ---
# Add sourcing to the user's bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/rosuser/.bashrc
RUN echo "source /home/rosuser/app/devel_isolated/setup.bash" >> /home/rosuser/.bashrc

# Set final working directory
WORKDIR /home/rosuser/app

# Command is still specified in docker-compose.yml, will run as rosuser