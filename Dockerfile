# --- Start from the ROS image we KNOW works ---
FROM ros:noetic-ros-core-focal

# Set up environment variables
ENV HOME /app
WORKDIR /app

# 1. Install all dependencies (CAN + GUI + Bridge + Build Tools)
RUN apt-get update && apt-get install -y \
    # ROS dependencies
    ros-noetic-rosbridge-server \
    # Python tools
    python3-pip \
    python3-dev \
    # Build tools
    git \
    build-essential \
    # GUI dependencies
    python3-tk \
    tk-dev \
    # NEW CAN dependencies
    can-utils \
    python3-can \
    # System tools (like sudo)
    sudo \
    && rm -rf /var/lib/apt/lists/*

# (No more pip install for spidev)

# 2. Copy all NEW source code
RUN mkdir -p /app/src/frankenmolder_gui \
             /app/src/frankenmolder_utils \
             /app/src/can_bridge_pkg
COPY frankenmolder_gui /app/src/frankenmolder_gui
COPY frankenmolder_utils /app/src/frankenmolder_utils
COPY can_bridge_pkg /app/src/can_bridge_pkg
# (Old sensor/heater/pico packages are not copied)

# 3. Copy the startup script
COPY start_node.sh /app/start_node.sh
RUN chmod +x /app/start_node.sh

# 4. Make Python nodes executable
RUN chmod +x /app/src/frankenmolder_gui/src/extruder_gui_node.py
RUN chmod +x /app/src/frankenmolder_utils/src/topic_watchdog.py
RUN chmod +x /app/src/can_bridge_pkg/src/can_bridge_node.py

# 5. Fix .ros permission issue from before
RUN mkdir -p /app/.ros && chmod -R 777 /app/.ros

# 6. Build the Catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    cd /app; \
    catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release"

# 7. Set up final environment
RUN echo "source /app/install_isolated/setup.bash" >> ~/.bashrc
