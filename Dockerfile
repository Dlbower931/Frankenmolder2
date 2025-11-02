# --- Start from the ROS image we KNOW works ---
FROM ros:noetic-ros-core-focal

# Set up environment variables
ENV HOME /app
WORKDIR /app

# 1. Install all dependencies (CAN + Build Tools)
RUN apt-get update && apt-get install -y \
    # ROS dependencies (rosbridge is optional if you use ROS1 connection)
    ros-noetic-rosbridge-server \
    # Python tools
    python3-pip \
    python3-dev \
    # Build tools
    git \
    build-essential \
    # NEW CAN dependencies
    can-utils \
    python3-can \
    # System tools (like sudo)
    sudo \
    && rm -rf /var/lib/apt/lists/*
# (No more pip install for spidev)
# (Removed python3-tk, tk-dev)

# 2. Copy all NEW source code
RUN mkdir -p /app/src/can_bridge_pkg
COPY can_bridge_pkg /app/src/can_bridge_pkg
# (Removed frankenmolder_gui and frankenmolder_utils)

# 3. Copy the startup script (Assuming you still have a start_node.sh)
COPY start_node.sh /app/start_node.sh
RUN chmod +x /app/start_node.sh

# 4. Make Python nodes executable
RUN chmod +x /app/src/can_bridge_pkg/src/can_bridge_node.py
# (Removed chmod for gui and watchdog nodes)

# 5. Fix .ros permission issue from before
RUN mkdir -p /app/.ros && chmod -R 777 /app/.ros

# 6. Build the Catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    cd /app; \
    catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release"

# --- FIX: Add execute permission to the generated setup file ---
RUN chmod +x /app/install_isolated/setup.bash
# -----------------------------------------------------------

# 7. Set up final environment
RUN echo "source /app/install_isolated/setup.bash" >> ~/.bashrc
