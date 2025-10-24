# Start from the official ROS Noetic image
FROM ros:noetic-ros-core-focal

# Set the working directory inside the container
WORKDIR /app

# Install necessary Python tools, dependencies, and core ROS packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    libpython3-dev \
    git \
    build-essential \
    # Install ROS communication packages (rospy, std_msgs)
    ros-noetic-ros-comm \
    && rm -rf /var/lib/apt/lists/*

# Install the Python spidev library for SPI communication
RUN pip3 install spidev

# --- Prepare Catkin Workspace Structure and Copy Files ---

# 1. Create the src directory
RUN mkdir -p /app/src/

# 2. Copy the ROS package folder into the src directory
COPY ./temperature_sensor_pkg /app/src/temperature_sensor_pkg

# 3. Copy the REQUIRED top-level workspace CMakeLists.txt
COPY CMakeLists.txt /app/temperature_sensor_pkg/CMakeLists.txt

# 4. Copy the startup script and make it executable (THIS IS THE MISSING PART)
COPY start_node.sh /app/start_node.sh
RUN chmod +x /app/start_node.sh

# --- Build the ROS Workspace ---
# This step creates the 'devel' directory and makes your package runnable
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    cd /app; \
    /opt/ros/noetic/bin/catkin_make"

# Add the ROS setup source to the bashrc for convenience
RUN echo "source /app/devel/setup.bash" >> ~/.bashrc
# CMD is defined in the docker-compose.yml file.