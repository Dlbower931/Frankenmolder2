# Start from the official ROS Noetic image (recommended for Raspberry Pi 4/newer)
FROM ros:noetic-ros-core-focal

# Set the working directory inside the container
WORKDIR /app

# Install necessary Python tools and dependencies
# 'python3-pip' and 'libpython3-dev' are needed for spidev
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    libpython3-dev \
    git \
    build-essential \
    # Clean up the apt cache to keep the image small
    && rm -rf /var/lib/apt/lists/*

# Install the Python spidev library for SPI communication
# NOTE: We have removed the unsupported '--break-system-packages' option.
RUN pip3 install \
    spidev

# Copy your ROS package source code into the container
# This assumes your package folder is next to the Dockerfile
COPY ./temperature_sensor_pkg /app/src/temperature_sensor_pkg

# Build the ROS workspace (using catkin_make)
# We execute this inside a bash shell that has sourced the ROS environment
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    cd /app; \
    /opt/ros/noetic/bin/catkin_make"

# Add the ROS setup source to the bashrc for convenience
RUN echo "source /app/devel/setup.bash" >> ~/.bashrc

# Set the entrypoint to the wrapper script (defined below)
ENTRYPOINT ["/bin/bash", "/app/start_node.sh"]