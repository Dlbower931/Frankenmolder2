cmake_minimum_required(VERSION 3.0.2)
project(temperature_sensor_pkg)

# Find necessary packages for Catkin build
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# Set up the Catkin build environment
# This is the standard package macro, NOT catkin_workspace()
catkin_package()

# Install the Python scripts so they can be run with 'rosrun'
install(PROGRAMS
  src/temp_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)