cmake_minimum_required(VERSION 3.0.2)
project(temperature_sensor_pkg)

# --- Find ROS dependencies (REQUIRED) ---
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# --- Catkin Setup (REQUIRED) ---
# This line must be present for any Catkin package
catkin_package()

# --- Install Python Scripts (CRUCIAL) ---
# This line registers your Python script to be found by 'rosrun'.
# PROGRAMS is correct for python executables.
install(PROGRAMS
  src/temp_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)