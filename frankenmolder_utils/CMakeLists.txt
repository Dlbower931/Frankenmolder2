cmake_minimum_required(VERSION 3.0.2)
project(frankenmolder_utils)

# Find necessary ROS packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# Declare this as a catkin package
catkin_package()

# Install the Python executable script
install(PROGRAMS
  src/topic_watchdog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
