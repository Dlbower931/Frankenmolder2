cmake_minimum_required(VERSION 3.0.2)
project(can_bridge_pkg)

# Find necessary ROS packages (rospy for python, std_msgs for common messages)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  can_msgs
)

# This is required for all catkin packages
catkin_package()

# --- Install Python Node ---
# This command tells the build system to copy your Python script
# into the final 'install' directory (under lib/package_name/)
# so that 'rosrun' can find and execute it.
install(PROGRAMS
  src/can_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

