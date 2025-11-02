cmake_minimum_required(VERSION 3.0.2)
project(frankenmolder_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# Install Python scripts
install(PROGRAMS
  src/extruder_gui_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)