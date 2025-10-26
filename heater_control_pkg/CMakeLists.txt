cmake_minimum_required(VERSION 3.0.2)
project(heater_control_pkg)

## Find catkin and other required ROS packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## Declare Python setup (needed if you have Python modules in src/, otherwise optional but good practice)
# catkin_python_setup() # Uncomment if you plan to add modules later

## Declare Catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

## Install Python executable scripts
# This makes heater_control_node.py findable by rosrun
install(PROGRAMS
  src/heater_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Note: If you add launch files later, uncomment and use this:
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
