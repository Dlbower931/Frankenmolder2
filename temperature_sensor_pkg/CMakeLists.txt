cmake_minimum_required(VERSION 3.0.2)
project(temperature_sensor_pkg)

# Find necessary ROS packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# Standard Catkin package setup
catkin_package()

# --- CRITICAL: Install Python Scripts ---
# This block tells 'make install' where to put the executable Python scripts
# so 'rosrun' can find them in the 'install' space.
install(PROGRAMS
  src/extruder_zone1_temp_node.py
  src/extruder_zone2_temp_node.py
  src/extruder_zone3_temp_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
