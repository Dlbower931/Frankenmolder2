cmake_minimum_required(VERSION 3.0.2)
project(pico_comm_pkg)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
