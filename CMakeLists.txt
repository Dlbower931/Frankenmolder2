# Top-level workspace file - Required for catkin_make/catkin_make_isolated
cmake_minimum_required(VERSION 3.0.2)
project(catkin_workspace)

# Find catkin and other packages
# This is sufficient to tell catkin_make_isolated where to look (in 'src')
find_package(catkin REQUIRED)

# Include all packages in the workspace
catkin_workspace()