# Top-level workspace file
cmake_minimum_required(VERSION 3.0.2)
project(catkin_workspace)

# Find catkin and other packages
find_package(catkin REQUIRED)

# Tell catkin where to find packages
# It looks for subdirectories containing 'package.xml'
# 'src' is the standard directory name
catkin_workspace()

# Uncomment the following to restrict your build to a subset of packages
# add_subdirectory(src/your_package_name)