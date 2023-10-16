#!/bin/bash

# Move to the Firmware directory
cd /home/yeonri/Firmware

# Source the setup file for gazebo
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

# Set the ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

# Set the QT environment variable
export QT_X11_NO_MITSHM=1

# Move to the launch directory
cd /home/yeonri/Firmware/launch

# Launch the roslaunch file
roslaunch posix_sitl2.launch

