#!/bin/bash

# This script is a wrapper to run rtabmap with proper environment setup
# to avoid the symbol lookup error

# Source ROS environment
source /opt/ros/humble/setup.bash
source /home/orinano/Tino-Robot/tino_ws/install/setup.bash

# Set up the LD_LIBRARY_PATH to include all necessary libraries
export LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/usr/local/lib/:/home/orinano/dev/depthai-core/build/install/lib/:$LD_LIBRARY_PATH

# Print environment for debugging
echo "Using LD_LIBRARY_PATH: $LD_LIBRARY_PATH"

# Launch rtabmap in localization mode using the fixed launch file
ros2 launch tino_ros rtab_localization_headless.launch.py
