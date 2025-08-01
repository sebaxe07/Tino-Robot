#!/bin/bash
# filepath: /home/orinano/Tino-Robot/build_workspace.sh

echo "Building ROS 2 workspace..."
cd /home/orinano/Tino-Robot/tino_ws
colcon build --symlink-install

echo "Done! Source the workspace to make the new launch files available:"
echo "source /home/orinano/Tino-Robot/tino_ws/install/setup.bash"
