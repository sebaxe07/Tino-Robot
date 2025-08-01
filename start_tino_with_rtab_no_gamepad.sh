#!/bin/bash

# This script launches the Tino robot with RTAB-Map localization in a tmux session
# with separate panes for better output visualization
# Left pane: RTAB-Map localization
# Right pane: Basic tino_robot (without gamepad)

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "tmux is not installed. Please install it with: sudo apt install tmux"
    exit 1
fi

# Check if a tmux session named 'tino_basic' already exists and kill it if it does
if tmux has-session -t tino_basic 2>/dev/null; then
    echo "Killing existing 'tino_basic' tmux session..."
    tmux kill-session -t tino_basic
fi

# Create a new tmux session named "tino_basic"
tmux new-session -d -s tino_basic

# Name the first window
tmux rename-window -t tino_basic:0 'Tino Control'

# Split the window horizontally
tmux split-window -h -t tino_basic:0

# In the left pane, set up environment and launch RTAB-Map localization
tmux send-keys -t tino_basic:0.0 "echo 'Setting up environment and starting RTAB-Map Headless Localization...'" C-m
tmux send-keys -t tino_basic:0.0 "source /home/orinano/Tino-Robot/tino_ws/install/setup.bash" C-m
tmux send-keys -t tino_basic:0.0 "export LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/usr/local/lib/:/home/orinano/dev/depthai-core/build/install/lib/:$LD_LIBRARY_PATH" C-m
tmux send-keys -t tino_basic:0.0 "echo 'Starting rtabmap using wrapper script...'" C-m
tmux send-keys -t tino_basic:0.0 "/home/orinano/Tino-Robot/run_rtabmap.sh" C-m

# In the right pane, set up environment and launch the basic tino robot (without gamepad)
tmux send-keys -t tino_basic:0.1 "echo 'Setting up environment and starting Tino Robot...'" C-m
tmux send-keys -t tino_basic:0.1 "source /home/orinano/Tino-Robot/tino_ws/install/setup.bash" C-m
tmux send-keys -t tino_basic:0.1 "ros2 launch tino_ros tino_robot.launch.py" C-m

# Attach to the tmux session
tmux attach-session -t tino_basic

# Note: To detach from the session without stopping it, press Ctrl+B then D
# To re-attach later: tmux attach-session -t tino_basic
