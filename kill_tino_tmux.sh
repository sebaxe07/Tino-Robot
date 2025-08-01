#!/bin/bash

# This script kills the Tino robot tmux session

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "tmux is not installed. Please install it with: sudo apt install tmux"
    exit 1
fi

# Check if a tmux session named 'tino' exists
if tmux has-session -t tino 2>/dev/null; then
    echo "Killing existing 'tino' tmux session..."
    tmux kill-session -t tino
    echo "Tmux session terminated successfully."
else
    echo "No active 'tino' tmux session found."
fi
