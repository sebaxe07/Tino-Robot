#!/bin/sh
# launcher.sh
# navigate to home directory, then to this directory, then execute python script, then back home

# bash /home/tino/Desktop/Tino-raspberrypi/launcher.sh

# start new MAIN.PY
cd "$(dirname "$(readlink -f "$0")")"

# Activate the virtual environment
source  ~/etereum/bin/activate

# Execute the Python script
python3 main_camera.py 

# python3 classes/Stream.py & #background

#navigate to http://192.168.0.101:5000/video_feed for video streaming
