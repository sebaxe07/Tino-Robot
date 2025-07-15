#!/bin/bash

# Script to upload the legacy leg_tino Arduino sketch

echo "Compiling and uploading legacy leg_tino sketch..."

# Install required libraries (if not already installed)
arduino-cli lib install --git-url https://github.com/arkhipenko/ezButton
arduino-cli lib install "Cytron Motor Drivers Library"

# Reset the Arduino to ensure clean upload
echo "Resetting Arduino serial port..."
stty -F /dev/ttyLEG 1200
sleep 2
stty -F /dev/ttyLEG 115200

# Compile and upload the sketch
echo "Compiling sketch..."
arduino-cli compile --fqbn arduino:avr:uno /home/orinano/Tino-Robot/legacy_tino/leg_tino/

if [ $? -eq 0 ]; then
    echo "Compilation successful! Attempting upload..."
    echo "Please press the RESET button on your Elegoo UNO R3 now!"
    echo "Starting upload in 3... 2... 1..."
    sleep 3
    
    arduino-cli upload -p /dev/ttyLEG --fqbn arduino:avr:uno /home/orinano/Tino-Robot/legacy_tino/leg_tino/
    
    if [ $? -eq 0 ]; then
        echo "Upload successful!"
    else
        echo "Upload failed!"
        exit 1
    fi
else
    echo "Compilation failed!"
    exit 1
fi
