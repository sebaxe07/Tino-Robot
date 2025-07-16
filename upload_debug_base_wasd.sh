#!/bin/bash

echo "=== TINO DEBUG BASE WASD UPLOADER ==="
echo "Compiling and uploading debug_base_wasd sketch..."

# Compile the sketch
echo "Compiling..."
if arduino-cli compile -b arduino:avr:mega debug_base_wasd; then
    echo "Compilation successful!"
else
    echo "Compilation failed! Check your code for errors."
    exit 1
fi

# Upload to the base Arduino
echo "Uploading to base Arduino..."
if arduino-cli upload -p /dev/ttyBASE -b arduino:avr:mega debug_base_wasd; then
    echo "Upload successful!"
else
    echo "Upload failed! Check connection and port."
    exit 1
fi

echo "Upload complete!"
echo ""
echo "=== OPENING SERIAL MONITOR ==="
echo "WASD Controls (HOLD keys, no Enter needed):"
echo "  W - Move Forward"
echo "  S - Move Backward"
echo "  A - Turn Left" 
echo "  D - Turn Right"
echo "  X - Stop"
echo ""
echo "Press Ctrl+A then K to exit, or Ctrl+C"
echo "================================="

# Wait a moment for the Arduino to reset after upload
sleep 2

# Check if screen is available (better for real-time input)
if command -v screen &> /dev/null; then
    echo "Using 'screen' for real-time control..."
    echo "To exit: Press Ctrl+A, then K, then Y"
    sleep 1
    screen /dev/ttyBASE 115200
elif command -v minicom &> /dev/null; then
    echo "Using 'minicom' for real-time control..."
    echo "To exit: Press Ctrl+A, then X"
    sleep 1
    minicom -b 115200 -D /dev/ttyBASE
else
    echo "Using arduino-cli monitor (requires Enter after each command)..."
    echo "For better control, install 'screen': sudo apt install screen"
    arduino-cli monitor -p /dev/ttyBASE -c baudrate=115200
fi
