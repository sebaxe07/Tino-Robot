#!/bin/bash
# Script to compile and upload the head_OL_tino_base sketch to Elegoo UNO R3

SOURCE_DIR="/home/orinano/Tino-Robot/new_head_tino"
PORT="/dev/ttyHEAD"
BOARD="arduino:avr:uno"

echo "Compiling and uploading head_OL_tino_base sketch..."

# Make sure the Arduino core and libraries are installed
arduino-cli core install arduino:avr
arduino-cli lib install Servo

# Reset the serial port before uploading
echo "Resetting Arduino serial port..."
stty -F $PORT hupcl
sleep 2

# Compile the sketch
echo "Compiling sketch..."
arduino-cli compile --fqbn $BOARD "$SOURCE_DIR" -v

if [ $? -eq 0 ]; then
    echo "Compilation successful! Attempting upload..."
    
    echo "Please press the RESET button on your Elegoo UNO R3 now!"
    sleep 2
    echo "Starting upload in 3... 2... 1..."
    sleep 1

    # The -v flag provides verbose output to help diagnose issues
    arduino-cli upload -p $PORT --fqbn $BOARD "$SOURCE_DIR" -v --verify

    if [ $? -eq 0 ]; then
        echo "Upload successful!"
    else
        echo "Upload failed. Trying alternative approach..."
        
        # Try avrdude directly with more relaxed timing
        echo "Trying direct avrdude upload..."
        avrdude -p atmega328p -c arduino -P $PORT -b 115200 -D -U flash:w:"$SOURCE_DIR/build/arduino.avr.uno/head_OL_tino_base.ino.hex":i
        
        if [ $? -eq 0 ]; then
            echo "Direct upload successful!"
        else
            echo "All upload attempts failed."
            echo "Try these troubleshooting steps:"
            echo "1. Make sure the Elegoo UNO R3 is properly connected"
            echo "2. Press the reset button right as upload starts"
            echo "3. Try a different USB cable or port"
            echo "4. Check permissions with: ls -l $PORT"
        fi
    fi
else
    echo "Compilation failed."
    exit 1
fi