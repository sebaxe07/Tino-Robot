#!/bin/bash
# Script to configure persistent device symlinks for Tino Robot
# Run this once for each device (Arduino/UWB) to identify and set up symlinks

echo "Tino Robot Persistent Device Name Setup"
echo "======================================="
echo ""
echo "This script will help you set up persistent device names for your devices."
echo "Please connect only ONE device at a time and follow the instructions."
echo ""

# Function to get device serial number
get_serial_number() {
    local dev_path=$1
    if [ -e "$dev_path" ]; then
        serial=$(udevadm info -a -n "$dev_path" | grep -m 1 '{serial}' | cut -d '"' -f 2)
        vendor=$(udevadm info -a -n "$dev_path" | grep -m 1 '{idVendor}' | cut -d '"' -f 2)
        product=$(udevadm info -a -n "$dev_path" | grep -m 1 '{idProduct}' | cut -d '"' -f 2)
        
        if [ -n "$serial" ]; then
            echo "Found device with:"
            echo "  Serial: $serial"
            echo "  Vendor ID: $vendor"
            echo "  Product ID: $product"
            return 0
        fi
    fi
    echo "No device found at $dev_path."
    return 1
}

# Function to add udev rule
add_udev_rule() {
    local serial=$1
    local vendor=$2
    local product=$3
    local device_type=$4
    local symlink=$5

    rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vendor\", ATTRS{idProduct}==\"$product\", ATTRS{serial}==\"$serial\", SYMLINK+=\"$symlink\", GROUP=\"dialout\", MODE=\"0666\""
    
    echo "Adding rule for $device_type Arduino ($symlink):"
    echo "$rule"
    
    # Check if the rule already exists
    if grep -q "$serial" /etc/udev/rules.d/99-tino-arduino.rules; then
        echo "Rule for this device already exists. Skipping."
    else
        echo "$rule" | sudo tee -a /etc/udev/rules.d/99-tino-arduino.rules > /dev/null
        echo "Rule added successfully."
    fi
}

# Main script
echo "Checking if device is connected..."

if [ -e "/dev/ttyACM0" ]; then
    dev_path="/dev/ttyACM0"
elif [ -e "/dev/ttyUSB0" ]; then
    dev_path="/dev/ttyUSB0"
else
    echo "No device found. Please connect a device and try again."
    exit 1
fi

echo "Device found at $dev_path"
echo ""

# Get serial number
get_serial_number "$dev_path"
if [ $? -ne 0 ]; then
    echo "Failed to get serial number. Please check the connection."
    exit 1
fi

echo ""
echo "Which device is this?"
echo "1) Base Arduino (ttyBASE)"
echo "2) Leg Arduino (ttyLEG)"
echo "3) Head Arduino (ttyHEAD)"
echo "4) UWB Receiver (ttyUWB)"
read -p "Select [1-4]: " choice

case $choice in
    1)
        add_udev_rule "$serial" "$vendor" "$product" "Base" "ttyBASE"
        ;;
    2)
        add_udev_rule "$serial" "$vendor" "$product" "Leg" "ttyLEG"
        ;;
    3)
        add_udev_rule "$serial" "$vendor" "$product" "Head" "ttyHEAD"
        ;;
    4)
        add_udev_rule "$serial" "$vendor" "$product" "UWB" "ttyUWB"
        ;;
    *)
        echo "Invalid selection."
        exit 1
        ;;
esac

echo ""
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "Setup complete!"
echo "After connecting all devices, you should have the following device links:"
echo "- Base Arduino: /dev/ttyBASE"
echo "- Leg Arduino:  /dev/ttyLEG"
echo "- Head Arduino: /dev/ttyHEAD"
echo "- UWB Receiver: /dev/ttyUWB"
echo ""
echo "These device names will persist across reboots."
echo ""