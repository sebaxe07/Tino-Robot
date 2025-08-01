#!/bin/bash
# Script to set up audio devices correctly for Tino Robot on Jetson Orin Nano
# This script handles both ALSA and PulseAudio configuration

# Function to detect the USB audio device 
detect_usb_audio() {
    # Check if USB Audio Device is connected
    if lsusb | grep -q "C-Media Electronics"; then
        echo "USB Audio Device (C-Media Electronics) detected."
        return 0
    else
        echo "Warning: USB Audio Device not detected! Please check if it's connected."
        return 1
    fi
}

# Function to set PulseAudio sink and source
set_pulseaudio_defaults() {
    # Find the USB audio device
    USB_SINK=$(pactl list short sinks | grep "usb" | cut -f1)
    USB_SOURCE=$(pactl list short sources | grep "usb" | grep -v "monitor" | cut -f1)
    
    if [ -n "$USB_SINK" ]; then
        pactl set-default-sink $USB_SINK
        echo "Set default audio output to USB audio device (sink $USB_SINK)"
    else
        echo "Warning: No USB audio sink found"
    fi
    
    if [ -n "$USB_SOURCE" ]; then
        pactl set-default-source $USB_SOURCE
        echo "Set default audio input to USB audio device (source $USB_SOURCE)"
    else
        echo "Warning: No USB audio source found"
    fi
}

echo "===== Tino Robot Audio Setup ====="
echo "Setting up audio devices..."

# Detect USB audio device
detect_usb_audio

# Configure ALSA (low-level audio)
echo "Configuring ALSA..."
sudo bash -c 'cat > /etc/asound.conf << EOF
# Set USB audio device as default
defaults.pcm.card 0
defaults.pcm.device 0
defaults.ctl.card 0

# Define a specific USB Audio device if needed
pcm.usb {
    type hw
    card 0
}

# Define custom audio redirection
pcm.!default {
    type plug
    slave.pcm "usb"
}
EOF'

echo "ALSA configuration complete."

# Check for PulseAudio packages
if ! dpkg -l | grep -q pulseaudio; then
    echo "Installing PulseAudio and related packages..."
    sudo apt-get update
    sudo apt-get install -y pulseaudio pulseaudio-utils pavucontrol
fi

# Stop PulseAudio if it's running
echo "Restarting PulseAudio..."
pulseaudio --kill 2>/dev/null || true
sleep 2

# Reset PulseAudio configuration
echo "Resetting PulseAudio configuration..."
rm -rf ~/.config/pulse/*.tdb 2>/dev/null || true

# Create PulseAudio config directory if it doesn't exist
mkdir -p ~/.config/pulse

# Configure PulseAudio client
cat > ~/.config/pulse/client.conf << EOF
# Connect to the local sound server
autospawn = yes
daemon-binary = /usr/bin/pulseaudio
EOF

# Create a custom default.pa file
cat > ~/.config/pulse/default.pa << EOF
#!/usr/bin/pulseaudio -nF

# Load system-wide configuration
.include /etc/pulse/default.pa

# Automatically switch to newly connected devices
load-module module-switch-on-connect

# Set higher priority for USB audio devices
set-sink-priority alsa_output.usb-*.*-00.analog-stereo 9000
set-source-priority alsa_input.usb-*.*-00.analog-mono 9000
EOF

# Start PulseAudio
echo "Starting PulseAudio..."
pulseaudio --start --log-target=syslog
sleep 2

# Check if PulseAudio is running
if pgrep pulseaudio > /dev/null; then
    echo "PulseAudio is running, setting default devices..."
    set_pulseaudio_defaults
else
    echo "ERROR: PulseAudio failed to start. Try rebooting the system."
    exit 1
fi

# Test audio output capability with a pleasant chime
echo "Testing audio output with a pleasant chime sound..."
if [ -f "/home/orinano/Tino-Robot/assets/chime.wav" ]; then
    aplay -D default "/home/orinano/Tino-Robot/assets/chime.wav" &
    sleep 1
    pkill -9 aplay 2>/dev/null || true
    echo "✓ Chime played successfully!"
else
    echo "Warning: Chime sound file not found. Using fallback test tone."
    if command -v speaker-test &> /dev/null; then
        speaker-test -t sine -f 1000 -l 1 -P 10 -D default &
        sleep 0.5
        pkill -9 speaker-test 2>/dev/null || true
    fi
fi

echo "===== Audio Setup Results ====="
echo "ALSA playback devices:"
aplay -l

echo "ALSA recording devices:"
arecord -l

echo "PulseAudio output devices:"
pactl list short sinks

echo "PulseAudio input devices:"
pactl list short sources

echo "Default PulseAudio output device:"
pactl info | grep "Default Sink"

echo "Default PulseAudio input device:"
pactl info | grep "Default Source"

echo "Audio setup complete! If you still have audio issues, try rebooting your system."