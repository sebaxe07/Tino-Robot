#!/bin/bash
# Script to set up audio devices correctly

echo "Setting up audio devices..."

# Make sure ALSA uses the USB audio device (card 0)
if [ ! -f /etc/asound.conf ]; then
    echo "Creating ALSA configuration file..."
    echo '# Set USB audio device as default
defaults.pcm.card 0
defaults.pcm.device 0
defaults.ctl.card 0' | sudo tee /etc/asound.conf
    echo "ALSA configuration created."
else
    echo "ALSA configuration already exists."
fi

# If PulseAudio is running, configure it as well
if pgrep pulseaudio > /dev/null; then
    echo "PulseAudio is running, setting default devices..."
    
    # Set default output device (speaker)
    pacmd set-default-sink $(pacmd list-sinks | grep -B 1 "name:.*usb" | grep index | awk '{print $2}')
    
    # Set default input device (microphone)
    pacmd set-default-source $(pacmd list-sources | grep -B 1 "name:.*usb" | grep index | awk '{print $2}')
    
    echo "PulseAudio configuration complete."
else
    echo "PulseAudio is not running, using ALSA only."
fi

# List current audio devices for verification
echo "Current audio playback devices:"
aplay -l
echo "Current audio recording devices:"
arecord -l

echo "Audio setup complete!"