# Using the VR Data Recording System

This guide explains how to use the VR data recording system to record and extract VR data for Unity testing.

## Quick Start

### 1. Start the Recording Node

```bash
# Start with default settings (recording enabled)
ros2 run tino_ros vr_data_recorder_node

# Or using the launch file with custom settings
ros2 launch tino_ros vr_recorder.launch.py recording_enabled:=true data_dir:=~/vr_recordings max_recording_time:=60
```

### 2. Control Recording with Services

```bash
# Start recording 
ros2 service call /vr_data_recorder_node/start std_srvs/srv/Trigger

# Stop recording
ros2 service call /vr_data_recorder_node/stop std_srvs/srv/Trigger

# Check recording status
ros2 service call /vr_data_recorder_node/status std_srvs/srv/Trigger
```

### 3. Extract Data for Unity

```bash
# List available recording sessions
ros2 run tino_ros vr_data_extractor --list

# Extract a specific recording session
ros2 run tino_ros vr_data_extractor --extract ~/vr_recordings/20250515_123456

# Extract only certain topics
ros2 run tino_ros vr_data_extractor --extract ~/vr_recordings/20250515_123456 --topics robot_pose human_position

# Specify a custom output directory
ros2 run tino_ros vr_data_extractor --extract ~/vr_recordings/20250515_123456 --output ~/unity_test_data
```

## Recording Details

The recorder node subscribes to the following topics:
- `/vr_in/robot_pose` (PoseWithCovarianceStamped)
- `/vr_in/human_position` (PoseStamped)
- `/vr_in/human_skeleton` (MarkerArray)
- `/vr_in/human_skeleton_poses` (PoseArray)
- `/vr_in/audio_output` (Int16MultiArray)

Data is saved in a SQLite database with serialized ROS messages, along with metadata in JSON format.

## Extraction Details

The extractor tool deserializes the recorded messages and converts them to a Unity-friendly JSON format.

Each extracted topic generates a JSON file with:
- Topic information
- Message type
- Array of timestamped frames with actual message data

## Troubleshooting

### Services Not Found
If you see errors like "service not found", make sure to use the full service name:
```bash
ros2 service call /vr_data_recorder_node/start std_srvs/srv/Trigger
```

### Zero Values in Extracted Data
If you see zeros instead of actual values in extracted data, check:
1. That data was actually recorded (check message counts in metadata.json)
2. That the ROS message types match between recording and extraction

### Ctrl+C Not Working Properly
If Ctrl+C doesn't properly stop the recording:
1. Use the service call method to stop recording first
2. Then press Ctrl+C to exit the node

## Advanced Usage

### Recording Data Selectively
You can modify the recorder node to only record specific topics if needed by editing the `message_counts` initialization in the `init_recording` method.

### Customizing the JSON Format
The extraction format can be customized by modifying the extractor tool's conversion logic for specific message types.
