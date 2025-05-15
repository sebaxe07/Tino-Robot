# VR Data Recording for Unity Testing

This package includes tools to record ROS2 `/vr_in` topics and export them to a format that can be used for testing in Unity without requiring a live connection to the robot.

## Available Tools

### 1. VR Data Recorder Node

This node subscribes to all topics published to the VR interface and saves them to a database for later playback.

Topics recorded:
- `/vr_in/robot_pose` (geometry_msgs/msg/PoseWithCovarianceStamped)
- `/vr_in/human_position` (geometry_msgs/msg/PoseStamped)
- `/vr_in/human_skeleton` (visualization_msgs/msg/MarkerArray)
- `/vr_in/human_skeleton_poses` (geometry_msgs/msg/PoseArray)
- `/vr_in/audio_output` (std_msgs/msg/Int16MultiArray)

### 2. VR Data Extractor

Command-line tool to extract data from recordings and convert it to a Unity-friendly format.

## Usage Instructions

### Recording VR Data

1. Build the package:
   ```bash
   cd ~/Tino-Robot/tino_ws
   colcon build --packages-select tino_ros
   source install/setup.bash
   ```

2. Launch the recorder node:
   ```bash
   ros2 launch tino_ros vr_recorder.launch.py
   ```

3. Start/Stop recording via ROS service:
   ```bash
   # Start recording
   ros2 service call /vr_recorder/command std_msgs/msg/String "{data: start}"
   
   # Stop recording
   ros2 service call /vr_recorder/command std_msgs/msg/String "{data: stop}"
   
   # Check status
   ros2 service call /vr_recorder/command std_msgs/msg/String "{data: status}"
   ```

4. Recordings are saved to `~/vr_recordings/[timestamp]/` by default

### Extracting Data for Unity

1. List available recordings:
   ```bash
   ros2 run tino_ros vr_data_extractor --list
   ```

2. Extract data from a specific recording:
   ```bash
   ros2 run tino_ros vr_data_extractor --extract ~/vr_recordings/[session_id]
   ```

3. Filter topics to extract specific data:
   ```bash
   ros2 run tino_ros vr_data_extractor --extract ~/vr_recordings/[session_id] --topics robot_pose human_position
   ```

4. Specify custom output directory:
   ```bash
   ros2 run tino_ros vr_data_extractor --extract ~/vr_recordings/[session_id] --output ~/unity_test_data
   ```

## Configuration

You can customize the recorder node behavior with launch file arguments:

```bash
# Change recording directory
ros2 launch tino_ros vr_recorder.launch.py data_dir:=/path/to/custom/dir

# Set maximum recording time (in minutes)
ros2 launch tino_ros vr_recorder.launch.py max_recording_time:=60

# Start with recording disabled
ros2 launch tino_ros vr_recorder.launch.py recording_enabled:=false
```

## Unity Integration

The extracted data is saved in JSON format that can be imported into Unity. Each file represents a single topic and contains an array of timestamped frames with message data.

1. Copy the extracted JSON files to your Unity project's `Assets/Resources` folder
2. Use a JSON parser to load and play back the data

Example Unity C# script for loading robot pose data:
```csharp
using UnityEngine;
using System.Collections.Generic;

[System.Serializable]
public class PoseFrame
{
    public double timestamp;
    public string frame_id;
    public float[] position;
    public float[] orientation;
}

[System.Serializable]
public class PoseRecording
{
    public string topic;
    public string type;
    public List<PoseFrame> frames;
}

public class RecordingPlayer : MonoBehaviour
{
    public TextAsset robotPoseJson;
    private PoseRecording recording;
    private int currentFrame = 0;
    
    void Start()
    {
        recording = JsonUtility.FromJson<PoseRecording>(robotPoseJson.text);
        Debug.Log($"Loaded recording with {recording.frames.Count} frames");
    }
    
    void Update()
    {
        // Simple playback example
        if (currentFrame < recording.frames.Count)
        {
            PoseFrame frame = recording.frames[currentFrame];
            
            // Apply position
            transform.position = new Vector3(
                frame.position[0],
                frame.position[1],
                frame.position[2]
            );
            
            // Apply rotation
            transform.rotation = new Quaternion(
                frame.orientation[0],
                frame.orientation[1],
                frame.orientation[2],
                frame.orientation[3]
            );
            
            currentFrame++;
        }
    }
}
```

## Notes

- The generated Unity data files contain placeholder values for most fields. In a real implementation, you would need to deserialize the binary ROS messages properly.
- For audio data, you may need to convert the Int16MultiArray to a proper audio format in Unity.
- The recording format uses SQLite for storage, which allows for efficient retrieval of specific time ranges or topics.
