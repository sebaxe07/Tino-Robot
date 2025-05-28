# Tino Robot - Control System

A comprehensive control system for Tino, a multi-component robot featuring a mobile base, articulated head, and leg mechanism.

## Project Overview

Tino Robot has evolved from a Raspberry Pi-controlled system to a modern ROS2-based implementation:

### Current Implementation

The current version of Tino Robot is built with a modular architecture running on a Jetson Orin Nano:

- **Base**: Differential drive system with 2 wheels and a caster wheel
- **Head**: Three-servo platform providing pan and tilt motion
- **Leg**: Single-actuator mechanism
- **Vision & Audio**: Integrated camera and audio for SLAM and human interaction

The system provides real-time control via gamepad and includes advanced features like SLAM, human pose detection, and audio communication.

### Legacy Implementation

The original version of Tino Robot consists of three main Arduino-controlled components that communicate with a central Raspberry Pi:

- **Base**: Holonomic movement system with omni-directional wheels
- **Head**: Three-servo platform providing pan and tilt motion
- **Leg**: Single-actuator mechanism with encoded motion

The legacy system provides real-time control via gamepad and includes video streaming capabilities through the Raspberry Pi camera.

## System Architecture

### Current Hardware Components

- **3× Arduino boards** - controlling base, head, and leg mechanisms
- **Jetson Orin Nano** - central controller running ROS2 Humble
- **Motors and servos** - actuators for movement
- **Luxonis Oak-D Pro Camera** - for visual SLAM with RTABMAP and pose detection
- **Microphone and speaker** - for audio communication
- **Gamepad** - user input device (Logitech Cordless RumblePad 2)

### Current Software Components

- **Arduino firmware** - motor control for robot components
- **ROS2 Humble nodes** - distributed system for control, perception, and communication
- **RTABMAP** - for Simultaneous Localization and Mapping (SLAM)
- **DepthAI** - for Oak-D Pro camera integration

### Legacy Hardware Components

- **3× Arduino boards** - controlling base, head, and leg mechanisms
- **Raspberry Pi** - central controller and video streaming server
- **Motors and servos** - actuators for movement
- **Encoders** - position feedback for closed-loop control
- **Camera** - for video streaming
- **Gamepad** - user input device (Logitech Cordless RumblePad 2)

### Legacy Software Components

- **Arduino firmware** - PID control loops and motor control
- **Python controllers** - multiprocess serial communication and gamepad integration
- **Flask server** - live video streaming

## Directory Structure

The project is divided into two main parts:

```
legacy_tino/               # Legacy Arduino and Raspberry Pi code
  ├── base_tino/           # Arduino code for the base module
  ├── head_OL_tino_base/   # Arduino code for the head module
  ├── leg_tino/            # Arduino code for the leg module
  └── Tino-raspberrypi/    # Original Raspberry Pi control software

tino_ws/                   # Modern ROS2-based implementation
  └── src/
      └── tino_ros/        # ROS2 package for Tino robot
          ├── launch/      # Launch files for different system configurations
          ├── resource/    # Resource files (models, etc.)
          └── tino_ros/    # Python implementation of ROS2 nodes
```

## ROS2 Node Architecture

The Tino ROS2 system consists of several interconnected nodes:

- **robot_controller_node**: Central controller that coordinates all components
- **hardware_interface_node**: Communicates with Arduino boards via serial
- **gamepad_node**: Processes gamepad inputs for robot control
- **pose_detection_node**: Human detection and skeleton tracking
- **audio_node**: Handles audio input/output
- **vr_interface_node**: Interface for potential VR integration

### Key Topics and Messages

- **/base_cmd_vel** (Twist): Commands for base movement
- **/head_cmd** (Twist): Commands for head movement
- **/human_position** (PoseStamped): Detected human position
- **/human_skeleton** (MarkerArray): Visualization of detected human skeleton
- **/localization_pose** (PoseWithCovarianceStamped): Robot's position from SLAM
- **/audio/mic_input** and **/audio/vr_output** (Int16MultiArray): Audio data streams

## Modern Setup Instructions

### Hardware Setup

1. Upload the appropriate sketch to each Arduino board:

   - `legacy_tino/base_tino/base_tino.ino` to the base Arduino
   - `legacy_tino/head_OL_tino_base/head_OL_tino_base.ino` to the head Arduino
   - `legacy_tino/leg_tino/leg_tino.ino` to the leg Arduino

2. Connect each Arduino to the Jetson Orin Nano via USB

3. Connect the Oak-D Pro camera to the Jetson Orin Nano

4. Connect microphone and speaker to the Jetson Orin Nano

### Software Setup

1. Install ROS2 Humble on the Jetson Orin Nano:

   ```bash
   # Follow ROS2 Humble installation instructions for Ubuntu 20.04
   ```

2. Install required ROS2 packages:

   ```bash
   sudo apt install ros-humble-rtabmap-ros ros-humble-depthai-ros
   ```

3. Create a symlink for the tino_ros package:

   ```bash
   cd ~/tino_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Install Python dependencies:
   ```bash
   pip install pyaudio numpy ultralytics evdev
   ```

### Modern Usage Instructions

1. Connect all batteries and turn on the system:

   - Battery for wheels and leg
   - Battery for head
   - Jetson Orin Nano power supply

2. Launch the ROS2 system:

   ```bash
   # Full system with SLAM and pose detection
   ros2 launch tino_ros tino_robot.launch.py

   # For SLAM mapping mode
   ros2 launch tino_ros rtab_mapping.launch.py

   # For SLAM localization mode
   ros2 launch tino_ros rtab_localization.launch.py
   ```

3. Turn on the gamepad by pressing the central button

4. Control the robot using the gamepad:

   - Left stick: Base forward movement
   - Bumper buttons: Base rotation
   - Right stick: Head pan/tilt

5. To stop the system, press Ctrl+C in the terminal

## Legacy Setup Instructions

### Arduino Setup

1. Upload the appropriate sketch to each Arduino board:

   - `legacy_tino/base_tino/base_tino.ino` to the base Arduino
   - `legacy_tino/head_OL_tino_base/head_OL_tino_base.ino` to the head Arduino
   - `legacy_tino/leg_tino/leg_tino.ino` to the leg Arduino

2. Connect each Arduino to the Raspberry Pi via USB

### Raspberry Pi Setup

1. Ensure Python environment is set up:

   ```
   source ~/etereum/bin/activate
   ```

2. Install required Python packages:

   - `serial`
   - `evdev`
   - `picamera2`
   - `flask`

3. Make launcher script executable:

   ```
   chmod +x launcher.sh
   ```

4. Connect the gamepad and camera to the Raspberry Pi

## Legacy Usage Instructions

1. Check the presence of the 4 batteries:

   - 1 for wheels and leg
   - 1 for head
   - 1 for the Raspberry Pi (Powerbank)
   - 1 for the Router (Powerbank)

2. Unplug all of the blue cables from the Raspberry Pi.

3. Connect all of the batteries and turn on all of the switches.

4. Connect via VSCode to the Raspberry Pi in SSH mode.

   4.1. Probably it will be usefull to download vscode in local via internet, then connet a cable ethernet al WAN del router [Do not disconnect any cable from the Raspberry Pi to the router] and reconnect via SSH.

5. Reconnect the blue USB cables to the Raspberry Pi.

6. Press the central button of the joystick to turn it on.

7. Run the launcher

8. Connet to the camera that is streaming with the instructions of the launcher.

9. To stop the execution ctrl + c in the terminal x2-3 times.

# SSH Connection

0. Connect to the network "reTino" password "earthbound"

1. Open VSCode

2. Connect to the remote SSH

   ```
   tino@192.168.0.101
   password: tino
   ```

3. Navigate to desktop -> Tino-raspberrypi

4. Write in the terminal the information of the launcher

   ```
   bash/home/Tine/Desktop/Tino-raspberry/launcher.sh
   ```

5. Done!

6. To view the camera stream, open a browser and go to:

   ```
   http://192.168.0.101:5000/video-feed
   ```

7. Control the robot.

## Communication Protocol

The Arduino modules and controllers communicate using a simple serial protocol with key-value pairs:

- `BF`: Base Forward motion
- `BB`: Base angular (rotation) motion
- `HP`: Head Pitch
- `HX`: Head X-axis (pan)
- `HY`: Head Y-axis (tilt)

## Notes

- The watchdog timer is set to 100000ms for safety
- Different controllers available in the `classes` directory allow for specialized control configurations
- Serial baudrate is set to 115200 for all devices

## Software Dependencies

### Modern System

- ROS2 Humble
- rtabmap_ros
- depthai_ros
- pyaudio
- numpy
- ultralytics (for pose detection)
- evdev (for gamepad input)
- PySerial
- Arduino libraries:
  - CytronMotorDriver
  - ezButton

### Legacy System

- Python 3
- picamera2
- Flask
- evdev
- pyserial
- ViRHas (Arduino library)
- Encoder (Arduino library)
- CytronMotorDriver (Arduino library)
- ezButton (Arduino library)

## VR Interface Message Structure

For developers integrating with the Unity VR system, here are the message structures you'll need to understand:

### Messages to Send from Unity to ROS2

1. **Robot Movement Commands** (`/vr_out/cmd_vel`, geometry_msgs/msg/Twist)
   - Controls the robot's base movement
   - Message structure:
   ```
   geometry_msgs/msg/Twist:
     Vector3 linear
       float64 x    # Forward/backward movement (-15.0 to 15.0)
       float64 y    # Not used for differential drive
       float64 z    # Not used
     Vector3 angular
       float64 x    # Not used
       float64 y    # Not used
       float64 z    # Rotation (-1.1 to 1.1, positive: counter-clockwise)
   ```

2. **Robot Head Commands** (`/vr_out/head_cmd`, geometry_msgs/msg/Twist)
   - Controls the robot's head pan and tilt movements
   - Message structure:
   ```
   geometry_msgs/msg/Twist:
     Vector3 linear
       float64 x    # Not used
       float64 y    # Not used
       float64 z    # Not used
     Vector3 angular
       float64 x    # Head pitch (-335, 0, or 335)
       float64 y    # Head pan, normalized to [-1.0, 1.0] (positive: left)
       float64 z    # Head tilt, normalized to [-1.0, 1.0] 
   ```

3. **Audio Data** (`/vr_out/audio_input`, std_msgs/msg/Int16MultiArray)
   - Sends audio from VR system to the robot
   - Message structure:
   ```
   std_msgs/msg/Int16MultiArray:
     std_msgs/MultiArrayLayout layout
       std_msgs/MultiArrayDimension[] dim
         string label
         uint32 size
         uint32 stride
       uint32 data_offset
     int16[] data    # Audio samples (16-bit PCM)
   ```

### Messages to Receive in Unity from ROS2

1. **Robot Pose** (`/vr_in/robot_pose`, geometry_msgs/msg/PoseWithCovarianceStamped)
   - Provides the current position and orientation of the robot
   - Message structure:
   ```
   geometry_msgs/msg/PoseWithCovarianceStamped:
     std_msgs/Header header
       uint32 seq
       time stamp
       string frame_id
     geometry_msgs/PoseWithCovariance pose
       geometry_msgs/Pose pose
         geometry_msgs/Point position
           float64 x
           float64 y
           float64 z
         geometry_msgs/Quaternion orientation
           float64 x
           float64 y
           float64 z
           float64 w
       float64[36] covariance
   ```

2. **Human Position** (`/vr_in/human_position`, geometry_msgs/msg/PoseStamped)
   - Provides the position of detected humans
   - Message structure:
   ```
   geometry_msgs/msg/PoseStamped:
     std_msgs/Header header
       uint32 seq
       time stamp
       string frame_id
     geometry_msgs/Pose pose
       geometry_msgs/Point position
         float64 x
         float64 y
         float64 z
       geometry_msgs/Quaternion orientation
         float64 x
         float64 y
         float64 z
         float64 w
   ```

3. **Human Skeleton** (`/vr_in/human_skeleton`, visualization_msgs/msg/MarkerArray)
   - Provides visualization data for human skeleton
   - Message structure:
   ```
   visualization_msgs/msg/MarkerArray:
     visualization_msgs/Marker[] markers
       # Each marker represents a joint or bone in the skeleton
       std_msgs/Header header
       string ns
       int32 id
       int32 type
       int32 action
       geometry_msgs/Pose pose
       geometry_msgs/Vector3 scale
       std_msgs/ColorRGBA color
       duration lifetime
       bool frame_locked
       geometry_msgs/Point[] points
       std_msgs/ColorRGBA[] colors
       string text
       string mesh_resource
       bool mesh_use_embedded_materials
   ```

4. **Human Skeleton Poses** (`/vr_in/human_skeleton_poses`, geometry_msgs/msg/PoseArray)
   - Provides joint positions for human skeleton
   - Message structure:
   ```
   geometry_msgs/msg/PoseArray:
     std_msgs/Header header
     geometry_msgs/Pose[] poses
       # Each pose represents a joint position
       geometry_msgs/Point position
         float64 x
         float64 y
         float64 z
       geometry_msgs/Quaternion orientation
         float64 x
         float64 y
         float64 z
         float64 w
   ```

5. **Audio Output** (`/vr_in/audio_output`, std_msgs/msg/Int16MultiArray)
   - Audio data from the robot's microphone
   - Message structure:
   ```
   std_msgs/msg/Int16MultiArray:
     std_msgs/MultiArrayLayout layout
       std_msgs/MultiArrayDimension[] dim
         string label
         uint32 size
         uint32 stride
       uint32 data_offset
     int16[] data    # Audio samples (16-bit PCM)
   ```

## System Scripts

Several utility scripts are provided to help manage the robot's operation:

### Building and Setup

- **build_workspace.sh** - Builds the ROS2 workspace using colcon
  ```bash
  ./build_workspace.sh
  ```

### Running the Robot

- **start_tino_with_rtab.sh** - Launches the Tino robot with RTAB-Map localization and gamepad control in a tmux session
  ```bash
  ./start_tino_with_rtab.sh
  ```
  This script creates a horizontally split tmux session with RTAB-Map on the left side and gamepad control on the right.

- **start_tino_with_rtab_no_gamepad.sh** - Launches the Tino robot with RTAB-Map localization and basic robot control (without gamepad) in a tmux session
  ```bash
  ./start_tino_with_rtab_no_gamepad.sh
  ```
  This script creates a horizontally split tmux session with RTAB-Map on the left side and basic robot control on the right.

### Stopping the Robot

- **kill_tino_tmux.sh** - Kills the Tino robot tmux session
  ```bash
  ./kill_tino_tmux.sh
  ```
  This script safely terminates any running tmux sessions for the robot.

### Other Utility Scripts

- **run_rtabmap.sh** - A wrapper script to run rtabmap with proper environment configuration
- **setup_arduino_symlinks.sh** - Sets up symbolic links for Arduino communication
- **setup_audio.sh** - Configures audio settings for the robot
- **upload_head.sh** - Uploads firmware to the head Arduino
- **upload_new_base.sh** - Uploads firmware to the base Arduino

### TMux Session Management

To detach from a tmux session without stopping it, press `Ctrl+B` then `D`.
To re-attach to a running session later:
- For the standard configuration: `tmux attach-session -t tino`
- For the no-gamepad configuration: `tmux attach-session -t tino_basic`
