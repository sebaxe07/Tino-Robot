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
- **DWM1001 UWB Module** - for precise indoor positioning and sensor fusion
- **Microphone and speaker** - for audio communication
- **Gamepad** - user input device (Logitech Cordless RumblePad 2)

### Current Software Components

- **Arduino firmware** - motor control for robot components
- **ROS2 Humble nodes** - distributed system for control, perception, and communication
- **RTABMAP** - for Simultaneous Localization and Mapping (SLAM)
- **DepthAI** - for Oak-D Pro camera integration
- **UWB Positioning** - DWM1001-based ultra-wideband indoor positioning system

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
      ├── tino_ros/        # ROS2 package for Tino robot
      │   ├── launch/      # Launch files for different system configurations
      │   ├── resource/    # Resource files (models, etc.)
      │   └── tino_ros/    # Python implementation of ROS2 nodes
      └── uwb_positioning/ # UWB positioning package for indoor localization
```

## ROS2 Node Architecture

The Tino ROS2 system consists of several interconnected nodes:

- **robot_controller_node**: Central controller that coordinates all components and performs sensor fusion
- **hardware_interface_node**: Communicates with Arduino boards via serial
- **gamepad_node**: Processes gamepad inputs for robot control
- **pose_detection_node**: Human detection and skeleton tracking
- **audio_node**: Handles audio input/output
- **vr_interface_node**: Interface for potential VR integration
- **uwb_positioning_node**: DWM1001 UWB positioning system for precise indoor localization

### Key Topics and Messages

- **/base_cmd_vel** (Twist): Commands for base movement
- **/head_cmd** (Twist): Commands for head movement
- **/human_position** (PoseStamped): Detected human position
- **/human_skeleton** (MarkerArray): Visualization of detected human skeleton
- **/localization_pose** (PoseWithCovarianceStamped): Robot's position from SLAM
- **/UWB/Pos** (Pose): UWB positioning data for sensor fusion
- **/UWB/Ranges** (Range): UWB range measurements from anchor nodes
- **/vr_in/robot_pose** (PoseWithCovarianceStamped): Fused robot pose (RTAB orientation + UWB position)
- **/audio/mic_input** and **/audio/vr_output** (Int16MultiArray): Audio data streams

### Sensor Fusion Architecture

The robot implements advanced sensor fusion combining:
- **RTAB-Map**: Provides reliable orientation and visual odometry
- **UWB Positioning**: Provides accurate absolute position in indoor environments
- **Final Output**: Robot pose with UWB position and RTAB-Map orientation for optimal localization

## Modern Setup Instructions

### Hardware Setup

1. Upload the appropriate sketch to each Arduino board:

   - `legacy_tino/base_tino/base_tino.ino` to the base Arduino
   - `legacy_tino/head_OL_tino_base/head_OL_tino_base.ino` to the head Arduino
   - `legacy_tino/leg_tino/leg_tino.ino` to the leg Arduino

2. Connect each Arduino to the Jetson Orin Nano via USB

3. Connect the DWM1001 UWB module to the Jetson Orin Nano via USB

4. Connect the Oak-D Pro camera to the Jetson Orin Nano

5. Connect microphone and speaker to the Jetson Orin Nano

6. Set up persistent device names for reliable connections:
   ```bash
   # Run the setup script for each device (one at a time)
   ./setup_arduino_symlinks.sh
   ```
   
   This will create persistent device links:
   - Base Arduino: `/dev/ttyBASE`
   - Leg Arduino: `/dev/ttyLEG`
   - Head Arduino: `/dev/ttyHEAD`
   - UWB Receiver: `/dev/ttyUWB`

7. Upload Arduino firmware:
   ```bash
   # Upload to individual Arduinos
   ./upload_new_base.sh    # Upload base Arduino firmware
   ./upload_head.sh        # Upload head Arduino firmware  
   ./upload_leg.sh         # Upload leg Arduino firmware
   
   # For debugging/fast mapping (higher speed base control)
   ./upload_debug_base_wasd.sh  # Upload debug base firmware with WASD controls
   ```

### Software Setup

1. Install ROS2 Humble on the Jetson Orin Nano:

   ```bash
   # Follow ROS2 Humble installation instructions for Ubuntu 20.04
   ```

2. Install required ROS2 packages:

   ```bash
   sudo apt install ros-humble-rtabmap-ros ros-humble-depthai-ros
   ```

3. Build the workspace:

   ```bash
   ./build_workspace.sh
   ```

4. Install Python dependencies:
   ```bash
   pip install pyaudio numpy ultralytics evdev pyserial
   ```

5. Set up audio system (runs automatically at startup, but can be run manually):
   ```bash
   # Configure audio devices and PulseAudio settings
   ./setup_audio.sh
   ```
   
   Note: This script is automatically executed when the robot controller starts, but you can run it manually if audio issues occur.

### Modern Usage Instructions

1. Connect all batteries and turn on the system:

   - Battery for wheels and leg
   - Battery for head
   - Battery for the Jetson Orin Nano

2. Launch the ROS2 system using tmux scripts:

   ```bash
   # Full system with SLAM, pose detection, and gamepad control
   ./start_tino_with_rtab.sh
   
   # Full system without gamepad (for VR or remote control)
   ./start_tino_with_rtab_no_gamepad.sh
   ```
   
   These scripts create a tmux session with split windows:
   - Left side: RTAB-Map SLAM system
   - Right side: Robot control system

3. Turn on the gamepad by pressing the central button (if using gamepad version)

4. **Detaching from tmux session**: Press `Ctrl+B` then `D` to detach without stopping the robot
   
5. **Re-attaching to session**:
   ```bash
   # For gamepad version
   tmux attach-session -t tino
   
   # For no-gamepad version  
   tmux attach-session -t tino_basic
   ```

6. **Stopping the system**:
   ```bash
   # Kill the entire tmux session and all robot processes
   ./kill_tino_tmux.sh
   ```

7. Control the robot using the gamepad:

   **Base Movement:**
   - Bumper buttons (L1/R1): Atomic rotation movements with automatic case 3 execution
     - Left bumper: Counter-clockwise rotation + case 3
     - Right bumper: Clockwise rotation + case 3

   **Head Control:**
   - Right stick: Head pan/tilt movement
   - D-pad up/down: Head pitch adjustment

   **Leg Control (Atomic Movement System):**
   - X button: Little push movement (case 1)
   - Y button: Forward movement until maximum reach (case 2)
   - B button: Finish cycle and return to neutral (case 3)
   - A button: Return to idle state (case 0)

   **Advanced Control Logic:**
   - Case 2 must be executed before case 3 can be triggered
   - All movements are atomic (must complete once started)
   - Pending system: case 3 commands received during case 2 are stored and executed automatically
   - Pulse system: each button press sends a 3-cycle command pulse

5. To stop the system, press Ctrl+C in the terminal

### Alternative Launch Methods

If you prefer to run components individually without tmux, you can use these manual launch commands:

```bash
# Full system with SLAM and pose detection
ros2 launch tino_ros tino_robot.launch.py

# Gamepad version with SLAM
ros2 launch tino_ros tino_robot_gamepad.launch.py

# For SLAM mapping mode only
ros2 launch tino_ros rtab_mapping.launch.py

# For SLAM localization mode only
ros2 launch tino_ros rtab_localization.launch.py
```

## Debug Mode for Fast Mapping

For faster robot movement during mapping or debugging, use the debug base firmware:

1. Upload debug firmware:
   ```bash
   ./upload_debug_base_wasd.sh
   ```

2. This enables:
   - Higher speed base movement
   - WASD-style directional controls
   - Optimized for quick environment mapping
   - Faster response times for debugging

3. **Important**: Switch back to normal firmware for regular operation:
   ```bash
   ./upload_new_base.sh
   ```

## UWB Positioning System

### Overview

The Tino robot integrates a DWM1001 Ultra-Wideband (UWB) positioning system for precise indoor localization. This system provides centimeter-level accuracy and is used in combination with RTAB-Map for optimal sensor fusion.

### UWB Topics and Data

The UWB positioning node publishes two main topics:

**`/UWB/Pos` (geometry_msgs/Pose)**
- 3D position coordinates (X, Y, Z) from the UWB positioning system
- Orientation set to identity quaternion (position-only data)
- Used for sensor fusion with RTAB-Map orientation

**`/UWB/Ranges` (sensor_msgs/Range)**
- Distance measurements from UWB anchor nodes
- Range: 0.1m to 50.0m
- Frame ID: 'dwm1001'
- Raw ranging data for debugging and analysis

### Sensor Fusion Process

1. **RTAB-Map**: Provides reliable orientation and visual odometry
2. **UWB System**: Provides accurate absolute position coordinates
3. **Robot Controller**: Fuses both sources:
   - **Position**: Uses UWB coordinates for precise localization
   - **Orientation**: Uses RTAB-Map quaternion for reliable heading
   - **Output**: Combined pose published to `/vr_in/robot_pose`

### UWB Configuration

The UWB module connects via USB and uses the following default parameters:
- **Serial Port**: `/dev/ttyUWB` (persistent device name)
- **Baud Rate**: 115200
- **Device**: DWM1001 development board

### Troubleshooting UWB

- **Check device connection**: `ls -la /dev/ttyUWB`
- **Monitor UWB topics**: `ros2 topic echo /UWB/Pos`
- **View UWB ranges**: `ros2 topic echo /UWB/Ranges`
- **Check fusion status**: Look for "Sensor Fusion" messages in robot controller logs

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

## Modern Control System

### Atomic Movement Architecture

The current implementation features an advanced atomic movement system that ensures coordinated operation between the base and leg components:

#### Movement States and Commands

**Base Controller:**
- **Case 1**: Little push movement (600ms delay + 200ms forward + 200ms backward)
- **Case 2**: Timing synchronization cycle (1.5 seconds, locks system for case 3)
- **Case 3**: Atomic movements that require case 2 completion:
  - `(3, 0)`: Forward movement (1.7 seconds)
  - `(3, 1)`: Right rotation (1.7 seconds)
  - `(3, -1)`: Left rotation (1.7 seconds)

**Leg Controller:**
- **Case 1**: Little push movement (optimized 3-phase: 50% forward, 5% stop, 45% return)
- **Case 2**: Forward movement until maximum extension with locking mechanism
- **Case 3**: Return to neutral position (button-press completion detection)

#### Synchronization Features

**Locking System:**
- Case 2 execution locks the system, preventing other commands
- Only case 3 can unlock the system after case 2 completion
- Ensures coordinated leg extension before base movement

**Pending Command System:**
- Commands received during case 2 are stored and executed automatically
- Supports pending forward movement and rotation commands
- Maintains command sequence integrity

**Pulse System:**
- Each button press generates a 3-cycle command pulse
- Automatic return to idle after pulse completion
- Consistent behavior regardless of button hold duration

### Gamepad Integration

The gamepad interface implements a sophisticated control scheme:

- **Rotation + Case 3**: Bumper buttons simultaneously trigger rotation and leg coordination
- **Atomic Leg Movements**: Face buttons control precise leg positioning
- **Real-time Feedback**: Debug output provides system state information

## Communication Protocol

The Arduino modules and controllers communicate using a simple serial protocol with key-value pairs:

**Basic Commands:**
- `BF`: Base Forward motion and leg command selector (0=idle, 1=little push, 2=forward, 3=finish cycle)
- `BB`: Base angular motion (rotation values and pulse system)
- `HP`: Head Pitch
- `HX`: Head X-axis (pan)
- `HY`: Head Y-axis (tilt)

**Advanced Command Combinations:**
- `BF=3, BB=0`: Forward movement (case 3,0)
- `BF=3, BB=1`: Right rotation (case 3,1) 
- `BF=3, BB=-1`: Left rotation (case 3,-1)
- `BF=2, BB=0`: Synchronization timing cycle (case 2)

**Command Flow Example:**
1. Press Y button → `BF=2` (case 2 timing cycle starts)
2. Press left bumper during case 2 → command stored as pending
3. Case 2 completes → automatically executes `BF=3, BB=-1` (left rotation)
4. Movement completes → returns to `BF=0, BB=0` (idle)

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

- **setup_arduino_symlinks.sh** - Sets up persistent device names for all connected devices
  ```bash
  ./setup_arduino_symlinks.sh
  ```

- **setup_audio.sh** - Configures audio settings and PulseAudio for the robot
  ```bash
  ./setup_audio.sh
  ```

### Arduino Firmware Upload

- **upload_new_base.sh** - Uploads standard firmware to the base Arduino
  ```bash
  ./upload_new_base.sh
  ```

- **upload_head.sh** - Uploads firmware to the head Arduino
  ```bash
  ./upload_head.sh
  ```

- **upload_leg.sh** - Uploads firmware to the leg Arduino
  ```bash
  ./upload_leg.sh
  ```

- **upload_legacy_leg.sh** - Uploads legacy firmware to the leg Arduino
  ```bash
  ./upload_legacy_leg.sh
  ```

- **upload_debug_base_wasd.sh** - Uploads debug firmware with enhanced speed and WASD controls
  ```bash
  ./upload_debug_base_wasd.sh
  ```
  Use this for faster movement during mapping or debugging sessions.

### Running the Robot

- **start_tino_with_rtab.sh** - Launches the complete Tino robot system with RTAB-Map localization and gamepad control in a tmux session
  ```bash
  ./start_tino_with_rtab.sh
  ```
  Creates a horizontally split tmux session with RTAB-Map on the left and gamepad control on the right.

- **start_tino_with_rtab_no_gamepad.sh** - Launches the Tino robot with RTAB-Map localization and basic robot control (without gamepad) in a tmux session
  ```bash
  ./start_tino_with_rtab_no_gamepad.sh
  ```
  Creates a horizontally split tmux session with RTAB-Map on the left and robot control on the right.

### Stopping the Robot

- **kill_tino_tmux.sh** - Safely terminates all Tino robot tmux sessions and processes
  ```bash
  ./kill_tino_tmux.sh
  ```
  This script ensures all robot processes are properly stopped.

### Other Utility Scripts

- **run_rtabmap.sh** - A wrapper script to run rtabmap with proper environment configuration
- **depth_calibration_helper.py** - Assists with camera depth calibration
- **test_skeleton_generator.py** - Tests the human skeleton detection system
- **test_skeleton_udp_receiver.py** - Tests UDP skeleton data reception
- **test_vr_udp_client.py** - Tests VR interface UDP communication

### TMux Session Management

- **Detach from session**: Press `Ctrl+B` then `D` to detach without stopping the robot
- **Re-attach to running session**:
  - Standard configuration: `tmux attach-session -t tino`
  - No-gamepad configuration: `tmux attach-session -t tino_basic`
- **List active sessions**: `tmux list-sessions`
- **Kill specific session**: `tmux kill-session -t tino` (or use the kill script above)
