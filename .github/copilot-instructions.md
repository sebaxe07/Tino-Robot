# Tino Robot AI Agent Instructions

## Project Overview

Tino Robot is a multi-component robotics platform that evolved from a legacy Raspberry Pi system to a modern ROS2-based implementation running on NVIDIA Orin Nano. The robot features differential drive locomotion, 3-DOF head movement, single-actuator leg mechanism, and advanced perception capabilities including SLAM, human pose detection, and UWB positioning.

## Architecture Components

### Hardware Architecture

- **3 Arduino Controllers**: Base (motor control), Head (3 servos), Leg (single actuator)
- **NVIDIA Orin Nano**: Main computation running ROS2 Humble
- **Luxonis Oak-D Pro**: Stereo camera for SLAM and human detection
- **DWM1001 UWB Module**: Precise indoor positioning (cm-level accuracy)
- **Audio System**: Microphone and speaker for human interaction

### Software Architecture - ROS2 Nodes

The system uses a distributed ROS2 architecture with these key nodes:

- `robot_controller_node`: Central coordinator with sensor fusion (UWB + RTAB-Map)
- `hardware_interface_node`: Serial communication with Arduino controllers
- `gamepad_node`: Xbox controller input processing
- `pose_detection_node`: YOLO11-based human skeleton detection
- `audio_node`: Audio I/O handling with PulseAudio
- `vr_interface_node`: Unity VR system communication
- `uwb_positioning_node`: DWM1001 UWB positioning system

## Critical Communication Patterns

### Arduino Serial Protocol

Uses persistent device names via udev rules:

```
/dev/ttyBASE (Base Arduino)
/dev/ttyLEG (Leg Arduino)
/dev/ttyHEAD (Head Arduino)
/dev/ttyUWB (UWB Module)
```

**Command Format**: Key-value pairs at 115200 baud with 3x repetition for reliability:

```
BF=<value>,BB=<value>,HP=<value>,HX=<value>,HY=<value>
```

### Atomic Movement System

The robot implements sophisticated movement coordination:

- **Case 1**: Little push (600ms sequence)
- **Case 2**: Timing synchronization (1.5s, locks system)
- **Case 3**: Coordinated movements (requires Case 2 completion)
- **Pending Commands**: Commands received during Case 2 are queued and auto-executed

### Sensor Fusion Architecture

**Position Source**: UWB positioning (when available) OR RTAB-Map localization
**Orientation Source**: Always RTAB-Map quaternion
**Output**: Combined pose on `/vr_in/robot_pose` (PoseStamped)

## Key Development Workflows

### Build & Setup

```bash
./build_workspace.sh                    # Build ROS2 workspace
./setup_arduino_symlinks.sh            # Configure persistent device names (run once per device)
./setup_audio.sh                       # Configure PulseAudio (auto-runs at startup)
```

### Arduino Firmware Upload

```bash
./upload_new_base.sh                    # Standard base firmware
./upload_debug_base_wasd.sh             # High-speed firmware for mapping
./upload_head.sh                        # Head controller
./upload_leg.sh                         # Leg controller
```

### Robot Operation

```bash
./start_tino_with_rtab.sh               # Full system with gamepad in tmux
./start_tino_with_rtab_no_gamepad.sh    # System without gamepad in tmux
./kill_tino_tmux.sh                     # Safe shutdown of all processes
```

**Tmux Session Management**:

- Detach: `Ctrl+B` then `D`
- Reattach: `tmux attach-session -t tino`
- Split view: RTAB-Map (left) + Robot Control (right)

## Code Patterns & Conventions

### ROS2 Launch File Structure

Launch files in `tino_ws/src/tino_ros/launch/` follow these patterns:

- Use persistent device names in parameters
- Include UWB positioning node with `/dev/ttyUWB`
- Camera launch includes stereo+inertial configuration
- Hardware interface expects specific port parameters

### Message Flow Patterns

**VR Interface Messages**:

- IN: `/vr_out/cmd_vel` (Twist), `/vr_out/head_cmd` (Twist), `/vr_out/audio_input`
- OUT: `/vr_in/robot_pose` (PoseStamped), `/vr_in/human_position`, `/vr_in/audio_output`

**Hardware Interface Pattern**:

- Queue-based command processing at 25Hz
- Thread-safe serial communication with locks
- Automatic reconnection on serial failures

### Error Handling Conventions

- Serial connections: Graceful degradation when devices unavailable
- UWB positioning: Fallback to RTAB-Map only when UWB unavailable
- Audio system: Auto-setup with fallback detection

## Common Tasks

### Adding New ROS2 Nodes

1. Create node in `tino_ws/src/tino_ros/tino_ros/`
2. Add executable to `setup.py`
3. Include in appropriate launch file
4. Follow existing parameter declaration patterns

### Modifying Arduino Communication

- Edit command generation in `hardware_interface_node.py::generate_command()`
- Maintain the key-value format: `KEY=value,KEY2=value2`
- Consider message repetition for reliability

### Debug Common Issues

**No Serial Connection**: Check `ls -la /dev/tty*` and udev rules in `/etc/udev/rules.d/99-tino-arduino.rules`
**Audio Issues**: Run `./setup_audio.sh` manually or check PulseAudio configuration
**SLAM Issues**: Verify camera topics with `ros2 topic list | grep stereo`
**UWB Issues**: Check `/UWB/Pos` and `/UWB/Ranges` topics

### VR Integration Development

- Unity should subscribe to `/vr_in/*` topics for robot data
- Unity publishes to `/vr_out/*` topics for control commands
- Audio uses Int16MultiArray format for PCM data
- Robot pose includes fused UWB position + RTAB-Map orientation

## File Organization

- `legacy_tino/`: Original Raspberry Pi implementation (preserved)
- `tino_ws/src/tino_ros/`: Modern ROS2 implementation
- Root shell scripts: System management and Arduino upload utilities
- `*.launch.py`: ROS2 launch configurations with parameter management

## Testing & Validation

- Use debug base firmware for faster movement during mapping
- Monitor sensor fusion in robot controller logs
- Verify atomic movement coordination through gamepad testing
- Check audio loopback functionality before VR integration
