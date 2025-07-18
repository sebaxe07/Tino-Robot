# VR Interface Node - Complete Modifications

## Overview

The VR interface node has been enhanced with:
1. **Consistent skeleton data format** (always 17 joints)
2. **Communication monitoring** with configurable rates and health checks
3. **Configurable send rates** for pose and skeleton data separately
4. **Comprehensive logging** for debugging and monitoring

## Key Features

### 1. Skeleton Data Consistency

The skeleton data sender now **always sends exactly 17 joints** in the COCO pose format:
- **Missing joints**: Set to `(0.0, 0.0, 0.0)` coordinates
- **Joint order**: Fixed COCO format (17 joints)
- **Data size**: Always 208 bytes per skeleton packet

### 2. Communication Monitoring

New monitoring system tracks:
- **Incoming data rate**: Monitors VR commands reception
- **Outgoing data rate**: Tracks pose and skeleton transmission
- **Health checks**: Logs errors when data is missing
- **Status reports**: Regular success confirmations

### 3. Configurable Parameters

All rates and monitoring intervals are configurable via ROS parameters:

```yaml
# Send rates (Hz)
pose_send_rate_hz: 10.0           # Rate for robot pose data
skeleton_send_rate_hz: 10.0       # Rate for skeleton data  
expected_receive_rate_hz: 25.0    # Expected incoming VR command rate
monitoring_log_interval_sec: 10.0 # Log interval for status reports
```

## Parameter Configuration

### ROS Parameters

The node supports these configurable parameters:

```yaml
# UDP Communication
udp_port: 5005                    # Listening port for VR commands
udp_bind_address: "0.0.0.0"       # Bind address for listening
udp_send_port: 5006               # Port for sending pose data
udp_send_address: "192.168.0.201" # Address for sending pose data
udp_skeleton_port: 5007           # Port for sending skeleton data
udp_skeleton_address: "192.168.0.201" # Address for sending skeleton data

# Rates and Monitoring
pose_send_rate_hz: 10.0           # Pose data send rate
skeleton_send_rate_hz: 10.0       # Skeleton data send rate
expected_receive_rate_hz: 25.0    # Expected VR command rate
monitoring_log_interval_sec: 10.0 # Status logging interval

# Logging
log_level: "INFO"                 # Log level (DEBUG, INFO, WARN, ERROR)
```

### Setting Parameters

You can override parameters when launching:

```bash
# Example: Change send rates and monitoring
ros2 run tino_ros vr_interface_node \
    --ros-args \
    -p pose_send_rate_hz:=20.0 \
    -p skeleton_send_rate_hz:=15.0 \
    -p expected_receive_rate_hz:=30.0 \
    -p monitoring_log_interval_sec:=5.0
```

## Monitoring Features

### 1. Incoming Data Monitoring

**Error Detection**: If no VR commands are received for more than 2× the expected interval:
```
[ERROR] No VR command data received for 0.5s (expected rate: 25Hz)
```

**Health Reports**: Every 10 seconds when data is flowing normally:
```
[INFO] VR communication healthy - receiving at 24.8Hz (expected: 25Hz)
```

### 2. Outgoing Data Monitoring  

**Success Reports**: Every 10 seconds showing transmission statistics:
```
[INFO] Successfully sending data - Pose: 10.0Hz, Skeleton: 10.0Hz (Total packets: pose=100, skeleton=100)
```

### 3. Startup Information

The node logs all configured rates at startup:
```
[INFO] Send rates configured - Pose: 10.0Hz, Skeleton: 10.0Hz
[INFO] Expected incoming rate: 25.0Hz
```

## UDP Packet Structures

### VR Command Packet (Port 5005 - Incoming)
```
Format: 3 floats + 2 ints + 1 int + 1 float + 1 int = 32 bytes
- 3 floats: head_pitch, head_pan, head_tilt
- 2 ints: base_state (0-3), base_angular (-1,0,1)  
- 1 int: audio_volume
- 1 float: audio_orientation
- 1 int: message_order
```

### Pose Data Packet (Port 5006 - Outgoing)
```
Format: 2 ints + 4 floats = 24 bytes
- 1 int: message_order
- 4 floats: robot position (x,y) and orientation (z,w)
- 1 int: audio_volume
```

### Skeleton Data Packet (Port 5007 - Outgoing)
```
Format: 1 int + 51 floats = 208 bytes
- 1 int: message_order
- 51 floats: 17 joints × 3 coordinates (x,y,z) each
```

## Testing Tools

### Enhanced Test Client

The `test_vr_udp_client.py` now includes rate testing:

```bash
# Interactive mode with menus
python3 test_vr_udp_client.py

# Direct demo sequence
python3 test_vr_udp_client.py demo

# Rate testing menu
python3 test_vr_udp_client.py rate
```

**Rate Testing Options**:
1. Send at 25Hz for 30 seconds (normal rate)
2. Send at 10Hz for 20 seconds (slow rate) 
3. Send at 50Hz for 15 seconds (fast rate)
4. Stop sending for 15 seconds (test error detection)
5. Custom rate and duration

### Monitoring Test

To test the monitoring system:

1. **Start VR interface**:
   ```bash
   ros2 run tino_ros vr_interface_node
   ```

2. **Test normal operation**:
   ```bash
   python3 test_vr_udp_client.py rate
   # Select option 1 (25Hz for 30s)
   ```

3. **Test error detection**:
   ```bash
   python3 test_vr_udp_client.py rate  
   # Select option 4 (stop sending)
   # Check logs for error messages
   ```

4. **Monitor skeleton output**:
   ```bash
   python3 test_skeleton_udp_receiver.py
   ```

## Benefits

### 1. Reliable Communication
- **Health monitoring**: Know immediately if communication fails
- **Rate validation**: Ensure data flows at expected rates
- **Message ordering**: Detect lost or duplicate packets

### 2. Flexible Configuration
- **Independent rates**: Optimize pose vs skeleton transmission separately
- **Runtime tuning**: Adjust rates without code changes
- **Environment adaptation**: Configure for different network conditions

### 3. Debugging Support
- **Comprehensive logging**: Detailed information for troubleshooting
- **Status reports**: Regular confirmation of system health
- **Error detection**: Clear indication of communication problems

### 4. Production Ready
- **Configurable monitoring**: Adjust logging frequency as needed
- **Resource efficient**: Minimal overhead for monitoring
- **Robust operation**: Graceful handling of communication issues

## Usage Examples

### Development Mode
```bash
# High logging for debugging
ros2 run tino_ros vr_interface_node \
    --ros-args \
    -p log_level:="DEBUG" \
    -p monitoring_log_interval_sec:=5.0
```

### Production Mode  
```bash
# Optimized rates, less frequent logging
ros2 run tino_ros vr_interface_node \
    --ros-args \
    -p pose_send_rate_hz:=15.0 \
    -p skeleton_send_rate_hz:=5.0 \
    -p monitoring_log_interval_sec:=30.0 \
    -p log_level:="INFO"
```

### High Performance Mode
```bash
# Maximum rates for low-latency applications
ros2 run tino_ros vr_interface_node \
    --ros-args \
    -p pose_send_rate_hz:=50.0 \
    -p skeleton_send_rate_hz:=30.0 \
    -p expected_receive_rate_hz:=60.0
```
