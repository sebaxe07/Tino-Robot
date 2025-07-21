import rclpy
from rclpy.node import Node
import rclpy.logging
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray
import socket
import threading
import struct

class VRInterfaceNode(Node):
    def __init__(self):
        super().__init__('vr_interface_node')
        
        # Set logging level to reduce verbosity (ERROR, WARN, INFO, DEBUG)
        # Default to INFO, but can be changed with ROS param
        self.declare_parameter('log_level', 'INFO')
        self.declare_parameter('udp_port', 5000)
        self.declare_parameter('udp_bind_address', '0.0.0.0')
        
        # Parameters for sending UDP data
        self.declare_parameter('udp_send_port', 5006)
        self.declare_parameter('udp_send_address', '192.168.0.201')
        
        # Parameters for sending skeleton UDP data
        self.declare_parameter('udp_skeleton_port', 5007)
        self.declare_parameter('udp_skeleton_address', '192.168.0.201')
        
        # Parameters for send rates and monitoring
        self.declare_parameter('pose_send_rate_hz', 10.0)  # Hz for pose data
        self.declare_parameter('skeleton_send_rate_hz', 10.0)  # Hz for skeleton data
        self.declare_parameter('expected_receive_rate_hz', 25.0)  # Expected incoming data rate
        self.declare_parameter('monitoring_log_interval_sec', 10.0)  # Log interval for monitoring
        self.declare_parameter('vr_disconnection_timeout_sec', 3.0)  # Timeout before considering VR disconnected

        log_level = self.get_parameter('log_level').get_parameter_value().string_value
        if hasattr(rclpy.logging, log_level):
            self.get_logger().set_level(getattr(rclpy.logging, log_level))
        
        # Subscribe to robot pose from controller
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vr_in/robot_pose', 
            self.pose_callback,
            10)
             
             

        # Subscribe to human skeleton data
        self.human_skeleton_sub = self.create_subscription(
            MarkerArray,
            '/vr_in/human_skeleton',
            self.human_skeleton_callback,
            10)
        self.human_skeleton_poses_sub = self.create_subscription(
            PoseArray,
            '/vr_in/human_skeleton_poses',
            self.human_skeleton_poses_callback,
            10)
        
        # Publishers for VR-based control
        self.cmd_vel_pub = self.create_publisher(Twist, '/vr_out/cmd_vel', 10)
        self.head_cmd_pub = self.create_publisher(Twist, '/vr_out/head_cmd', 10)

        # Subscribe to Audio from controller
        self.mic_audio_sub = self.create_subscription(
            Int16MultiArray, '/vr_in/audio_output', self.mic_audio_callback, 10)

        self.vr_audio_pub = self.create_publisher(
            Float32MultiArray, '/vr_out/audio_input', 10)
        
        
        # Store human position data
        self.human_position = None
        
        # Store skeleton data
        self.human_skeleton = None
        self.human_skeleton_poses = None
        
        # COCO skeleton joint names and indices (consistent with pose_detection_node.py)
        # This ensures we always send data in the same order
        self.skeleton_joint_names = [
            'nose',           # 0
            'left_eye',       # 1
            'right_eye',      # 2
            'left_ear',       # 3
            'right_ear',      # 4
            'left_shoulder',  # 5
            'right_shoulder', # 6
            'left_elbow',     # 7
            'right_elbow',    # 8
            'left_wrist',     # 9
            'right_wrist',    # 10
            'left_hip',       # 11
            'right_hip',      # 12
            'left_knee',      # 13
            'right_knee',     # 14
            'left_ankle',     # 15
            'right_ankle'     # 16
        ]
        
        # Number of skeleton joints (COCO format)
        self.num_skeleton_joints = 17
        
        # UDP Socket for VR communication
        self.udp_socket = None
        self.udp_thread = None
        self.is_running = True
        
        # Message ordering tracking
        self.last_message_order = -1
        
        # VR connection tracking
        self.is_vr_connected = False
        self.vr_disconnection_timeout = self.get_parameter('vr_disconnection_timeout_sec').get_parameter_value().double_value
        
        # Monitoring variables for incoming data rate
        self.last_received_time = None
        self.received_message_count = 0
        self.last_monitoring_log_time = None
        
        # Monitoring variables for outgoing data
        self.pose_packets_sent = 0
        self.skeleton_packets_sent = 0
        self.last_send_log_time = None
        
        # Outgoing message order counter
        self.outgoing_message_order = 0
        
        # Outgoing skeleton message order counter
        self.skeleton_message_order = 0
        
        # VR client address for sending data
        self.vr_client_address = None
        
        # UDP send configuration
        self.udp_send_port = None
        self.udp_send_address = None
        
        # UDP skeleton send configuration
        self.udp_skeleton_port = None
        self.udp_skeleton_address = None
        
        # Store latest data for sending
        self.latest_robot_pose = None
        self.latest_audio_volume = 0
        self.latest_skeleton_data = None
        

        self.latest_command = None

        # Initialize VR communication
        self.setup_vr_communication()
        
        # Get UDP send configuration
        self.udp_send_port = self.get_parameter('udp_send_port').get_parameter_value().integer_value
        self.udp_send_address = self.get_parameter('udp_send_address').get_parameter_value().string_value
        
        # Get UDP skeleton send configuration
        self.udp_skeleton_port = self.get_parameter('udp_skeleton_port').get_parameter_value().integer_value
        self.udp_skeleton_address = self.get_parameter('udp_skeleton_address').get_parameter_value().string_value
        
        # Get send rates
        pose_send_rate = self.get_parameter('pose_send_rate_hz').get_parameter_value().double_value
        skeleton_send_rate = self.get_parameter('skeleton_send_rate_hz').get_parameter_value().double_value
        
        # Create timers with configurable rates
        self.vr_send_timer = self.create_timer(1.0 / pose_send_rate, self.send_data_to_vr)
        self.skeleton_send_timer = self.create_timer(1.0 / skeleton_send_rate, self.send_skeleton_to_vr)
        
        # Create monitoring timer (every second to check rates)
        self.monitoring_timer = self.create_timer(1.0, self.check_communication_rates)
        
        self.get_logger().info('VR interface node initialized with UDP communication and human skeleton tracking')
    
    def setup_vr_communication(self):
        """Set up UDP communication with VR system"""
        try:
            udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
            bind_address = self.get_parameter('udp_bind_address').get_parameter_value().string_value
            
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_socket.bind((bind_address, udp_port))
            self.udp_socket.settimeout(1.0)  # 1 second timeout for non-blocking behavior
            
            # Start UDP listening thread
            self.udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
            self.udp_thread.start()
            
            self.get_logger().info(f'UDP VR interface listening on {bind_address}:{udp_port}')
            self.get_logger().info(f'UDP VR interface will send to {self.udp_send_address}:{self.udp_send_port}')
            self.get_logger().info(f'UDP VR skeleton will send to {self.udp_skeleton_address}:{self.udp_skeleton_port}')
            
            # Log configured rates
            pose_rate = self.get_parameter('pose_send_rate_hz').get_parameter_value().double_value
            skeleton_rate = self.get_parameter('skeleton_send_rate_hz').get_parameter_value().double_value
            expected_rate = self.get_parameter('expected_receive_rate_hz').get_parameter_value().double_value
            self.get_logger().info(f'Send rates configured - Pose: {pose_rate}Hz, Skeleton: {skeleton_rate}Hz')
            self.get_logger().info(f'Expected incoming rate: {expected_rate}Hz')
        except Exception as e:
            self.get_logger().error(f'Failed to setup UDP communication: {str(e)}')
    
    def udp_listener(self):
        """Listen for UDP messages from VR system"""
        while self.is_running:
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                
                # Store the VR client address for sending data back (but we'll use configured send address/port)
                self.vr_client_address = addr
                
                self.process_vr_data(data, addr)
            except socket.timeout:
                # Normal timeout, continue listening
                continue
            except Exception as e:
                if self.is_running:  # Only log if we're still supposed to be running
                    self.get_logger().error(f'UDP listener error: {str(e)}')
    
    def process_vr_data(self, data, addr):
        """Process incoming VR data and convert to ROS commands"""
        try:
            # Parse the UDP message: float,float,float,int,int,int,float,int
            # Expected format: 8 values (3 floats for head, 2 ints for base, 2 float for audio, 1 int for order)
            expected_size = 3*4 + 2*4 + 2*4 + 1*4  # 3 floats + 2 ints + 2 float + 1 int = 32 bytes

            # Debug: Log raw data to help diagnose packet format issues
            self.get_logger().debug(f"Raw UDP data ({len(data)} bytes): {data.hex()}")

            if len(data) == expected_size:  # 32 bytes total
                values = struct.unpack('fffiiffi', data)
                
                head_tilt, head_pan, head_pitch, base_state, base_angular, audio_volume, audio_orientation, message_order = values
                
                # Check for VR reconnection (reset counter if disconnected)
                import time
                current_time = time.time()
                
                if not self.is_vr_connected:
                    # VR was disconnected, now reconnecting
                    self.get_logger().info('VR system reconnected - resetting message order counter')
                    self.last_message_order = -1  # Reset counter
                    self.is_vr_connected = True
                
                # Check message ordering to detect lost or duplicate messages
                if self.last_message_order != -1:
                    expected_order = (self.last_message_order + 1) % (2**31)  # Handle 32-bit int overflow
                    if message_order != expected_order:
                        if message_order <= self.last_message_order and message_order > 0:
                            self.get_logger().warn(f'Duplicate or old message received. Order: {message_order}, Expected: {expected_order}')
                            return  # Skip processing duplicate/old messages
                        else:
                            self.get_logger().warn(f'Message order gap detected. Expected: {expected_order}, Received: {message_order}')
                
                self.last_message_order = message_order
                
                # Update monitoring variables
                self.last_received_time = current_time
                self.received_message_count += 1
                
                self.get_logger().debug(f'VR data from {addr}: pitch={head_pitch:.2f}, pan={head_pan:.2f}, '
                                      f'tilt={head_tilt:.2f}, state={base_state}, angular={base_angular}, '
                                      f'audio_vol={audio_volume}, audio_orient={audio_orientation:.2f}, order={message_order}')
                
                # Process head commands
                self.process_head_command(head_pitch, head_pan, head_tilt)
                
                # Process base commands
                self.process_base_command(base_state, base_angular)
                
                # Process audio commands
                self.process_audio_command(audio_volume, audio_orientation)
                
            else:
                self.get_logger().warn(f'Invalid VR data length: expected {expected_size} bytes, got {len(data)}')
                
        except struct.error as e:
            self.get_logger().error(f'Failed to unpack VR data: {str(e)}')
            self.get_logger().error(f'Raw data: {data.hex() if data else "None"}')
        except Exception as e:
            self.get_logger().error(f'Error processing VR data: {str(e)}')
            self.get_logger().error(f'Raw data: {data.hex() if data else "None"}')
    
    def process_head_command(self, pitch, pan, tilt):
        """Convert VR head values to ROS Twist message for head control"""
        head_cmd = Twist()
        
        # Map VR values to head control
        # Based on hardware_interface_node.py and head Arduino code:
        # angular.x = pitch (HP - head forward/backward)
        # angular.y = pan (HX - head left/right, needs offset)  
        # angular.z = tilt (HY - head tilt, needs offset)
        
        head_cmd.angular.x = float(pitch)    # Direct pitch mapping
        head_cmd.angular.y = float(pan)      # Pan mapping (will be scaled in hardware interface)
        head_cmd.angular.z = float(tilt)     # Tilt mapping (will be scaled in hardware interface)
        
        # Publish head command
        self.head_cmd_pub.publish(head_cmd)
        
        self.get_logger().debug(f'Published head command: pitch={pitch:.2f}, pan={pan:.2f}, tilt={tilt:.2f}')
    
    def process_base_command(self, state, angular):
        """Convert VR base values to ROS Twist message for base control"""
        base_cmd = Twist()
        
        # Based on the Arduino base code analysis:
        # state: 0=idle, 1=little push, 2=timing cycle, 3=atomic movement
        # angular: -1=left, 0=forward, 1=right (for state 3)
        
        if state == 0:
            # Idle state - no movement
            base_cmd.linear.x = 0.0
            base_cmd.angular.z = 0.0
        elif state == 1:
            # Little push movement (case 1) - small forward movement
            base_cmd.linear.x = 1.0  # Small forward movement to trigger case 1
            base_cmd.angular.z = 0.0
        elif state == 2:
            # Timing cycle (case 2) - synchronization with leg
            base_cmd.linear.x = 2.0  # Special value to trigger case 2
            base_cmd.angular.z = 0.0
        elif state == 3:
            # Atomic movement (case 3)
            base_cmd.linear.x = 3.0  # Special value to trigger case 3
            if angular == -1:
                base_cmd.angular.z = -1.0  # Left rotation
            elif angular == 1:
                base_cmd.angular.z = 1.0   # Right rotation
            else:
                base_cmd.angular.z = 0.0   # Forward movement
        else:
            # Invalid state, default to idle
            self.get_logger().warn(f'Invalid base state: {state}, defaulting to idle')
            base_cmd.linear.x = 0.0
            base_cmd.angular.z = 0.0

        
        if self.latest_command != state:
            # Publish base command
            self.cmd_vel_pub.publish(base_cmd)
            self.latest_command = state
    
            self.get_logger().debug(f'Published base command: state={state}, angular={angular}, '
                                f'linear.x={base_cmd.linear.x:.1f}, angular.z={base_cmd.angular.z:.1f}')
    
    def process_audio_command(self, volume, orientation):
        """Convert VR audio values to ROS message for audio control"""
        audio_msg = Float32MultiArray()
        
        # Pack volume (int) and orientation (float) into the audio message
        # The audio system can interpret these values as needed
        audio_msg.data = [float(volume), float(orientation)]
        
        # Publish audio command
        self.vr_audio_pub.publish(audio_msg)
        
        self.get_logger().debug(f'Published audio command: volume={volume}, orientation={orientation:.2f}')
    
    def pose_callback(self, msg):
        """Send robot pose to VR system and check for lost odometry"""        
        # Store the latest robot pose
        self.latest_robot_pose = msg

    def mic_audio_callback(self, msg):
        """Process audio data from microphone and send to VR system"""
        # Convert ROS message to VR-compatible format
        # Store the audio volume using proper RMS calculation
        if len(msg.data) > 0:
            # Calculate RMS (Root Mean Square) for proper volume measurement
            import math
            rms = math.sqrt(sum(sample * sample for sample in msg.data) / len(msg.data))
            
            # Scale RMS to 0-100 range with adjusted scaling factor
            # If current volume ~9 should be ~25, we need to scale up by factor of ~2.8
            # Adjusted scaling: divide by smaller factor to get higher output values
            volume = int(min(rms / 120.0, 100))  # Adjusted from 327.67 to 120 for better scaling
            
            self.get_logger().debug(f"volume last is {volume} (RMS: {rms:.1f})")
            self.latest_audio_volume = min(max(volume, 0), 100)  # Clamp to 0-100
            
        
    # Human skeleton callbacks
    def human_skeleton_callback(self, msg):
        """Process human skeleton visualization data"""
        self.human_skeleton = msg
        
        # Log at debug level to reduce verbosity
        self.get_logger().debug(f'VR: Human skeleton visualization received with {len(msg.markers)} markers')
        
        # Send the skeleton visualization data to the VR system
        # This would be implemented based on the specific VR system
    
    def human_skeleton_poses_callback(self, msg):
        """Process human skeleton poses data (PoseArray)"""
        self.latest_skeleton_data = msg
        
        # Log at debug level to reduce verbosity
        received_joints = len(msg.poses)
        self.get_logger().debug(f'VR: Human skeleton poses received with {received_joints}/{self.num_skeleton_joints} joints')
        
        # Debug: Log which joints are missing if any
        if received_joints < self.num_skeleton_joints:
            missing_joints = [self.skeleton_joint_names[i] for i in range(received_joints, self.num_skeleton_joints)]
            self.get_logger().debug(f'Missing joints will be set to (0,0,0): {missing_joints}')
    
    def send_data_to_vr(self):
        """Send robot pose, audio, and skeleton data to VR system via UDP"""
        if not self.udp_socket:
            return
            
        try:
            # Create the data packet
            data_packet = self.create_vr_data_packet()
            
            if data_packet:
                # Send the packet to the configured VR address/port
                send_address = (self.udp_send_address, self.udp_send_port)
                self.udp_socket.sendto(data_packet, send_address)
                self.pose_packets_sent += 1
                self.get_logger().debug(f'Sent {len(data_packet)} bytes to VR client at {send_address}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to send data to VR system: {str(e)}')
    
    def create_vr_data_packet(self):
        """Create a binary data packet containing robot pose and audio data (no skeleton)"""
        try:
            # Increment message order counter with overflow handling
            self.outgoing_message_order = (self.outgoing_message_order + 1) % (2**31)
            
            # Initialize data arrays
            data_ints = []
            data_floats = []
            
            # 1. Message order (1 int) - FIRST in the packet
            data_ints.append(self.outgoing_message_order)
            
            # 2. Robot pose (4 floats: x, y, z, w)
            if self.latest_robot_pose:
                pos = self.latest_robot_pose.pose.position
                ori = self.latest_robot_pose.pose.orientation
                data_floats.extend([pos.x, pos.y, ori.z, ori.w])
            else:
                # Default values if no pose available
                data_floats.extend([0.0, 0.0, 0.0, 1.0])
            
            # 3. Audio volume (1 int)
            data_ints.append(self.latest_audio_volume)
            
            # Create binary packet
            # Format: 2 ints (message_order + audio) + 4 floats (pose)
            packet_format = f'{len(data_ints)}i{len(data_floats)}f'
            packet_data = data_ints + data_floats
            
            # Pack the data
            data_packet = struct.pack(packet_format, *packet_data)
            
            self.get_logger().debug(f'Created VR packet: order={self.outgoing_message_order}, '
                                  f'{len(data_ints)} ints, {len(data_floats)} floats, '
                                  f'total {len(data_packet)} bytes')
            
            return data_packet
            
        except Exception as e:
            self.get_logger().error(f'Failed to create VR data packet: {str(e)}')
            return None
    
    def send_skeleton_to_vr(self):
        """Send skeleton data to VR system via UDP on separate port"""
        if not self.udp_socket:
            return
            
        try:
            # Create the skeleton data packet
            skeleton_packet = self.create_skeleton_data_packet()
            
            if skeleton_packet:
                # Send the packet to the configured skeleton address/port
                skeleton_address = (self.udp_skeleton_address, self.udp_skeleton_port)
                self.udp_socket.sendto(skeleton_packet, skeleton_address)
                self.skeleton_packets_sent += 1
                self.get_logger().debug(f'Sent {len(skeleton_packet)} bytes to skeleton port at {skeleton_address}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to send skeleton data to VR system: {str(e)}')
    
    def create_skeleton_data_packet(self):
        """Create a binary data packet containing only skeleton data"""
        try:
            # Increment skeleton message order counter with overflow handling
            self.skeleton_message_order = (self.skeleton_message_order + 1) % (2**31)
            
            # Initialize data arrays
            data_ints = []
            data_floats = []
            
            # 1. Message order (1 int) - FIRST in the packet
            data_ints.append(self.skeleton_message_order)
            
            # 2. Skeleton data (always 17 joints * 3 coordinates = 51 floats)
            skeleton_floats = []
            
            if self.latest_skeleton_data and len(self.latest_skeleton_data.poses) > 0:
                # Create a mapping of available poses by index
                available_poses = {}
                for i, pose in enumerate(self.latest_skeleton_data.poses):
                    if i < self.num_skeleton_joints:  # Only use up to 17 joints
                        available_poses[i] = pose
                
                # Always send exactly 17 joints in COCO order
                for joint_idx in range(self.num_skeleton_joints):
                    if joint_idx in available_poses:
                        # Joint is available, use its coordinates
                        pose = available_poses[joint_idx]
                        skeleton_floats.extend([
                            pose.position.x,
                            pose.position.y, 
                            pose.position.z
                        ])
                    else:
                        # Joint is missing, send zeros
                        skeleton_floats.extend([0.0, 0.0, 0.0])
            else:
                # No skeleton data available, send zeros for all 17 joints
                for _ in range(self.num_skeleton_joints):
                    skeleton_floats.extend([0.0, 0.0, 0.0])
            
            # Verify we always have exactly 51 floats (17 joints * 3 coordinates)
            assert len(skeleton_floats) == self.num_skeleton_joints * 3, f"Expected {self.num_skeleton_joints * 3} skeleton floats, got {len(skeleton_floats)}"
            

            # Add skeleton data to main data array
            data_floats.extend(skeleton_floats)
            
            # Create binary packet
            # Format: 1 int (message_order) + N*3 floats (skeleton)
            packet_format = f'{len(data_ints)}i{len(data_floats)}f'
            packet_data = data_ints + data_floats
            
            # Pack the data
            data_packet = struct.pack(packet_format, *packet_data)
            
            self.get_logger().debug(f'Created skeleton packet: order={self.skeleton_message_order}, '
                                  f'{len(data_ints)} ints, {len(data_floats)} floats ({self.num_skeleton_joints} joints), '
                                  f'total {len(data_packet)} bytes')
            
            return data_packet
            
        except Exception as e:
            self.get_logger().error(f'Failed to create skeleton data packet: {str(e)}')
            return None
    
    def check_communication_rates(self):
        """Monitor communication rates and log status every 10 seconds"""
        import time
        current_time = time.time()
        
        # Initialize monitoring log time if not set
        if self.last_monitoring_log_time is None:
            self.last_monitoring_log_time = current_time
            self.last_send_log_time = current_time
            return
        
        # Check for VR disconnection based on timeout
        if self.last_received_time is not None:
            time_since_last = current_time - self.last_received_time
            if time_since_last > self.vr_disconnection_timeout:
                if self.is_vr_connected:
                    # VR just disconnected
                    self.get_logger().warn(f'VR system disconnected - no data for {time_since_last:.1f}s')
                    self.is_vr_connected = False
                    # Reset message order counter for next reconnection
                    self.last_message_order = -1
            
        # Check if it's time to log (every 10 seconds)
        monitoring_interval = self.get_parameter('monitoring_log_interval_sec').get_parameter_value().double_value
        
        if current_time - self.last_monitoring_log_time >= monitoring_interval:
            # Check incoming data rate
            expected_rate = self.get_parameter('expected_receive_rate_hz').get_parameter_value().double_value
            
            if self.last_received_time is None:
                self.get_logger().error('No VR command data received yet - check VR system connection')
            else:
                time_since_last = current_time - self.last_received_time
                if time_since_last > (2.0 / expected_rate):  # Allow some tolerance
                    self.get_logger().error(f'No VR command data received for {time_since_last:.1f}s '
                                          f'(expected rate: {expected_rate}Hz)')
                else:
                    # Calculate actual receive rate over the monitoring period
                    time_period = current_time - self.last_monitoring_log_time
                    actual_rate = self.received_message_count / time_period if time_period > 0 else 0
                    self.get_logger().info(f'VR communication healthy - receiving at {actual_rate:.1f}Hz '
                                         f'(expected: {expected_rate}Hz)')
            
            # Log successful send statistics
            if current_time - self.last_send_log_time >= monitoring_interval:
                time_period = current_time - self.last_send_log_time
                pose_rate = self.pose_packets_sent / time_period if time_period > 0 else 0
                skeleton_rate = self.skeleton_packets_sent / time_period if time_period > 0 else 0
                
                self.get_logger().info(f'Successfully sending data - Pose: {pose_rate:.1f}Hz, '
                                     f'Skeleton: {skeleton_rate:.1f}Hz '
                                     f'(Total packets: pose={self.pose_packets_sent}, skeleton={self.skeleton_packets_sent})')
                
                # Reset counters
                self.pose_packets_sent = 0
                self.skeleton_packets_sent = 0
                self.last_send_log_time = current_time
            
            # Reset monitoring counters
            self.received_message_count = 0
            self.last_monitoring_log_time = current_time


def main(args=None):
    rclpy.init(args=args)
    vr_interface_node = VRInterfaceNode()
    
    try:
        rclpy.spin(vr_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup UDP resources
        vr_interface_node.is_running = False
        if vr_interface_node.udp_socket:
            vr_interface_node.udp_socket.close()
        vr_interface_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

