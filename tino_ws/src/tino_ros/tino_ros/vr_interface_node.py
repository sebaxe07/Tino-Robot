import rclpy
from rclpy.node import Node
import rclpy.logging
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped, PoseArray
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
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('udp_bind_address', '0.0.0.0')
        
        log_level = self.get_parameter('log_level').get_parameter_value().string_value
        if hasattr(rclpy.logging, log_level):
            self.get_logger().set_level(getattr(rclpy.logging, log_level))
        
        # Subscribe to robot pose from controller
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/vr_in/robot_pose', 
            self.pose_callback,
            10)
            
        # Subscribe to human position data
        self.human_position_sub = self.create_subscription(
            PoseStamped,
            '/vr_in/human_position',
            self.human_position_callback,
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
        
        # UDP Socket for VR communication
        self.udp_socket = None
        self.udp_thread = None
        self.is_running = True
        
        # Message ordering tracking
        self.last_message_order = -1
        
        # Initialize VR communication
        self.setup_vr_communication()
        
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
        except Exception as e:
            self.get_logger().error(f'Failed to setup UDP communication: {str(e)}')
    
    def udp_listener(self):
        """Listen for UDP messages from VR system"""
        while self.is_running:
            try:
                self.get_logger().info('Waiting for UDP data...')
                data, addr = self.udp_socket.recvfrom(1024)
                self.get_logger().info(f'Received UDP data from {addr}')
                self.get_logger().info(f'Data length: {len(data)} bytes') 
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

            if len(data) == expected_size:  # 32 bytes total
                values = struct.unpack('fffiiffi', data)
                head_pan, head_pitch, head_tilt, base_state, base_angular, audio_volume, audio_orientation, message_order = values

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
        except Exception as e:
            self.get_logger().error(f'Error processing VR data: {str(e)}')
    
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
        
        # Publish base command
        self.cmd_vel_pub.publish(base_cmd)
        
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
        # Check for the specific pose values that indicate odometry loss
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

    def mic_audio_callback(self, msg):
        """Process audio data from microphone and send to VR system"""
        # Convert ROS message to VR-compatible format
        # This would be implemented based on the specific VR system
        pass
    
    def human_position_callback(self, msg):
        """Process human position data and send to VR system"""
        self.human_position = msg
        
        # Using debug level instead of info to reduce verbosity
        pos = msg.pose.position
        self.get_logger().debug(
            f'VR: Human position received: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})'
        )
        
        # Send the human position data to the VR system
        # This would be implemented based on the specific VR system
        
    # Human skeleton callbacks
    def human_skeleton_callback(self, msg):
        """Process human skeleton visualization data"""
        self.human_skeleton = msg
        
        # Log at debug level to reduce verbosity
        self.get_logger().debug(f'VR: Human skeleton visualization received with {len(msg.markers)} markers')
        
        # Send the skeleton visualization data to the VR system
        # This would be implemented based on the specific VR system
    
    def human_skeleton_poses_callback(self, msg):
        """Process human skeleton joint poses data"""
        self.human_skeleton_poses = msg
        
        # Log at debug level to reduce verbosity
        self.get_logger().debug(f'VR: Human skeleton poses received with {len(msg.poses)} joints')
        
        # This would be implemented based on the specific VR system
    
    def vr_command_callback(self, vr_data):
        """Process commands from VR system and publish to ROS topics"""
        # This method is now replaced by the UDP-based process_vr_data method
        # Keeping for backwards compatibility but VR commands now come via UDP
        pass


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

