import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, PoseArray
from visualization_msgs.msg import MarkerArray
import numpy as np
import subprocess  # Add this import for audio setup

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        
        # Check audio setup on startup
        self._check_audio_setup()
        
        # Declare parameters
        self.declare_parameter('skeleton_publish_rate_hz', 5.0)  # Default 10Hz
        
        # Publishers for each robot component
        self.base_pub = self.create_publisher(Twist, 'base_cmd_vel', 10)
        self.head_pub = self.create_publisher(Twist, 'head_cmd', 10)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/vr_in/robot_pose', 10)
            
        # Publisher for human position to VR interface
        self.human_position_pub = self.create_publisher(
            PoseStamped, '/vr_in/human_position', 10)
            
        # Publishers for human skeleton data to VR interface
        self.human_skeleton_pub = self.create_publisher(
            MarkerArray, '/vr_in/human_skeleton', 10)
        self.human_skeleton_poses_pub = self.create_publisher(
            PoseArray, '/vr_in/human_skeleton_poses', 10)
       
        # Subscribe to localization pose from rtabmap
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose', 
            self.pose_callback,
            10)
        
        # Subscribe to human position from pose detection
        self.human_position_sub = self.create_subscription(
            PoseStamped,
            '/human_position',
            self.human_position_callback,
            10)
            
        # Subscribe to human skeleton data from pose detection
        self.human_skeleton_sub = self.create_subscription(
            MarkerArray,
            '/human_skeleton',
            self.human_skeleton_callback,
            10)
        self.human_skeleton_poses_sub = self.create_subscription(
            PoseArray,
            '/human_skeleton_poses',
            self.human_skeleton_poses_callback,
            10)
        
        # Subscribe to VR commands
        self.vr_cmd_sub = self.create_subscription(
            Twist, '/vr_out/cmd_vel', self.vr_cmd_callback, 10)
        self.vr_head_cmd_sub = self.create_subscription(
            Twist, '/vr_out/head_cmd', self.vr_head_cmd_callback, 10)
        
        # Audio publishers and subscribers
        self.mic_audio_sub = self.create_subscription(
            Int16MultiArray, '/audio/mic_input', self.mic_audio_callback, 10)
        self.vr_audio_sub = self.create_subscription(
            Int16MultiArray, '/vr_out/audio_input', self.vr_audio_callback, 10)
        self.vr_audio_pub = self.create_publisher(
            Int16MultiArray, '/vr_in/audio_output', 10)
        self.robot_audio_pub = self.create_publisher(
            Int16MultiArray, '/audio/vr_output', 10)
        
        # Store the latest pose information
        self.current_pose = None
        self.pose_received = False
        
        # Store the latest human position information
        self.human_position = None
        self.human_detected = False
        
        # Store skeleton data
        self.human_skeleton = None
        self.human_skeleton_poses = None
        
        # Create timer for controller update
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Set up skeleton publishing rate from parameter
        rate_hz = self.get_parameter('skeleton_publish_rate_hz').value
        if rate_hz <= 0:
            self.get_logger().warn(f"Invalid skeleton_publish_rate_hz parameter: {rate_hz}. Using default of 10Hz.")
            rate_hz = 10.0
        self.skeleton_publish_rate = 1.0 / rate_hz
        self.skeleton_timer = self.create_timer(self.skeleton_publish_rate, self.publish_skeleton_data)
        
        self.get_logger().info(f'Robot controller node initialized with localization, audio, and human skeleton tracking (skeleton publish rate: {1.0/self.skeleton_publish_rate:.1f}Hz)')
        
    def pose_callback(self, msg):
        """Process incoming localization pose data"""
        self.current_pose = msg
        self.pose_received = True
        
        # Log first pose received and then periodically
        if not hasattr(self, 'pose_count'):
            self.pose_count = 0
            self.get_logger().info('First localization pose received')
            ## Log the initial pose
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            self.get_logger().info(
                f'Initial Pose: Position ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), '
                f'Orientation ({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f})'
            )
        
        self.pose_count += 1
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.get_logger().info(
            f'Pose update: Position ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), '
            f'Orientation ({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f})'
        )

        self.pose_pub.publish(msg)
    
    def human_position_callback(self, msg):
        """Process incoming human position data"""
        self.human_position = msg
        self.human_detected = True
        
        # Log human position information
        pos = msg.pose.position
        # self.get_logger().info(
        #     f'Human detected: Position ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})'
        # )
        
        # Forward the human position to the VR interface
        self.human_position_pub.publish(msg)
    
    def vr_cmd_callback(self, msg):
        # Process and forward VR commands to hardware
        self.base_pub.publish(msg)

    def vr_head_cmd_callback(self, msg):
        # Process and forward VR head commands
        self.head_pub.publish(msg)

    def mic_audio_callback(self, msg):
        """Forward microphone audio to VR"""
        self.vr_audio_pub.publish(msg)
        
    def vr_audio_callback(self, msg):
        """Forward VR audio to audio node for playback"""
        self.robot_audio_pub.publish(msg)
        
    # Skeleton callbacks
    def human_skeleton_callback(self, msg):
        """Process incoming human skeleton visualization data"""
        self.human_skeleton = msg
        # Store latest data but don't publish immediately
        self.get_logger().debug('Received new human skeleton visualization data')
    
    def human_skeleton_poses_callback(self, msg):
        """Process incoming human skeleton joint poses"""
        self.human_skeleton_poses = msg
        # Store latest data but don't publish immediately
        self.get_logger().debug(f'Received new human skeleton poses data: {len(msg.poses)} joints')
        
    def publish_skeleton_data(self):
        """Publish the latest skeleton data at the controlled rate"""
        # Publish skeleton visualization if available
        if self.human_skeleton is not None:
            self.human_skeleton_pub.publish(self.human_skeleton)
            self.get_logger().debug('Published human skeleton visualization to VR at controlled rate')
            
        # Publish skeleton poses if available
        if self.human_skeleton_poses is not None:
            self.human_skeleton_poses_pub.publish(self.human_skeleton_poses)
            self.get_logger().debug(f'Published human skeleton poses to VR at controlled rate: {len(self.human_skeleton_poses.poses)} joints')

    def control_loop(self):
        """Main control loop that processes localization and sends commands"""
        if not self.pose_received:
            # No pose data yet
            return
            
        # You could implement more complex control logic here
        # For example, processing both robot pose and human position
        # to control the robot's movement based on the human's location
        
        # For now, we're just forwarding the data between systems

    def set_skeleton_publish_rate(self, rate_hz):
        """
        Set the rate for skeleton data publishing
        
        Args:
            rate_hz (float): The publishing rate in Hz
        """
        if rate_hz <= 0:
            self.get_logger().error(f"Invalid publishing rate: {rate_hz}Hz. Must be positive.")
            return False
            
        # Create a new timer with the updated rate
        self.skeleton_publish_rate = 1.0 / rate_hz
        
        # Destroy existing timer if present
        if hasattr(self, 'skeleton_timer') and self.skeleton_timer is not None:
            self.skeleton_timer.cancel()
            
        # Create new timer with updated rate
        self.skeleton_timer = self.create_timer(self.skeleton_publish_rate, self.publish_skeleton_data)
        self.get_logger().info(f"Updated skeleton publishing rate to {rate_hz:.1f}Hz")
        return True

    def _check_audio_setup(self):
        """Check if audio devices are properly set up, and run setup script if needed"""
        try:
            # Check if the USB audio device is the default
            self.get_logger().info('Checking audio devices on startup')
            
            # Run a simple check for USB audio devices
            result = subprocess.run(['aplay', '-l'], capture_output=True, text=True)
            usb_audio = 'USB Audio Device' in result.stdout
            
            if usb_audio:
                self.get_logger().info('USB Audio Device detected')
                
                # Check PulseAudio status
                pulse_result = subprocess.run(['pactl', 'info'], capture_output=True, text=True)
                if 'Failed to connect' in pulse_result.stderr:
                    self.get_logger().warn('PulseAudio is not running, attempting to fix')
                    subprocess.run(['/home/orinano/Tino-Robot/setup_audio.sh'], shell=True)
            else:
                self.get_logger().warn('USB Audio device not found or not default. Audio features may not work correctly.')
                self.get_logger().warn('Consider running setup_audio.sh script')
        except Exception as e:
            self.get_logger().error(f'Error checking audio setup: {str(e)}')
            self.get_logger().warn('Audio features may not work correctly')

def main(args=None):
    rclpy.init(args=args)
    controller_node = RobotControllerNode()
    
    try:
        print("Robot controller node running with localization. Press Ctrl+C to exit...")
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        print("Robot controller shutting down...")
    finally:
        controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
