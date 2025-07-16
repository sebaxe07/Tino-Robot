import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, PoseArray
from visualization_msgs.msg import MarkerArray
import numpy as np
import subprocess  # Add this import for audio setup
import math
from std_srvs.srv import Empty

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        
        # Check audio setup on startup
        self._check_audio_setup()
        
        # Declare parameters
        self.declare_parameter('skeleton_publish_rate_hz', 10.0)  # Default 10Hz
        
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
        
        
        # Create a client for the reset odometry service
        self.reset_odom_client = self.create_client(Empty, '/reset_odom')
        
        # Subscribe to VR commands
        self.vr_cmd_sub = self.create_subscription(
            Twist, '/vr_out/cmd_vel', self.vr_cmd_callback, 10)
        self.vr_head_cmd_sub = self.create_subscription(
            Twist, '/vr_out/head_cmd', self.vr_head_cmd_callback, 10)
        
        # Audio publishers and subscribers
        self.mic_audio_sub = self.create_subscription(
            Int16MultiArray, '/audio/mic_input', self.mic_audio_callback, 10)
        self.vr_audio_sub = self.create_subscription(
            Float32MultiArray, '/vr_out/audio_input', self.vr_audio_callback, 10)
        self.vr_audio_pub = self.create_publisher(
            Int16MultiArray, '/vr_in/audio_output', 10)
        self.robot_audio_pub = self.create_publisher(
            Float32MultiArray, '/audio/vr_output', 10)
        
        # Store the latest pose information
        self.current_pose = None
        self.pose_received = False
        
        # Store last 5 positions for tracking validation
        self.position_history = []
        self.max_history_size = 5
        self.tracking_lost = False
        self.position_tolerance = 1.0  # ±1 meter tolerance for x and y
        
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
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Check for the specific values indicating lost odometry
        if (orientation.x == 1.0 and orientation.y == 0.0 and orientation.z == 0.0 and orientation.w == 0.0):
            self.get_logger().warn('Detected odometry loss! Attempting to reset odometry...')
            self.tracking_lost = True
            self.reset_odometry()
            return  # Don't process this pose
        
        # If tracking was lost, check if the new position is reasonable
        if self.tracking_lost:
            if not self._is_position_reasonable(position):
                self.get_logger().debug(f'Position not reasonable after tracking loss: ({position.x:.2f}, {position.y:.2f}), skipping...')
                return  # Don't process this pose
            else:
                self.get_logger().info('Tracking recovered! Position is reasonable, resuming pose publishing.')
                self.tracking_lost = False
        
        # Store position in history (before rotation)
        self._update_position_history(position)
        
        # Apply 60-degree rotation to align with map orientation
        rotated_msg = self._apply_rotation_to_pose(msg, 21.0)
        
        self.current_pose = rotated_msg
        self.pose_received = True

        # Log first pose received and then periodically
        if not hasattr(self, 'pose_count'):
            self.pose_count = 0
            self.get_logger().info('First localization pose received (with 20° rotation applied)')
            ## Log the initial pose
            pos = rotated_msg.pose.pose.position
            ori = rotated_msg.pose.pose.orientation
            self.get_logger().info(
                f'Initial Rotated Pose: Position ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), '
                f'Orientation ({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f})'
            )
        
        self.pose_count += 1
        pos = rotated_msg.pose.pose.position
        ori = rotated_msg.pose.pose.orientation
        # self.get_logger().info(
        #     f'Pose update: Position ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), '
        #     f'Orientation ({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f})'
        # )

        self.pose_pub.publish(rotated_msg)
    
    def _apply_rotation_to_pose(self, pose_msg, rotation_degrees):
        """
        Apply a rotation around the Z-axis to the pose
        
        Args:
            pose_msg: PoseWithCovarianceStamped message
            rotation_degrees: Rotation angle in degrees around Z-axis
            
        Returns:
            PoseWithCovarianceStamped: Rotated pose message
        """
        # Create a copy of the message
        rotated_msg = PoseWithCovarianceStamped()
        rotated_msg.header = pose_msg.header
        rotated_msg.pose.covariance = pose_msg.pose.covariance
        
        # Convert rotation angle to radians
        rotation_rad = math.radians(rotation_degrees)
        
        # Get original pose
        orig_pose = pose_msg.pose.pose
        
        # Apply rotation to position (rotate around Z-axis)
        cos_theta = math.cos(rotation_rad)
        sin_theta = math.sin(rotation_rad)
        
        # Rotate position
        rotated_msg.pose.pose.position.x = (orig_pose.position.x * cos_theta - 
                                           orig_pose.position.y * sin_theta)
        rotated_msg.pose.pose.position.y = (orig_pose.position.x * sin_theta + 
                                           orig_pose.position.y * cos_theta)
        rotated_msg.pose.pose.position.z = orig_pose.position.z
        
        # Create rotation quaternion for Z-axis rotation
        rotation_quat_z = math.sin(rotation_rad / 2.0)
        rotation_quat_w = math.cos(rotation_rad / 2.0)
        
        # Apply rotation to orientation (quaternion multiplication)
        # Original quaternion
        q1_x, q1_y, q1_z, q1_w = (orig_pose.orientation.x, orig_pose.orientation.y, 
                                   orig_pose.orientation.z, orig_pose.orientation.w)
        
        # Rotation quaternion (around Z-axis)
        q2_x, q2_y, q2_z, q2_w = 0.0, 0.0, rotation_quat_z, rotation_quat_w
        
        # Quaternion multiplication: result = q2 * q1
        rotated_msg.pose.pose.orientation.x = (q2_w * q1_x + q2_x * q1_w + 
                                              q2_y * q1_z - q2_z * q1_y)
        rotated_msg.pose.pose.orientation.y = (q2_w * q1_y - q2_x * q1_z + 
                                              q2_y * q1_w + q2_z * q1_x)
        rotated_msg.pose.pose.orientation.z = (q2_w * q1_z + q2_x * q1_y - 
                                              q2_y * q1_x + q2_z * q1_w)
        rotated_msg.pose.pose.orientation.w = (q2_w * q1_w - q2_x * q1_x - 
                                              q2_y * q1_y - q2_z * q1_z)
        
        return rotated_msg
    def reset_odometry(self):
        """Call the reset_odom service to reset the odometry"""
        # Check if service is available
        if not self.reset_odom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Reset odometry service not available')
            return
            
        # Create request
        request = Empty.Request()
        
        # Call service
        future = self.reset_odom_client.call_async(request)
        
        # Add callback for when the service call is completed
        future.add_done_callback(self.reset_odometry_callback)
    
    def reset_odometry_callback(self, future):
        """Callback for reset_odometry service response"""
        try:
            future.result()
            self.get_logger().info('Successfully reset odometry')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

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
            
            # Set a short timeout for all audio commands to prevent hanging
            try:
                # Run a simple check for USB audio devices with timeout
                result = subprocess.run(['aplay', '-l'], capture_output=True, text=True, timeout=3)
                usb_audio = 'USB Audio Device' in result.stdout
                
                self.get_logger().debug(f"Audio device check result: {'USB device found' if usb_audio else 'No USB audio found'}")
                
                if usb_audio:
                    self.get_logger().info('USB Audio Device detected')
                    
                    # Check PulseAudio status with timeout
                    try:
                        pulse_result = subprocess.run(['pactl', 'info'], capture_output=True, text=True, timeout=2)
                        if pulse_result.returncode != 0 or (hasattr(pulse_result, 'stderr') and 'Failed to connect' in pulse_result.stderr):
                            self.get_logger().warn('PulseAudio is not running or has issues, attempting to fix in background')
                            # Run the audio setup script in background to avoid blocking
                            subprocess.Popen(['/home/orinano/Tino-Robot/setup_audio.sh'], 
                                            shell=True, 
                                            start_new_session=True)
                    except subprocess.TimeoutExpired:
                        self.get_logger().warn('PulseAudio check timed out, running setup script in background')
                        subprocess.Popen(['/home/orinano/Tino-Robot/setup_audio.sh'], 
                                        shell=True, 
                                        start_new_session=True)
                else:
                    self.get_logger().warn('USB Audio device not found or not default. Audio features may not work correctly.')
                    self.get_logger().warn('Running setup_audio.sh script in background')
                    # Run in background to avoid blocking
                    subprocess.Popen(['/home/orinano/Tino-Robot/setup_audio.sh'], 
                                    shell=True, 
                                    start_new_session=True)
            except subprocess.TimeoutExpired:
                self.get_logger().error('Audio device check timed out')
                self.get_logger().warn('Running setup_audio.sh script in background to fix audio')
                # Run in background to avoid blocking
                subprocess.Popen(['/home/orinano/Tino-Robot/setup_audio.sh'], 
                                shell=True, 
                                start_new_session=True)
        except Exception as e:
            self.get_logger().error(f'Error checking audio setup: {str(e)}')
            self.get_logger().warn('Audio features may not work correctly')

    def _update_position_history(self, position):
        """
        Update the position history with the latest position
        
        Args:
            position: geometry_msgs.msg.Point - The position to add to history
        """
        # Add current position to history
        self.position_history.append((position.x, position.y))
        
        # Keep only the last max_history_size positions
        if len(self.position_history) > self.max_history_size:
            self.position_history.pop(0)
            
        self.get_logger().debug(f'Position history updated: {len(self.position_history)} positions stored')
    
    def _is_position_reasonable(self, position):
        """
        Check if the given position is reasonable compared to recent history
        
        Args:
            position: geometry_msgs.msg.Point - The position to validate
            
        Returns:
            bool: True if position is reasonable, False otherwise
        """
        if not self.position_history:
            # No history yet, accept the position
            self.get_logger().debug('No position history available, accepting position')
            return True
            
        # Check if the position is within tolerance of any recent position
        for hist_x, hist_y in self.position_history:
            x_diff = abs(position.x - hist_x)
            y_diff = abs(position.y - hist_y)
            
            if x_diff <= self.position_tolerance and y_diff <= self.position_tolerance:
                self.get_logger().debug(
                    f'Position ({position.x:.2f}, {position.y:.2f}) is reasonable '
                    f'(within {self.position_tolerance}m of ({hist_x:.2f}, {hist_y:.2f}))'
                )
                return True
        
        # Position is not close to any recent position
        self.get_logger().debug(
            f'Position ({position.x:.2f}, {position.y:.2f}) is NOT reasonable. '
            f'Recent positions: {self.position_history}'
        )
        return False

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
