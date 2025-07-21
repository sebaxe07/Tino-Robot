import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
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
            PoseStamped, '/vr_in/robot_pose', 10)
            
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
        
        # Subscribe to UWB position data
        self.uwb_position_sub = self.create_subscription(
            Pose,
            '/UWB/Pos',
            self.uwb_position_callback,
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
        
        # Store UWB position information
        self.current_uwb_position = None
        self.uwb_position_received = False
        
        # Store RTAB-Map orientation information
        self.current_rtab_orientation = None
        self.rtab_orientation_received = False
        
        # Store last 5 positions for tracking validation and orientation estimation
        self.position_history = []
        self.max_history_size = 5
        self.rtab_orientation_lost = False
        self.position_tolerance = 1.0  # ±1 meter tolerance for x and y
        
        # Temporary orientation estimation when RTAB-Map is lost
        self.estimated_orientation = None
        self.last_valid_rtab_orientation = None
        self.orientation_estimation_active = False
        
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
        
        self.get_logger().info(f'Robot controller node initialized with UWB positioning, RTAB-Map orientation (with movement-based fallback), audio, and human skeleton tracking (skeleton publish rate: {1.0/self.skeleton_publish_rate:.1f}Hz)')
        
    def pose_callback(self, msg):
        """Process incoming localization pose data"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Check for the specific values indicating lost odometry
        if (orientation.x == 1.0 and orientation.y == 0.0 and orientation.z == 0.0 and orientation.w == 0.0):
            self.get_logger().warn('Detected RTAB-Map orientation loss! Using estimated orientation from movement direction...')
            self.rtab_orientation_lost = True
            self.orientation_estimation_active = True
            self.reset_odometry()
            
            # Don't return - we still want to process position and use estimated orientation
        
        # If RTAB-Map orientation was lost, check if it has recovered
        if self.rtab_orientation_lost:
            if self._is_orientation_valid(orientation):
                self.get_logger().info('RTAB-Map orientation recovered! Switching back to RTAB-Map orientation.')
                self.rtab_orientation_lost = False
                self.orientation_estimation_active = False
                # Store the recovered orientation as our new baseline
                self.last_valid_rtab_orientation = orientation
            else:
                self.get_logger().debug('RTAB-Map orientation still invalid, continuing with estimated orientation')
        
        # Store position in history (before rotation) - always do this for UWB and orientation estimation
        # Use UWB position if available for more accurate movement tracking
        if self.uwb_position_received and self.current_uwb_position is not None:
            self._update_position_history(self.current_uwb_position)
        else:
            self._update_position_history(position)
        
        # Store RTAB-Map orientation for sensor fusion (if valid)
        if not self.rtab_orientation_lost:
            self.current_rtab_orientation = orientation
            self.last_valid_rtab_orientation = orientation
            self.rtab_orientation_received = True
        
        # Apply rotation to align with map orientation (only if using RTAB-Map data)
        if not self.rtab_orientation_lost:
            rotated_msg = self._apply_rotation_to_pose(msg, 11.5)
        else:
            # Create a dummy message for consistent processing
            rotated_msg = msg
        
        # Create fused pose using UWB position and appropriate orientation
        fused_msg = self._create_fused_pose(rotated_msg)
        
        self.current_pose = fused_msg
        self.pose_received = True

        # Log first pose received and then periodically
        if not hasattr(self, 'pose_count'):
            self.pose_count = 0
            orientation_source = "estimated from movement" if self.orientation_estimation_active else "RTAB-Map"
            self.get_logger().info(f'First localization pose received (with UWB position and {orientation_source} orientation)')
            ## Log the initial pose
            pos = fused_msg.pose.position
            ori = fused_msg.pose.orientation
            self.get_logger().info(
                f'Initial Fused Pose: Position ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), '
                f'Orientation ({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f}) - Source: {orientation_source}'
            )
        
        self.pose_count += 1

        self.pose_pub.publish(fused_msg)
    
    def uwb_position_callback(self, msg):
        """Process incoming UWB position data"""
        self.current_uwb_position = msg.position
        self.uwb_position_received = True
        
        # Log UWB position information
        pos = msg.position
        self.get_logger().debug(
            f'UWB Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})'
        )
    
    def _create_fused_pose(self, rtab_pose_msg):
        """
        Create a fused pose using UWB position and appropriate orientation
        
        Args:
            rtab_pose_msg: PoseWithCovarianceStamped from RTAB-Map (already rotated if valid)
            
        Returns:
            PoseStamped: Fused pose message
        """
        # Create a simplified pose message
        fused_msg = PoseStamped()
        fused_msg.header = rtab_pose_msg.header
        
        # Determine which orientation to use
        if self.orientation_estimation_active:
            # Use estimated orientation from movement direction
            estimated_orientation = self._estimate_orientation_from_movement()
            if estimated_orientation is not None:
                fused_msg.pose.orientation = estimated_orientation
                orientation_source = "movement estimation"
            elif self.last_valid_rtab_orientation is not None:
                # Fall back to last known valid orientation
                fused_msg.pose.orientation = self.last_valid_rtab_orientation
                orientation_source = "last valid RTAB-Map"
            else:
                # Default to identity quaternion
                fused_msg.pose.orientation.x = 0.0
                fused_msg.pose.orientation.y = 0.0
                fused_msg.pose.orientation.z = 0.0
                fused_msg.pose.orientation.w = 1.0
                orientation_source = "identity (no data)"
        else:
            # Use RTAB-Map orientation (already rotated)
            fused_msg.pose.orientation = rtab_pose_msg.pose.pose.orientation
            orientation_source = "RTAB-Map"
        
        # Use UWB position if available, otherwise fall back to RTAB-Map position
        if self.uwb_position_received and self.current_uwb_position is not None:
            # Use UWB position
            fused_msg.pose.position.x = self.current_uwb_position.x
            fused_msg.pose.position.y = self.current_uwb_position.y
            fused_msg.pose.position.z = self.current_uwb_position.z
            
            # Log fusion info (only occasionally to avoid spam)
            if hasattr(self, 'fusion_log_counter'):
                self.fusion_log_counter += 1
            else:
                self.fusion_log_counter = 0
                
            if self.fusion_log_counter % 50 == 0:  # Log every 50th message (every 5 seconds at 10Hz)
                self.get_logger().info(
                    f'Sensor Fusion: Using UWB position ({self.current_uwb_position.x:.2f}, '
                    f'{self.current_uwb_position.y:.2f}, {self.current_uwb_position.z:.2f}) '
                    f'with {orientation_source} orientation'
                )
        else:
            # Fall back to RTAB-Map position
            fused_msg.pose.position = rtab_pose_msg.pose.pose.position
            
            if hasattr(self, 'fallback_log_counter'):
                self.fallback_log_counter += 1
            else:
                self.fallback_log_counter = 0
                
            if self.fallback_log_counter % 50 == 0:  # Log every 50th message
                self.get_logger().warn(f'Sensor Fusion: UWB position not available, using RTAB-Map position with {orientation_source} orientation')
        
        return fused_msg
    
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
    
    def _is_orientation_valid(self, orientation):
        """
        Check if the given orientation is valid (not the lost odometry indicator)
        
        Args:
            orientation: geometry_msgs.msg.Quaternion - The orientation to validate
            
        Returns:
            bool: True if orientation is valid, False if it indicates lost odometry
        """
        # Check for the specific values indicating lost odometry
        return not (orientation.x == 1.0 and orientation.y == 0.0 and 
                   orientation.z == 0.0 and orientation.w == 0.0)
    
    def _estimate_orientation_from_movement(self):
        """
        Estimate orientation from recent movement direction
        
        Returns:
            geometry_msgs.msg.Quaternion: Estimated orientation quaternion, or None if not enough data
        """
        if len(self.position_history) < 2:
            self.get_logger().debug('Not enough position history for orientation estimation')
            return None
            
        # Get the most recent positions for movement calculation
        current_pos = self.position_history[-1]
        previous_pos = self.position_history[-2]
        
        # Calculate movement vector
        dx = current_pos[0] - previous_pos[0]
        dy = current_pos[1] - previous_pos[1]
        
        # Check if there's significant movement
        movement_magnitude = math.sqrt(dx*dx + dy*dy)
        if movement_magnitude < 0.05:  # Less than 5cm movement
            self.get_logger().debug(f'Insufficient movement for orientation estimation: {movement_magnitude:.3f}m')
            return None
            
        # Calculate yaw angle from movement direction
        yaw = math.atan2(dy, dx)
        
        # Convert yaw to quaternion (rotation around Z-axis)
        from geometry_msgs.msg import Quaternion
        estimated_quat = Quaternion()
        estimated_quat.x = 0.0
        estimated_quat.y = 0.0
        estimated_quat.z = math.sin(yaw / 2.0)
        estimated_quat.w = math.cos(yaw / 2.0)
        
        self.get_logger().debug(
            f'Estimated orientation from movement: dx={dx:.3f}, dy={dy:.3f}, '
            f'yaw={math.degrees(yaw):.1f}°, quat=({estimated_quat.x:.3f}, {estimated_quat.y:.3f}, '
            f'{estimated_quat.z:.3f}, {estimated_quat.w:.3f})'
        )
        
        return estimated_quat

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
