import rclpy
from rclpy.node import Node
import rclpy.logging
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped, PoseArray
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray
# Import any VR-specific libraries you'll need

class VRInterfaceNode(Node):
    def __init__(self):
        super().__init__('vr_interface_node')
        
        # Set logging level to reduce verbosity (ERROR, WARN, INFO, DEBUG)
        # Default to INFO, but can be changed with ROS param
        self.declare_parameter('log_level', 'INFO')
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
        
        # Initialize VR communication
        self.setup_vr_communication()
        
        self.get_logger().info('VR interface node initialized with human skeleton tracking')
    
    def setup_vr_communication(self):
        """Set up communication with VR system"""
        # This would be implemented based on the specific VR system
        pass
    
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
        # Convert VR commands to ROS messages
        base_cmd = Twist()
        head_cmd = Twist()
        
        # Fill in the commands based on VR data
        # base_cmd.linear.x = vr_data.forward
        # base_cmd.angular.z = vr_data.rotation
        
        # Publish commands
        self.cmd_vel_pub.publish(base_cmd)
        self.head_cmd_pub.publish(head_cmd)


def main(args=None):
    rclpy.init(args=args)
    vr_interface_node = VRInterfaceNode()
    
    try:
        rclpy.spin(vr_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        vr_interface_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

