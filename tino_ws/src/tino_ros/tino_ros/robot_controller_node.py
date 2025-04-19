import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import numpy as np

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        
        # Publishers for each robot component
        self.base_pub = self.create_publisher(Twist, 'base_cmd_vel', 10)
        self.head_pub = self.create_publisher(Twist, 'head_cmd', 10)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot_pose', 10)
       
        # Subscribe to localization pose from rtabmap
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose', 
            self.pose_callback,
            10)
        
        # Subscribe to VR commands
        self.vr_cmd_sub = self.create_subscription(
            Twist, 'vr_cmd_vel', self.vr_cmd_callback, 10)
        self.vr_head_cmd_sub = self.create_subscription(
            Twist, 'vr_head_cmd', self.vr_head_cmd_callback, 10)
        
        # Store the latest pose information
        self.current_pose = None
        self.pose_received = False
        
        # Create timer for controller update
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Robot controller node initialized with localization')
        
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
    
    def vr_cmd_callback(self, msg):
        # Process and forward VR commands to hardware
        self.base_pub.publish(msg)

    def vr_head_cmd_callback(self, msg):
        # Process and forward VR head commands
        self.head_pub.publish(msg)

    def control_loop(self):
        """Main control loop that processes localization and sends commands"""
        if not self.pose_received:
            # No pose data yet
            return
            
        # For now, just keep track of the pose data
        # In the future, this will process VR input and generate commands
        
        # Example of sending data from here (currently empty commands)
        # This would be replaced with actual commands from VR system
        base_cmd = Twist()
        head_cmd = Twist()
        
        # When VR system is integrated, commands would be set here
        # base_cmd.linear.x = vr_forward_command
        # base_cmd.angular.z = vr_rotation_command
        
        # self.base_pub.publish(base_cmd)
        # self.head_pub.publish(head_cmd)

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
