#!/usr/bin/env python3
# filepath: /home/orinano/Tino-Robot/tino_ws/src/tino_ros/tino_ros/audio_test_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class AudioTestNode(Node):
    def __init__(self):
        super().__init__('audio_test_node')
        
        # Default volume and orientation settings
        self.volume = 255.0
        self.orientation = 0.0
        
        # Subscribe to the head_cmd topic from the gamepad
        self.head_cmd_subscription = self.create_subscription(
            Twist,
            'head_cmd',
            self.head_cmd_callback,
            10)
        
        # Publisher for the audio control parameters
        self.audio_pub = self.create_publisher(
            Float32MultiArray, 
            '/audio/vr_output',
            10)
        
        # Create a timer to publish audio settings periodically
        self.timer = self.create_timer(0.1, self.publish_audio_settings)  # 10 Hz
        
        self.get_logger().info('Audio test node initialized')
        
    def head_cmd_callback(self, msg):
        """
        Process incoming head commands from the gamepad
        Map head movements to audio control parameters
        """
        
        try:
            # Use head_cmd.angular.y for volume control (vertical movement)
            # Map from -1.0 to 1.0 to 0 to 255
            # When stick is up (positive y), increase volume
            self.volume = 127.5 + (127.5 * msg.angular.y)
            self.volume = max(0.0, min(255.0, self.volume))
            
            # Use head_cmd.angular.z for orientation control (horizontal movement)
            # This is already normalized to [-1, 1] which is what we need
            self.orientation = msg.angular.z
            
            self.get_logger().debug(f'Updated audio settings: volume={self.volume:.1f}, orientation={self.orientation:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing head command: {str(e)}')
    
    def publish_audio_settings(self):
        """
        Publish the current audio settings
        """
        try:
            msg = Float32MultiArray()
            msg.data = [float(self.volume), float(self.orientation)]
            self.audio_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing audio settings: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    audio_test_node = AudioTestNode()
    
    try:
        print("Audio test node running. Press Ctrl+C to exit...")
        rclpy.spin(audio_test_node)
    except KeyboardInterrupt:
        print("Audio test node shutting down...")
    finally:
        audio_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
