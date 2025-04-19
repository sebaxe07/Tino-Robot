import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from evdev import InputDevice, ecodes, categorize
import threading
import time

class GamepadNode(Node):
    def __init__(self):
        super().__init__('gamepad_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'base_cmd_vel', 10)
        self.head_pub = self.create_publisher(Twist, 'head_cmd', 10)
        
        # Parameters with default values
        self.declare_parameter('gamepad_device', 
            '/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick')
        self.gamepad_device = self.get_parameter('gamepad_device').value
        
        # Initialize gamepad values
        self.bf = 0.0  # Base forward/backward
        self.bb = 0.0  # Base rotation
        self.hp = 0.0  # Head pitch
        self.hx = 511.0  # Head X
        self.hy = 511.0  # Head Y
        
        # Status flags
        self.is_running = True
        self.gamepad_connected = False
        
        # Set up publishing timer
        self.create_timer(0.04, self.publish_commands)  # 25Hz like original code
        
        # Start gamepad thread
        self.gamepad_thread = threading.Thread(target=self.read_gamepad)
        self.gamepad_thread.daemon = True
        self.gamepad_thread.start()
        
        self.get_logger().info(f"Gamepad node initialized, using device: {self.gamepad_device}")
    
    def map_range(self, value, in_min, in_max, out_min, out_max):
        """Map a value from one range to another"""
        return out_min + (((value - in_min) / (in_max - in_min)) * (out_max - out_min))
    
    def read_gamepad(self):
        """Thread function to read gamepad input"""
        while self.is_running:
            try:
                gamepad = InputDevice(self.gamepad_device)
                self.gamepad_connected = True
                self.get_logger().info("Gamepad connected")
                
                # Main loop for reading gamepad events
                for event in gamepad.read_loop():
                    if not self.is_running:
                        break
                        
                    if event.code == 0 and event.type == 0:
                        continue
                        
                    # Process gamepad events similar to legacy code
                    if event.code == 5:  # BF button
                        if 120 <= event.value <= 130:
                            self.bf = 0.0
                        else:
                            self.bf = self.map_range(event.value, 130, 255, 0, 0) if event.value > 130 else self.map_range(event.value, 0, 120, 15, 0)
                    
                    elif event.code == 309:  # BB button right
                        self.bb = -1.1 if event.value == 1 else 0.0
                    
                    elif event.code == 308:  # BB button left
                        self.bb = 1.1 if event.value == 1 else 0.0
                    
                    elif event.code == 0 and event.type == 3:  # HY
                        if 120 <= event.value <= 130:
                            self.hy = 511.0
                        else:
                            in_min = 130 if event.value < 120 else 0
                            in_max = 255 if event.value < 120 else 120
                            out_min = 512 if event.value < 120 else 0
                            out_max = 1024 if event.value < 120 else 511
                            self.hy = self.map_range(event.value, in_min, in_max, out_min, out_max)
                    
                    elif event.code == 1:  # HX
                        if 120 <= event.value <= 130:
                            self.hx = 511.0
                        else:
                            in_min = 130 if event.value < 120 else 0
                            in_max = 255 if event.value < 120 else 120
                            out_min = 512 if event.value < 120 else 0
                            out_max = 1024 if event.value < 120 else 511
                            self.hx = self.map_range(event.value, in_min, in_max, out_min, out_max)
                    
                    elif event.code == 17:  # HP
                        self.hp = -335 if event.value < 0 else (335 if event.value > 0 else 0)
                
                gamepad.close()
                
            except FileNotFoundError:
                self.gamepad_connected = False
                self.get_logger().warning(f"Gamepad not found at {self.gamepad_device}. Retrying in 1 second...")
                time.sleep(1)
                
            except Exception as e:
                self.gamepad_connected = False
                self.get_logger().error(f"Gamepad error: {e}. Retrying in 1 second...")
                time.sleep(1)
    
    def publish_commands(self):
        """Timer callback to publish command messages"""
        # Publish base velocity command
        base_cmd = Twist()
        base_cmd.linear.x = float(self.bf)  # Forward/backward
        base_cmd.angular.z = float(self.bb)  # Rotation
        self.cmd_vel_pub.publish(base_cmd)
        
        # Publish head command
        head_cmd = Twist()
        head_cmd.angular.x = float(self.hp)  # Head pitch
        head_cmd.angular.y = float(self.hy - 511.0) / 511.0  # Head Y normalized to [-1, 1]
        head_cmd.angular.z = float(self.hx - 511.0) / 511.0  # Head X normalized to [-1, 1]
        self.head_pub.publish(head_cmd)
        
    def destroy_node(self):
        self.is_running = False
        if self.gamepad_thread.is_alive():
            self.gamepad_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GamepadNode()
    
    try:
        print("Gamepad node running. Press Ctrl+C to exit...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down gamepad node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()